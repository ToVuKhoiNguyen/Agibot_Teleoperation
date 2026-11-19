"""
teleop/robot/ros2_backend.py
ROS 2 (Jazzy) backend for AGIBOT A2 Ultra real robot execution with strict safety locks.
"""

import os
import time
import threading
import numpy as np
from teleop.robot.interface import RobotInterface
from teleop.robot.aimdk_map import (
    ARM_JOINT_COMMAND_TOPIC, HAND_JOINT_COMMAND_TOPIC,
    ARM_JOINT_STATE_TOPIC, HAND_JOINT_STATE_TOPIC,
    ARM_JOINT_NAMES_AIMDK, HAND_JOINT_NAMES, HAND_EFFORT_DEFAULT,
    HAND_FRAME_ID, HAND_EFFORT_RAW_MIN, HAND_EFFORT_RAW_MAX,
    sim_hand_to_robot_raw, robot_raw_to_sim_hand, align_joint_state_positions,
)


class Ros2Backend(RobotInterface):
    """ROS 2 Hardware backend communicating over AimDK DDS topics with 5 safety locks."""

    def __init__(
        self,
        arms: str = "both",
        no_hand: bool = False,
        real_max_vel: float = 0.3,
        i_accept_higher_speed: bool = False,
        hand_raw_max: int = 2000,
        enable_thumb_swing: bool = False,
        hand_effort: float = HAND_EFFORT_DEFAULT,
        allow_hardware_risk: bool = False,
        confirmation_input_fn=None,
    ):
        self.arms = arms.lower()  # "left", "right", or "both"
        self.no_hand = bool(no_hand)
        ceiling = 3.0 if i_accept_higher_speed else 1.0
        self.real_max_vel = min(float(real_max_vel), ceiling)
        self.hand_raw_max = min(int(hand_raw_max), 2000)
        self.enable_thumb_swing = bool(enable_thumb_swing)
        self.hand_effort = float(np.clip(hand_effort, HAND_EFFORT_RAW_MIN, HAND_EFFORT_RAW_MAX))
        self.allow_hardware_risk = bool(allow_hardware_risk)
        self.confirmation_input_fn = confirmation_input_fn or input

        self.connected = False
        self.holding = False
        self.estop_latched = False
        self.fault_latched = False
        self.fault_reason = ""

        self._target_arm_q14 = np.zeros(14, dtype=np.float64)
        self._target_hand_h12 = np.zeros(12, dtype=np.float64)
        self._latest_arm_q14 = np.zeros(14, dtype=np.float64)
        self._latest_hand_h12 = np.zeros(12, dtype=np.float64)
        self._last_arm_state_t = None
        self._last_hand_state_t = None

        self._node = self._pub_arm = self._pub_hand = None
        self._sub_arm = self._sub_hand = self._executor = None
        self._spin_thread = self._pub_thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

    def connect(self) -> None:
        """Enforces 5-point hardware safety interlock before initializing ROS 2."""
        # 1. Lock 1: CLI hardware risk acknowledgement
        if not self.allow_hardware_risk:
            raise RuntimeError("Hardware execution locked: missing --i-understand-hardware-risk flag.")
        # 2. Lock 2: Explicit environment variable authorization
        if os.environ.get("AGIBOT_ALLOW_REAL") != "1":
            raise RuntimeError("Hardware execution locked: AGIBOT_ALLOW_REAL != 1 in environment.")
        # 3. Lock 3: Exact confirmation phrase typed on stdin
        phrase = self.confirmation_input_fn("Type exact phrase 'GANTRY AND ESTOP READY' to connect hardware: ")
        if phrase.strip() != "GANTRY AND ESTOP READY":
            raise RuntimeError("Hardware execution locked: confirmation phrase mismatch.")

        # Lazy import of rclpy to preserve package zero-ROS import isolation
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("agibot_teleop_ros2_backend")
        self._pub_arm = self._node.create_publisher(JointState, ARM_JOINT_COMMAND_TOPIC, 10)
        if not self.no_hand:
            self._pub_hand = self._node.create_publisher(JointState, HAND_JOINT_COMMAND_TOPIC, 10)
        self._sub_arm = self._node.create_subscription(JointState, ARM_JOINT_STATE_TOPIC, self._on_arm_state, 10)
        self._sub_hand = self._node.create_subscription(JointState, HAND_JOINT_STATE_TOPIC, self._on_hand_state, 10)

        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._stop_event.clear()
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        # 4 & 5. Lock 4 & 5: State topics must already be publishing; first state within 1.0 s
        t0 = time.time()
        while time.time() - t0 < 1.0:
            if self._last_arm_state_t is not None:
                break
            time.sleep(0.05)

        if self._last_arm_state_t is None:
            self.disconnect()
            raise RuntimeError("Hardware execution locked: no robot arm_joint_state received within 1.0s (robot not alive).")

        with self._lock:
            self._target_arm_q14 = self._latest_arm_q14.copy()
            self._target_hand_h12 = self._latest_hand_h12.copy()

        self.connected = True
        self.holding = False
        self.estop_latched = False

        # Start steady 50 Hz publishing thread
        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()
        print(f"[Ros2Backend] Connected to real robot topics at 50 Hz (arms={self.arms}, no_hand={self.no_hand}).")

    def disconnect(self) -> None:
        self.connected = False
        self._stop_event.set()
        if self._executor is not None:
            self._executor.shutdown()
        for t in (self._spin_thread, self._pub_thread):
            if t is not None and t.is_alive():
                t.join(timeout=0.5)
        if self._node is not None:
            self._node.destroy_node()
        self._node = None
        print("[Ros2Backend] Disconnected.")

    def _on_arm_state(self, msg) -> None:
        if len(msg.position) >= 14:
            with self._lock:
                self._latest_arm_q14 = align_joint_state_positions(msg, ARM_JOINT_NAMES_AIMDK, 14)
                self._last_arm_state_t = time.time()

    def _on_hand_state(self, msg) -> None:
        frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
        if frame_id and frame_id != HAND_FRAME_ID:
            return  # Refuse / drop packets not matching vendor AgiHand frame_id
        if len(msg.position) >= 12:
            with self._lock:
                raw_p = align_joint_state_positions(msg, HAND_JOINT_NAMES, 12)
                self._latest_hand_h12 = robot_raw_to_sim_hand(raw_p)
                self._last_hand_state_t = time.time()

    def send_arm(self, q14: np.ndarray) -> None:
        if not self.connected or self.estop_latched or self.holding:
            return
        q = np.asarray(q14, dtype=np.float64)
        with self._lock:
            if self.arms == "left":
                self._target_arm_q14 = np.concatenate([q[:7], self._latest_arm_q14[7:]])
            elif self.arms == "right":
                self._target_arm_q14 = np.concatenate([self._latest_arm_q14[:7], q[7:]])
            else:
                self._target_arm_q14 = q.copy()

    def send_hand(self, h12: np.ndarray) -> None:
        if not self.connected or self.no_hand or self.estop_latched or self.holding:
            return
        with self._lock:
            self._target_hand_h12 = np.clip(np.asarray(h12, dtype=np.float64), 0.0, 1.0).copy()

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        with self._lock:
            return self._latest_arm_q14.copy(), self._latest_hand_h12.copy(), self._last_arm_state_t or time.time()

    def hold(self) -> None:
        """Keeps publishing CURRENT measured position continuously at 50 Hz."""
        self.holding = True
        with self._lock:
            self._target_arm_q14 = self._latest_arm_q14.copy()
            self._target_hand_h12 = self._latest_hand_h12.copy()

    def estop(self) -> None:
        self.estop_latched = True
        self.hold()

    def is_alive(self) -> bool:
        if not self.connected or self._last_arm_state_t is None:
            return False
        fresh = (time.time() - self._last_arm_state_t) <= 0.200
        if not fresh and not self.fault_latched:
            self.fault_latched = True
            self.fault_reason = "Robot state stale (>200ms)"
            self.hold()
        return fresh

    def clear_fault(self, confirm: bool = False) -> bool:
        if not confirm:
            return False
        self.fault_latched = False
        self.fault_reason = ""
        self.holding = False
        return True

    def _publish_loop(self) -> None:
        """Fixed 50 Hz background loop publishing JointState messages."""
        from sensor_msgs.msg import JointState

        rate_dt = 0.020  # 50 Hz fixed rate
        next_t = time.time()
        while not self._stop_event.is_set():
            if self.holding or self.estop_latched:
                with self._lock:
                    self._target_arm_q14 = self._latest_arm_q14.copy()
                    self._target_hand_h12 = self._latest_hand_h12.copy()

            with self._lock:
                arm_q = self._target_arm_q14.copy()
                hand_h = self._target_hand_h12.copy()

            # Publish 14-DoF Arm JointState
            if self._pub_arm is not None:
                msg_arm = JointState()
                msg_arm.header.stamp = self._node.get_clock().now().to_msg()
                msg_arm.name = list(ARM_JOINT_NAMES_AIMDK)
                msg_arm.position = [float(x) for x in arm_q]
                self._pub_arm.publish(msg_arm)

            # Publish 12-DoF Hand JointState
            if self._pub_hand is not None and not self.no_hand:
                msg_hand = JointState()
                msg_hand.header.stamp = self._node.get_clock().now().to_msg()
                msg_hand.header.frame_id = HAND_FRAME_ID
                msg_hand.name = list(HAND_JOINT_NAMES)
                msg_hand.position = [
                    float(x) for x in sim_hand_to_robot_raw(
                        hand_h, raw_max=self.hand_raw_max, enable_thumb_swing=self.enable_thumb_swing
                    )
                ]
                msg_hand.effort = [float(self.hand_effort)] * 12
                self._pub_hand.publish(msg_hand)

            next_t += rate_dt
            sleep_t = next_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.time()
