#!/usr/bin/env python3
"""scripts/test_hand_path.py
Unit and loopback tests for the end-to-end hand path:
- Mapping round-trip (real 0..2000 vs sim rad, clamp <= 800)
- Loopback packet packing in UdpBackend (12 values, raw <= 800)
- arm_teleop_rx logic: ENGAGED gating, FAULT/hold/stale no-close, thumb_0 neutral, AgiHand frame_id
- Sim twin finger joint qpos/ctrl updates
"""
import sys, time, socket, json
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from teleop.robot.aimdk_map import (
    sim_hand_to_robot_raw,
    robot_raw_to_sim_hand,
    SIM_HAND_COMFORTABLE_LIMITS,
    HAND_SAFE_RAW_MAX,
    HAND_JOINT_NAMES,
    HAND_FRAME_ID,
)
from teleop.robot.udp_backend import UdpBackend
from teleop.simulation.mujoco_env import AgibotSimEnv

def test_mapping_roundtrip():
    """Verify real raw 0..2000 vs sim radians and clamp <= 800."""
    # 0 = open
    open_sim = np.zeros(12)
    raw_open = sim_hand_to_robot_raw(open_sim, raw_max=HAND_SAFE_RAW_MAX)
    assert np.all(raw_open == 0), f"Open sim should map to raw 0, got {raw_open}"
    assert np.allclose(robot_raw_to_sim_hand(raw_open), 0.0)

    # Full flexion clamped to 800
    full_sim = SIM_HAND_COMFORTABLE_LIMITS.copy()
    raw_clamped = sim_hand_to_robot_raw(full_sim, raw_max=HAND_SAFE_RAW_MAX)
    assert np.all(raw_clamped <= 800), f"Raw must be <= 800, got {raw_clamped}"
    assert np.all(raw_clamped[1:6] == 800), f"Fingers must reach 800 clamp, got {raw_clamped}"

    # Thumb swing neutral when disabled
    raw_no_swing = sim_hand_to_robot_raw(full_sim, raw_max=HAND_SAFE_RAW_MAX, enable_thumb_swing=False)
    assert raw_no_swing[0] == 0 and raw_no_swing[6] == 0, "Thumb swing must be 0 when disabled"

    # Monotonicity and no axis inversion (both 0=open, positive=closed)
    h_half = SIM_HAND_COMFORTABLE_LIMITS * 0.2
    raw_half = sim_hand_to_robot_raw(h_half, raw_max=HAND_SAFE_RAW_MAX)
    assert np.all(raw_half >= 0) and np.all(raw_half < raw_clamped)
    print("  [PASS] test_mapping_roundtrip")

def test_udp_backend_loopback_packet():
    """Verify UdpBackend sends packet with 'hand' 12 values and raw <= 800."""
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("127.0.0.1", 5995))
    rx_sock.settimeout(1.0)

    backend = UdpBackend(host="127.0.0.1", tx_port=5995, rx_port=5996)
    backend.connect()

    # Pass extreme values (both rad and raw)
    extreme_sim = np.array([2.5] * 12)
    backend.send_hand(extreme_sim)
    backend.send_arm(np.zeros(14))

    data, _ = rx_sock.recvfrom(2048)
    pkt = json.loads(data.decode("utf-8"))

    assert "hand" in pkt, "Packet must have 'hand' key"
    assert len(pkt["hand"]) == 12, f"Expected 12 hand values, got {len(pkt['hand'])}"
    assert all(0 <= v <= 800 for v in pkt["hand"]), f"Hand values must be 0..800, got {pkt['hand']}"
    assert "engaged" in pkt, "Packet must have 'engaged' key"

    backend.disconnect()
    rx_sock.close()
    print("  [PASS] test_udp_backend_loopback_packet")

def test_arm_teleop_rx_hand_logic():
    """Verify arm_teleop_rx hand publishing rules: ENGAGED gating, FAULT, frame_id."""
    import rclpy
    from scripts.arm_teleop_rx import ArmTeleopRx
    from sensor_msgs.msg import JointState

    if not rclpy.ok():
        rclpy.init()

    node = ArmTeleopRx(mode_live=False, allow_ip="127.0.0.1", enable_thumb_swing=False)
    
    # 1. Test frame_id check
    msg_bad = JointState()
    msg_bad.header.frame_id = "WrongFrame"
    node._on_hand_state(msg_bad)
    assert not node.hand_enabled, "Should disable hand on wrong frame_id"

    msg_good = JointState()
    msg_good.header.frame_id = "AgiHand"
    msg_good.name = list(HAND_JOINT_NAMES)
    msg_good.position = [100.0] * 12
    node._on_hand_state(msg_good)
    assert node.hand_enabled, "Should enable hand on AgiHand frame_id"
    assert node.meas_h == [100.0] * 12

    # 2. Test ENGAGED gating & thumb_0 neutral
    node.target_h = [500.0] * 12
    node.target_engaged = [True, False]  # Left engaged, right not
    node.last_pkt_time = time.monotonic()
    node.is_live = True
    
    published = []
    class DummyPub:
        def publish(self, msg): published.append(msg)
    node.hand_cmd_pub = DummyPub()

    # Trigger control tick (homing disabled for test)
    node.homing_active = False
    node.anchored = True
    node._control_tick()

    assert len(published) == 1
    h_cmd = published[0]
    assert h_cmd.header.frame_id == "AgiHand"
    assert np.allclose(h_cmd.effort, [1140.0] * 12)
    # Left thumb_0 neutral because enable_thumb_swing=False
    assert h_cmd.position[0] == 0.0
    # Left fingers 1..5 active
    assert all(p == 500.0 for p in h_cmd.position[1:6])
    # Right hand inactive (not engaged) -> all 0.0
    assert all(p == 0.0 for p in h_cmd.position[6:])

    # 3. Test FAULT latched -> sends 0 (no close)
    published.clear()
    node.fault_latched = True
    node._control_tick()
    assert len(published) == 1
    assert all(p == 0.0 for p in published[0].position), "FAULT must command 0 (never close)"

    # 4. Test stale packet (>0.400s) -> sends 0
    published.clear()
    node.fault_latched = False
    node.last_pkt_time = time.monotonic() - 0.500
    node._control_tick()
    assert len(published) == 1
    assert all(p == 0.0 for p in published[0].position), "Stale must command 0 (never close)"

    node.destroy_node()
    rclpy.shutdown()
    print("  [PASS] test_arm_teleop_rx_hand_logic")

def test_sim_hand_writes_qpos_ctrl():
    """Verify env.set_hand_commands writes both ctrl and qpos for finger joints."""
    scene_xml = str(Path(__file__).resolve().parent.parent / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")
    env = AgibotSimEnv(scene_xml, render_gui=False)
    target_l = np.array([0.5, 0.2, 0.8, 0.8, 0.8, 0.8])
    target_r = np.array([0.0, 0.1, 0.4, 0.4, 0.4, 0.4])

    env.set_hand_commands(target_l, target_r)

    # Check qpos and ctrl
    for i, jname in enumerate(env.left_hand_active_joints):
        act_idx = env.actuator_map[jname]
        qpos_idx = env.qpos_idx_map[jname]
        assert np.isclose(env.data.ctrl[act_idx], target_l[i], atol=1e-4)
        assert np.isclose(env.data.qpos[qpos_idx], target_l[i], atol=1e-4)

    for i, jname in enumerate(env.right_hand_active_joints):
        act_idx = env.actuator_map[jname]
        qpos_idx = env.qpos_idx_map[jname]
        assert np.isclose(env.data.ctrl[act_idx], target_r[i], atol=1e-4)
        assert np.isclose(env.data.qpos[qpos_idx], target_r[i], atol=1e-4)
    print("  [PASS] test_sim_hand_writes_qpos_ctrl")

def test_sync_hand_mirroring():
    """Verify SYNC mirrors measured hand into sim without sending hand command."""
    from types import SimpleNamespace
    from teleop.app.sync_manager import process_arm_sync_and_dispatch

    h_meas = np.array([0.2, 0.1, 0.4, 0.4, 0.4, 0.4, 0.2, 0.1, 0.4, 0.4, 0.4, 0.4])
    hand_cmds_sent = []
    sim_hand_cmds = []

    app = SimpleNamespace(
        sync_state="SYNC",
        arm_engaged_l=False,
        arm_engaged_r=False,
        last_arm_cmd_left=np.zeros(7),
        last_arm_cmd_right=np.zeros(7),
        last_hand_cmd_left=np.zeros(6),
        last_hand_cmd_right=np.zeros(6),
        current_safe_hand_l=np.zeros(6),
        current_safe_hand_r=np.zeros(6),
        mirror_limiter=SimpleNamespace(reset=lambda q: None, step=lambda *a, **k: np.zeros(14), lim_min=np.zeros(14), lim_max=np.zeros(14), tau_val=0.138, v_plan=np.zeros(14), q_plan=np.zeros(14)),
        robot_safety=SimpleNamespace(
            read_state=lambda: (np.zeros(14), h_meas, {"age_s": 0.01}),
            send_arm=lambda q, dt: True,
            send_hand=lambda h: hand_cmds_sent.append(h.copy()),
            context={},
            deviation_thresh_rad=0.05,
        ),
        env=SimpleNamespace(
            set_arm_commands=lambda l, r: None,
            set_hand_commands=lambda l, r: sim_hand_cmds.append((l.copy(), r.copy())),
            get_ee_poses=lambda: {"left": {"pos": [0,0,0], "rot": [0,0,0,1]}, "right": {"pos": [0,0,0], "rot": [0,0,0,1]}},
        ),
        cur_target_pos_l=[0,0,0], cur_target_rot_l=[0,0,0,1],
        cur_target_pos_r=[0,0,0], cur_target_rot_r=[0,0,0,1],
        direct_hand_override_l=False, direct_hand_override_r=False,
        hand_sweep_active=False,
    )

    # 1. In SYNC: mirrors measured into sim, sends NO hand command to robot
    process_arm_sync_and_dispatch(app, is_real=True, dt=0.02)
    assert len(sim_hand_cmds) == 1, "Sim hand commands must be updated in SYNC"
    assert np.allclose(sim_hand_cmds[0][0], h_meas[:6])
    assert np.allclose(sim_hand_cmds[0][1], h_meas[6:])
    assert len(hand_cmds_sent) == 0, "NEVER send hand command on connect/sync"

    # 2. Transition to ENGAGED: first hand command sent == measured hand
    app.sync_state = "ENGAGED"
    app.arm_engaged_l = True
    app.arm_engaged_r = True
    process_arm_sync_and_dispatch(app, is_real=True, dt=0.02)
    assert len(hand_cmds_sent) == 1, "Hand command sent when ENGAGED"
    assert np.allclose(hand_cmds_sent[0], h_meas), "First hand command == measured"
    print("  [PASS] test_sync_hand_mirroring")

if __name__ == "__main__":
    print("[TEST] Running hand path verification...")
    test_mapping_roundtrip()
    test_udp_backend_loopback_packet()
    test_arm_teleop_rx_hand_logic()
    test_sim_hand_writes_qpos_ctrl()
    test_sync_hand_mirroring()
    print("[TEST] ALL HAND PATH TESTS PASSED.")
