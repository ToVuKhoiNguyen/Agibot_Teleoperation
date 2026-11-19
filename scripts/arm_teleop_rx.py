#!/usr/bin/env python3
"""scripts/arm_teleop_rx.py - Audited 50/100 Hz Arm & Hand Teleoperation Receiver for Orin."""
import os, sys, gc, time, json, math, socket, hashlib, argparse, threading, traceback
from pathlib import Path
import numpy as np
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
except ImportError:
    rclpy, Node, JointState = None, object, None

try:
    from scripts.orin_teleop_core import *
    from scripts.orin_trace_logger import OrinTraceLogger
except ImportError:
    from orin_teleop_core import *
    from orin_trace_logger import OrinTraceLogger

class ArmTeleopRx(Node):
    def __init__(self, mode_live=True, allow_ip="192.168.2.75", port=5005, tx_state_port=5006,
                 speed_level=3, deadband_rad=0.003, enable_thumb_swing=True, hand_raw_max=2000, thumb_rest=500, thumb_pose=(1700, 0),
                 neck_pitch=0.0, neck_yaw=0.0):
        super().__init__("arm_teleop_rx")
        self.is_live, self.allow_ip, self.tx_state_port = mode_live, allow_ip, tx_state_port
        self.deadband_rad, self.enable_thumb_swing = deadband_rad, enable_thumb_swing
        # Finger travel cap (vendor range 0..2000 = fully closed) and the default thumb pose: both thumb joints stay
        # folded at thumb_rest whenever a hand is not being driven (not engaged, stale link, hold), instead of 0.
        self.hand_raw_max = float(np.clip(hand_raw_max, 0, 2000)); self.thumb_rest = float(np.clip(thumb_rest, 0, self.hand_raw_max))
        # 17/11: thumb FIXED in a grasp pose (swing, flex) also while not driven, same as main.py --thumb-pose; None = old rest
        self.thumb_pose = None if thumb_pose is None else [float(np.clip(x, 0, self.hand_raw_max)) for x in thumb_pose]
        # Finger torque limit (raw 0..5700, vendor default 1140 = 20 %). The two thumb joints get THUMB_EFFORT (default 2280
        # = 40 % since 17/11 evening, operator: less thumb wobble together with the low rest pose 400/900): the resting
        # thumb wobbled when the arm swung; a higher limit lets its servo resist that. It also squeezes harder when
        # grasping. THUMB_EFFORT=1140 = as before.
        te = float(np.clip(float(os.environ.get("THUMB_EFFORT", "2280")), 0.0, 5700.0))
        self._hand_effort = [te if i in (0, 1, 6, 7) else 1140.0 for i in range(12)]
        if te != 1140.0: print(f"[arm_teleop_rx] thumb torque limit {te:.0f} raw (other fingers 1140)", flush=True)
        self.speed_level = speed_level if speed_level in SPEED_LEVELS else 3
        self.max_vel, self.max_acc = SPEED_LEVELS[self.speed_level]
        self.wire_hz = int(os.environ.get("WIRE_HZ", "100"))
        # 30 ms (fast default since 17/11); PLAYOUT_MS=50 absorbs late packet bursts better for +20 ms lag
        self.playout_delay = float(os.environ.get("PLAYOUT_MS", "30")) / 1000.0
        self.max_step = self.max_vel * (1.0 / self.wire_hz)
        self.curr_cmd_q, self.curr_cmd_v = None, np.zeros(14)
        self.lim_lo, self.lim_hi = np.array(LIMS_MIN, dtype=np.float64), np.array(LIMS_MAX, dtype=np.float64)
        self.target_q, self.target_h, self.target_engaged = None, [0.0]*12, [False, False]
        self.meas_h, self.hand_enabled, self.session_lock, self.foreign_sender_count = None, True, None, 0
        self.last_pkt_time, self.rx_count, self.last_tick_t = 0.0, 0, time.monotonic()
        self.fault_latched, self.holding, self.fault_reason = False, False, ""
        self.has_published_engaged, self.our_arm_pub_count, self.total_arm_cmd_count = False, 0, 0
        self.dual_check_timer_t, self._last_drained_seqs = time.monotonic(), []
        self.pkt_buffer = PlayoutBuffer(self.wire_hz, self.playout_delay)
        self.max_step_abs = min(0.018, max(self.max_step, 0.016))  # MC rejects |step| > 0.02 rad per message (level 6 would sit exactly on it)
        self.preflight_done = not mode_live  # LIVE: ignore laptop packets until main() finishes the publisher pre-flight
        with open(__file__, "rb") as f: self.file_sha = hashlib.sha256(f.read()).hexdigest()
        self.meas_q, self.meas_stamp, self.cmd_pub_count_sampled = None, 0.0, 1
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos) if self.is_live else None
        self.hand_cmd_pub = self.create_publisher(JointState, "/motion/control/hand_joint_command", qos) if self.is_live else None
        self.neck_cmd_pub = self.create_publisher(JointState, "/motion/control/neck_joint_command", qos) if self.is_live else None
        self.target_neck = [float(np.radians(neck_yaw)), float(np.radians(neck_pitch))]
        self._neck_tick_div = 0
        self.meas_neck, self.meas_h_effort_raw, self.hand_type = None, None, ""
        for top, cb in [("/motion/control/arm_joint_state", self._on_state), ("/motion/control/hand_joint_state", self._on_hand_state),
                        ("/motion/control/arm_joint_command", self._on_arm_cmd_seen), ("/motion/control/neck_joint_state", self._on_neck_state)]:
            self.create_subscription(JointState, top, cb, qos)
        self.pub_sample_timer = self.create_timer(1.0, self._sample_pub_count)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind_ip = os.environ.get("ROBOT_BIND_IP", "127.0.0.1" if allow_ip in ("127.0.0.1", "localhost") else "0.0.0.0")
        self.sock.bind((self.bind_ip, port))
        self.sock.setblocking(False)
        self._ps = [0.0] * 6
        self.recorder, self.target_rec = None, None   # episode recorder (data collection), started in main()
        self.cam = None                               # camera streamer to the laptop, only while an episode records
        # ARM_CMD_FORMAT=vendor: publish like the vendor's arm.py (velocity = 0, effort = 0). Default = true velocity
        # (13/11 checkpoint).
        self.vendor_fmt = os.environ.get("ARM_CMD_FORMAT", "").strip().lower() == "vendor"
        self.pacer, self._rx = None, self.sock   # _rx: where laptop packets are read (the pacer forwards them when active)
        self.running = True
        log_d = Path("./logs") if Path("./logs").exists() else Path(".")
        self.trace_logger = OrinTraceLogger(log_d, self.file_sha, self.wire_hz, int(self.playout_delay * 1000))
        # timing probe per tick [wake lateness, previous tick wall ms, drain ms, publish ms, previous tick CPU ms]
        self._tm = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._cnt_lock = threading.Lock()  # publisher counters: written by the ROS callback and the tick thread
        self._tick_thread = None  # main() -> start_tick_thread(): the control tick does NOT run on the ROS executor
        print(f"[arm_teleop_rx] SHA256: {self.file_sha} | {self.wire_hz}Hz | playout={int(self.playout_delay*1000)}ms | lvl={self.speed_level}")

    def _sample_pub_count(self):
        try: self.cmd_pub_count_sampled = len(self.get_publishers_info_by_topic("/motion/control/arm_joint_command"))
        except Exception: pass

    def _on_arm_cmd_seen(self, msg: JointState):
        with self._cnt_lock: self.total_arm_cmd_count += 1
        if hasattr(self, "trace_logger") and self.trace_logger:
            if not self.trace_logger.rec_writer: return
            q = extract_q14_from_msg(msg)
            if len(q) == 14: self.trace_logger.log_recorded_cmd(time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, q, self.cmd_pub_count_sampled)
    def _on_neck_state(self, msg: JointState):
        if len(msg.position) >= 1: self.meas_neck = [float(x) for x in msg.position[:4]]   # yaw, pitch (rad)

    def _on_hand_state(self, msg: JointState):
        fid = getattr(getattr(msg, "header", None), "frame_id", "")
        # Whole effort array exactly as published: finger torque and, per the vendor guide, the pressure-sensor
        # arrays of both hands (16 x 5 fingers + 25 palm + 25 back of hand per hand). Not interpreted here.
        self.hand_type, self.meas_h_effort_raw = fid, (list(msg.effort) if len(msg.effort) else None)
        self.hand_enabled = (not fid or fid == "AgiHand")
        if not self.hand_enabled: return
        n2p = dict(zip(msg.name, msg.position))
        if all(k in n2p for k in HAND_NAMES): self.meas_h = [float(n2p[k]) for k in HAND_NAMES]
        elif len(msg.position) >= 12: self.meas_h = [float(x) for x in msg.position[:12]]
        self.meas_h_tau = extract_field_by_name(msg, "effort", HAND_NAMES)

    def _on_state(self, msg: JointState):
        q = extract_q14_from_msg(msg)
        if len(q) == 14:
            self.meas_q = q
            self.meas_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not getattr(self, "_first_state_printed", False):
                print(f"[FEEDBACK] First joint state received! Streaming feedback to {self.allow_ip}:{self.tx_state_port}")
                self._first_state_printed = True
            if hasattr(self, "trace_logger") and self.trace_logger and self.trace_logger.rec_writer:
                self.trace_logger.log_recorded_state(time.monotonic(), self.meas_stamp, self.meas_q)
            if self.allow_ip not in ("0.0.0.0",):
                try:
                    rx_st = self._compute_rx_state()
                    pkt = {"t": time.time(), "q": [float(round(x, 5)) for x in self.meas_q],
                           "speed_level": self.speed_level, "robot_fault": bool(self.fault_latched),
                           "fault_reason": str(self.fault_reason), "rx_state": rx_st, "proto_ver": 3,
                           "wire_hz": self.wire_hz, "playout_ms": int(self.playout_delay * 1000),
                           "sha": self.file_sha[:12]}
                    pkt["stamp"] = self.meas_stamp          # MC's own state time: exact timing in the laptop logs
                    if self.pacer is not None:
                        try: pkt["pacer"] = self.pacer.health()
                        except Exception: pass
                    if self.meas_h is not None: pkt["hand"] = [float(round(x, 1)) for x in self.meas_h]
                    if self.meas_neck is not None: pkt["neck"] = [float(round(x, 4)) for x in self.meas_neck]
                    if self.recorder is not None: pkt["rec"] = self.recorder.status()
                    if self.target_rec and (time.monotonic() - self.last_pkt_time) < 3.0:
                        # Data collection: while the laptop records an episode, every feedback packet also carries the
                        # MC's own timestamp, joint velocity and torque, finger effort and the command just published.
                        pkt["stamp"], pkt["hand_type"] = self.meas_stamp, self.hand_type
                        for key, vals, nd in (("dq", extract_field_by_name(msg, "velocity", ARM_NAMES_LONG), 4),
                                              ("tau", extract_field_by_name(msg, "effort", ARM_NAMES_LONG), 3),
                                              ("hand_tau", getattr(self, "meas_h_tau", None), 1),
                                              ("hand_effort_raw", self.meas_h_effort_raw, 1), ("neck", self.meas_neck, 5),
                                              ("cmd", self.curr_cmd_q, 5)):
                            if vals is not None: pkt[key] = [float(round(x, nd)) for x in vals]
                    self.sock.sendto(json.dumps(pkt).encode(), (self.allow_ip, self.tx_state_port))
                except Exception: pass

    def _compute_rx_state(self) -> str:
        if self.fault_latched: return f"FAULT:{self.fault_reason}"
        if self.holding: return "HOLDING"
        if not any(self.target_engaged): return "NOT_ENGAGED"
        if time.monotonic() - self.last_pkt_time > 0.400: return "REJECT:stale_target"
        return "RX_OK"

    def _drain_socket(self):
        newest_pkt = None
        self._last_drained_seqs = []
        while True:
            try: data, addr = self._rx.recvfrom(2048)
            except (BlockingIOError, socket.error): break
            if self.allow_ip != "0.0.0.0" and addr[0] != self.allow_ip:
                self.foreign_sender_count += 1; continue
            try: p = json.loads(data.decode("utf-8"))
            except Exception: continue
            q = p.get("q", [])
            if len(q) != 14 or any(math.isnan(x) or math.isinf(x) for x in q): continue
            sender_id = (addr[0], p.get("session_id", "default"))
            if self.session_lock is not None and (time.monotonic() - self.last_pkt_time > 2.0):
                self.session_lock = None; self.holding = False; self.pkt_buffer.reset()  # no stale playout across sessions
                if self.pacer: self.pacer.reset_buffer()
                self.has_published_engaged = False; self.target_engaged = [False, False]
            if self.session_lock is None: self.session_lock = sender_id
            elif self.session_lock != sender_id:
                self.foreign_sender_count += 1; self.holding = True; continue
            if "seq" in p: self._last_drained_seqs.append(p["seq"])
            newest_pkt = p
            # every packet goes into the jitter buffer (bursts of 2 per tick are normal on this link)
            q_clip = np.clip(np.asarray(q, dtype=np.float64), VENDOR_MIN, VENDOR_MAX)   # session bounds applied at playout
            self.pkt_buffer.push(time.monotonic(), q_clip, p.get("seq", self.rx_count + len(self._last_drained_seqs)))
        if newest_pkt is not None:
            self.target_q = [float(x) for x in np.clip(np.asarray(newest_pkt["q"], dtype=np.float64), VENDOR_MIN, VENDOR_MAX)]
            h = newest_pkt.get("hand")
            if h and len(h) == 12: self.target_h = [float(np.clip(x, 0.0, self.hand_raw_max)) for x in h]
            self.target_rec = newest_pkt.get("rec")
            eng = newest_pkt.get("engaged")
            if eng and len(eng) >= 2: self.target_engaged = [bool(eng[0]), bool(eng[1])]
            neck = newest_pkt.get("neck")
            if neck and len(neck) >= 2: self.target_neck = [float(neck[0]), float(neck[1])]
            self.last_pkt_time, self.rx_count = time.monotonic(), self.rx_count + 1

    def _get_playout_target(self, now):
        tgt, extrapolating = self.pkt_buffer.target(now)
        if tgt is None:
            tgt = np.array(self.target_q) if self.target_q else np.array(self.curr_cmd_q)
        return np.clip(tgt, self.lim_lo, self.lim_hi), extrapolating

    def _final_line_guard_and_publish(self, target_pos, dt):
        if self.fault_latched or self.holding: return
        err = np.asarray(target_pos, dtype=np.float64) - np.asarray(self.curr_cmd_q, dtype=np.float64)
        D = float(np.max(np.abs(err)))
        # Speed limit in rad/s over the TRUE tick period (a fixed per-tick step made the arm fall behind on a late
        # tick and jump ahead on an early one); never above max_step_abs per message.
        cap = min(self.max_step_abs, self.max_vel * dt)
        step = err * (cap / D) if D > cap else err
        next_cmd_q = np.asarray(self.curr_cmd_q) + step
        if np.any(np.isnan(next_cmd_q)) or np.any(np.isinf(next_cmd_q)):
            self.fault_latched, self.fault_reason = True, "NaN/inf in command"; return
        self.curr_cmd_v = step / dt
        self.curr_cmd_q = [float(x) for x in np.clip(next_cmd_q, self.lim_lo, self.lim_hi)]
        if self.is_live and self.cmd_pub:
            msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position, msg.velocity = ARM_NAMES_LONG, list(self.curr_cmd_q), list(self.curr_cmd_v)
            if self.vendor_fmt: msg.velocity, msg.effort = [0.0] * 14, [0.0] * 14
            self.cmd_pub.publish(msg)
            with self._cnt_lock: self.our_arm_pub_count += 1

    def _control_tick(self):
        if not self.preflight_done:
            # Pre-flight must see a quiet command topic: never engage/publish before it finishes, even if the
            # laptop is already streaming engaged packets (else the rate check counts our own hold commands).
            while True:
                try: self._rx.recvfrom(2048)
                except (BlockingIOError, socket.error): break
            if self.meas_q: self.curr_cmd_q = list(self.meas_q)
            self.last_tick_t = time.monotonic()
            return
        if self.pacer: self.pacer.heartbeat()   # FIRST: recorder / camera calls below can block (starting rosbag)
        rec_now = self.target_rec if (time.monotonic() - self.last_pkt_time) < 3.0 else None
        if self.cam is not None: self.cam.request(rec_now.get("cam") if isinstance(rec_now, dict) else None)
        if self.recorder is not None:   # record only while the laptop keeps asking for it (3 s without packets = stop)
            self.recorder.request(self.target_rec if (time.monotonic() - self.last_pkt_time) < 3.0 else None)
        if self.pacer:
            if not self.fault_latched:
                if not self.pacer.alive():
                    rc = self.pacer.proc.poll()
                    self.fault_latched, self.fault_reason = True, ("pacer process stopped" if rc is not None else "pacer process frozen > 0.5 s") + (f" (exit code {rc}, see [pacer] lines in this terminal)" if rc is not None else "")
                    print(f"[arm_teleop_rx] FAULT: {self.fault_reason}", flush=True)
                elif self.pacer.S[C_FAULT] > 0.5: self.fault_latched, self.fault_reason = True, "NaN/inf in command"
        t_a = time.monotonic(); self._drain_socket(); self._tm[2] = (time.monotonic() - t_a) * 1000.0
        # Clock read AFTER the drain: on the loaded Orin the drain can wait > 10 ms for the CPU, and the playout
        # position must belong to the moment it is published, not to the start of the tick.
        now = time.monotonic()
        dt = min(0.040, max(0.002, now - self.last_tick_t))  # true period between commands
        self.last_tick_t = now
        meas = list(self.meas_q) if self.meas_q else None

        if self.is_live and (now - self.dual_check_timer_t >= 0.30):
            with self._cnt_lock:
                # Order matters: read what was SEEN first, then what the pacer has PUBLISHED up to now. Every seen
                # message of ours was published before, so without a second publisher total - ours is never
                # positive (reading "published" one tick late gave false faults at 200 Hz when this tick stalled).
                total = self.total_arm_cmd_count
                if self.pacer: self.our_arm_pub_count += self.pacer.new_pub_count()
                ours = self.our_arm_pub_count
                self.dual_check_timer_t, self.our_arm_pub_count, self.total_arm_cmd_count = now, 0, 0
            # Second-publisher detector. "ours" (published, possibly by the pacer process) and "total" (seen on the
            # topic through the ROS executor) are sampled at slightly different moments, so a single window can be
            # off by the few messages in flight, in either direction. Those errors cancel between consecutive windows;
            # a real second publisher adds a steady surplus. Accumulate (never below 0) and fault on a clear surplus.
            self._dual_cusum = max(0.0, getattr(self, "_dual_cusum", 0.0) + (total - ours)) if ours > 0 else 0.0
            if self._dual_cusum > 15.0:
                self.fault_latched, self.fault_reason = True, "DUAL PUBLISHER: another node is publishing"
                print(f"\n[FAULT] DUAL PUBLISHER DETECTED! Total: {total}, Ours: {ours}")

        if not getattr(self, "_control_loop_alive_printed", False):
            print(f"[arm_teleop_rx] Control loop alive! Feedback will stream as soon as /motion/control/arm_joint_state publishes.")
            self._control_loop_alive_printed = True

        is_any_engaged = any(self.target_engaged)
        is_fresh = (now - self.last_pkt_time <= 0.400) and not self.holding
        if not (self.has_published_engaged or (is_any_engaged and is_fresh)):
            if meas: self.curr_cmd_q = list(meas)
            if self.pacer: self.pacer.set_mode(MODE_SILENT)
            return

        if not self.has_published_engaged:
            self.curr_cmd_q, self.curr_cmd_v = list(meas) if meas else list(DATA_COLLECTION), np.zeros(14)
            self.has_published_engaged = True
            self.lim_lo, self.lim_hi = start_limits(self.curr_cmd_q)   # no start jump for a joint outside our bounds
            self.pkt_buffer.keep_last()  # playout must not blend pre-engage targets into the first moves
            if self.pacer: self.pacer.sync(self.curr_cmd_q)
            print("[ENGAGE] First command synchronized to measured pose. Teleop live active.")

        extrapolating = False
        if self.pacer:
            # The pacer process plays out, guards and publishes on its own even 100 Hz clock; this (irregular) tick
            # only decides WHAT it may do: track the laptop, hold the last command, or stay silent.
            if self.fault_latched or self.holding: self.pacer.set_mode(MODE_SILENT)
            else: self.pacer.set_mode(MODE_TRACK if (is_fresh and self.target_q) else MODE_HOLD)
            S = self.pacer.S
            if S[C_PUB] > 0: self.curr_cmd_q, self.curr_cmd_v = [float(x) for x in S[C_Q:C_Q + 14]], np.array(S[C_V:C_V + 14])
            extrapolating = bool(S[C_EXTRA] > 0.5)
            with self._cnt_lock: self.our_arm_pub_count += self.pacer.new_pub_count()
            self._ps = self.pacer.stats()   # pacer [ticks, gaps > 12 ms, gaps > 18 ms, max gap ms, work ms, accel-guard cuts]
        else:
            if is_fresh and self.target_q:
                target, extrapolating = self._get_playout_target(now)
            else:
                target = np.array(self.curr_cmd_q)
            t_a = time.monotonic(); self._final_line_guard_and_publish(target, dt); self._tm[3] = (time.monotonic() - t_a) * 1000.0

        if self.trace_logger:
            pub_q = list(self.curr_cmd_q) if self.curr_cmd_q else [0.0]*14
            depth = int(self.pacer.S[C_DEPTH]) if self.pacer else len(self.pkt_buffer)
            self.trace_logger.log_tick(now, self._last_drained_seqs, pub_q, depth, extrapolating, self.holding, meas, self.meas_stamp,
                                       tm=[round(x, 2) for x in self._tm] + ([round(x, 2) for x in self._ps] if self.pacer else []))

        if self.is_live and self.hand_cmd_pub and self.hand_enabled:
            is_hand_fresh = is_fresh and not self.holding and not self.fault_latched
            rest_h = np.zeros(12); rest_h[[0, 1, 6, 7]] = self.thumb_rest       # open hand, thumb slightly folded
            if self.thumb_pose is not None: rest_h[[0, 6]], rest_h[[1, 7]] = self.thumb_pose[0], self.thumb_pose[1]
            cmd_h = np.array(self.target_h, dtype=np.float64) if is_hand_fresh else rest_h
            if is_hand_fresh:
                if not self.target_engaged[0]: cmd_h[:6] = rest_h[:6]
                if not self.target_engaged[1]: cmd_h[6:] = rest_h[6:]
            if not self.enable_thumb_swing: cmd_h[0], cmd_h[6] = 0.0, 0.0
            cmd_h = np.clip(cmd_h, 0.0, self.hand_raw_max)
            hmsg = JointState(); hmsg.header.stamp = self.get_clock().now().to_msg()
            hmsg.header.frame_id, hmsg.name = "AgiHand", HAND_NAMES
            hmsg.position, hmsg.effort = [float(x) for x in cmd_h], self._hand_effort
            self.hand_cmd_pub.publish(hmsg)

        if self.is_live and self.neck_cmd_pub and self.target_neck:
            self._neck_tick_div = (getattr(self, "_neck_tick_div", 0) + 1) % 2
            if self._neck_tick_div == 0:
                nmsg = JointState(); nmsg.header.stamp = self.get_clock().now().to_msg()
                nmsg.name = ["idx27_head_joint1", "idx28_head_joint2"]
                yaw = float(np.clip(self.target_neck[0], -0.785, 0.785))
                pitch = float(np.clip(self.target_neck[1], -0.401, 0.401))
                nmsg.position = [yaw, pitch]
                nmsg.velocity = [0.0, 0.0]
                nmsg.effort = [0.0, 0.0]
                self.neck_cmd_pub.publish(nmsg)

    def start_pacer(self):
        """Hand the arm command timing to the light pacer process (see orin_teleop_core). ORIN_PACER=0 keeps the
        former in-process publishing; any start-up failure also falls back to it."""
        if not self.is_live or os.environ.get("ORIN_PACER", "1") == "0": return
        try:
            self.pacer = PacerLink(self.sock, self.allow_ip, self.wire_hz, self.playout_delay, True, self.max_vel, self.max_step_abs, self.vendor_fmt,
                                   pub_hz=max(50, min(400, int(os.environ.get("PACER_HZ", "400")))),  # 400 Hz: least vibration on the real robot (user test 13/11)
                                   # Low-pass (vendor: y = (1-a) y + a x): 2 stages; PACER_SMOOTH_MS=60 = smoothest (default 16/11
                                   # setting, chosen by the operator). Replay of a real 16/11 session with the 50 ms buffer
                                   # (tools/filter_report.py, docs/FILTER_REPORT.md): command shake 0.022 -> 0.009 rad/s, peak
                                   # accel 40 -> 5.6 rad/s^2 (under the vendor 6.28), no overshoot, ~+120 ms lag. 0 = off.
                                   # 17/11: default OFF (fast 1:1 config chosen by the operator; 60 ms adds ~120 ms lag).
                                   # Scalar PACER_MAX_ACC stays OFF (with feed-forward it rang at stops, 16/11); the default
                                   # acceleration / speed limits are now PER JOINT inside the pacer (JOINT_VMAX / JOINT_AMAX,
                                   # orin_teleop_core.pacer_main, default 1.5,1,2,0.8,2,2,2 rad/s, 6 rad/s^2).
                                   smooth_ms=max(0.0, min(80.0, float(os.environ.get("PACER_SMOOTH_MS", "0")))),
                                   max_acc=max(0.0, min(6.28, float(os.environ.get("PACER_MAX_ACC", "0")))))
            self._rx = self.pacer
            print("[arm_teleop_rx] arm commands are paced by a separate light process (ORIN_PACER=0 to disable)")
        except Exception as e:
            self.pacer, self._rx = None, self.sock
            print(f"[arm_teleop_rx] pacer not started ({e}); publishing in-process")

    def start_tick_thread(self):
        """Run the control tick on its own thread with an absolute 1/wire_hz schedule. As a ROS timer it shared the
        single-threaded executor with three 100 Hz subscriptions, and 7.6 % of ticks came > 18 ms apart (max 61 ms):
        the MC then saw the same smooth trajectory as uneven position steps, i.e. a light jerk about every 0.1 s."""
        self._tick_thread = threading.Thread(target=self._tick_loop, name="teleop_tick", daemon=True)
        self._tick_thread.start()

    def _tick_loop(self):
        prio = "normal priority"
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20)); prio = "SCHED_FIFO 20"
        except Exception:
            try: os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), -10); prio = "nice -10"
            except Exception: pass
        dt = 1.0 / self.wire_hz
        print(f"[arm_teleop_rx] control tick thread: {self.wire_hz} Hz absolute schedule, {prio}")
        next_t = time.monotonic()
        while self.running:
            t_w, c_w = time.monotonic(), time.thread_time()
            self._tm[0] = (t_w - next_t) * 1000.0          # how late the OS woke this thread up
            try: self._control_tick()
            except Exception as e:
                self.fault_latched, self.fault_reason = True, f"control tick error: {e}"
                traceback.print_exc()
            self._tm[1], self._tm[4] = (time.monotonic() - t_w) * 1000.0, (time.thread_time() - c_w) * 1000.0
            next_t += dt
            slack = next_t - time.monotonic()
            if slack > 0.0: time.sleep(slack)
            elif slack < -dt:                              # overran by more than a period: do not burst to catch up
                next_t = time.monotonic(); time.sleep(0.001)  # and always yield (this thread may be real-time)

    def shutdown_hold(self):
        self.holding, self.running = True, False
        if self.recorder is not None: self.recorder.close()
        if self.cam is not None: self.cam.stop()
        if self.pacer: self.pacer.stop()
        if hasattr(self, "trace_logger") and self.trace_logger: self.trace_logger.close()

def main():
    p = argparse.ArgumentParser()
    for flg, dflt, typ in [("--speed", 3, int), ("--port", 5005, int), ("--tx-state-port", 5006, int), ("--hand-raw-max", 2000, int), ("--thumb-rest", 500, int)]: p.add_argument(flg, type=typ, default=dflt)
    p.add_argument("--allow-ip", type=str, default="192.168.2.167")
    p.add_argument("--dry", action="store_true"); p.add_argument("--no-thumb-swing", action="store_true")
    p.add_argument("--thumb-pose", default="1700,0", help="thumb grasp pose (swing,flex) at rest 'swing,flex' raw; 'off' = --thumb-rest as before")
    p.add_argument("--allow-action", type=str, default="", help="Additional action substring or ID to allow (e.g. 203, 1, or custom)")
    p.add_argument("--auto-set-action", type=int, default=0, metavar="ID",
                   help="If robot is NOT in an allowed action, automatically call SetAction(ID) then wait up to 8 s for "
                        "it to take effect before starting. Use only when the operator is ready and E-stop is manned. "
                        "Example: --auto-set-action 7 (PASSIVE_UPPER_BODY_JOINT_SERVO) or --auto-set-action 12 "
                        "(RL_LOCOMOTION_ARM_EXT_JOINT_SERVO). The receiver still verifies the action afterwards and "
                        "exits if the robot did not accept it.")
    p.add_argument("--neck-pitch", type=float, default=0.0, help="Initial neck pitch angle in degrees (default: 0.0, positive = backwards)")
    p.add_argument("--neck-yaw", type=float, default=0.0, help="Initial neck yaw angle in degrees (default: 0.0)")
    args, _ = p.parse_known_args(); live = not args.dry
    if live:
        print("="*60 + f"\n  WARNING: REAL ROBOT TELEOP (IP: {args.allow_ip}, Speed: {args.speed})\n" + "="*60)
        curr_act = rpc_get_action()
        allowed = ["RL_LOCOMOTION_ARM_EXT_JOINT_SERVO", "402", "STAND_ARM_EXT_JOINT_SERVO", "203", "PASSIVE_UPPER_BODY_JOINT_SERVO"]
        if args.allow_action:
            allowed.append(str(args.allow_action))
        if not any(k in str(curr_act) for k in allowed):
            if args.auto_set_action > 0:
                print(f"[arm_teleop_rx] Robot is in action '{curr_act}'. Calling SetAction({args.auto_set_action}) ...")
                ok = rpc_set_action(args.auto_set_action)
                if not ok:
                    print(f"FATAL: SetAction({args.auto_set_action}) RPC failed (connection error). "
                          f"Check MC is running and RPC port is accessible.")
                    sys.exit(1)
                # Wait for action to take effect (MC may take several seconds to transition)
                deadline = time.time() + 8.0
                while time.time() < deadline:
                    time.sleep(1.0)
                    curr_act = rpc_get_action()
                    print(f"[arm_teleop_rx] GetAction -> '{curr_act}'")
                    if any(k in str(curr_act) for k in allowed):
                        print(f"[arm_teleop_rx] Robot accepted SetAction({args.auto_set_action}). Continuing ...")
                        break
                else:
                    print(f"FATAL: Robot did not switch to an allowed action after 8 s "
                          f"(still '{curr_act}'). MC may have rejected the request "
                          f"(robot not in correct state). Run SetAction manually first.")
                    sys.exit(1)
            else:
                print(
                    f"FATAL: Robot action is '{curr_act}'.\n"
                    f"  Allowed actions: {', '.join(allowed)}\n"
                    f"\n"
                    f"  Fix: on the Orin, run SetAction BEFORE starting this receiver:\n"
                    f"    Luồng A (Action 12 / 402):\n"
                    f"      python SetAction.py   # enter 10 (RL_JOINT_DEFAULT), wait robot stands stable\n"
                    f"      python SetAction.py   # enter 12 (RL_LOCOMOTION_ARM_EXT_JOINT_SERVO / 402)\n"
                    f"    Luồng B (Action 07 / PASSIVE_UPPER_BODY_JOINT_SERVO):\n"
                    f"      python SetAction.py   # enter 2 (STAND), wait robot stands stable\n"
                    f"      python SetAction.py   # enter 7 (PASSIVE_UPPER_BODY_JOINT_SERVO)\n"
                    f"\n"
                    f"  Advanced: --auto-set-action 7  (auto SetAction; operator + E-stop required)\n"
                    f"            --allow-action McAction_DEFAULT  (bypass check, NOT recommended)\n"
                    f"  See: real_robot_command.md, Buoc 2, buoc 5."
                )
                sys.exit(1)
    # The Orin runs at load ~30 on 12 cores and the robot's own processes are niced -5..-20: at nice 0 this loop
    # waits up to tens of ms for a CPU. Raise the priority when the shell allows it (see real_robot_command.md:
    # `sudo prlimit --pid $$ --nice=40:40 --rtprio=30:30` before starting); threads created later inherit it.
    try: os.setpriority(os.PRIO_PROCESS, 0, -15); print("[arm_teleop_rx] process priority: nice -15")
    except Exception: print("[arm_teleop_rx] process priority: nice 0 (not permitted to raise; expect uneven 100 Hz timing)")
    rclpy.init()
    node = ArmTeleopRx(mode_live=live, allow_ip=args.allow_ip, port=args.port, tx_state_port=args.tx_state_port, speed_level=args.speed, enable_thumb_swing=not args.no_thumb_swing,
                       hand_raw_max=args.hand_raw_max, thumb_rest=args.thumb_rest,
                       thumb_pose=None if str(args.thumb_pose).strip().lower() in ("off", "none", "") else
                       tuple(int(float(x)) for x in str(args.thumb_pose).split(",")[:2]),
                       neck_pitch=args.neck_pitch, neck_yaw=args.neck_yaw)
    if live:
        arm_p, h_p = check_other_publishers(node, "/motion/control/arm_joint_command"), check_other_publishers(node, "/motion/control/hand_joint_command")
        print(f"[PUB_CHECK] publishers: arm={arm_p or 'clean'}, hand={h_p or 'clean'}")
        arm_r, h_r = check_topic_rate_preflight(node, "/motion/control/arm_joint_command", 1.5), check_topic_rate_preflight(node, "/motion/control/hand_joint_command", 1.5)
        print(f"[PREFLIGHT] arm: {arm_r:.1f} msg/s, hand: {h_r:.1f} msg/s")
        if arm_r > 0.0 or h_r > 0.0: print(f"FATAL: Active pub: arm={arm_p}, hand={h_p}"); sys.exit(1)
        node.preflight_done = True  # packets that arrived during pre-flight were discarded; engage starts fresh
    sys.setswitchinterval(0.001)          # tick thread waits at most ~1 ms for the GIL (default 5 ms)
    gc.collect(); gc.freeze(); gc.set_threshold(20000, 50, 100)   # fewer / shorter GC pauses in the 100 Hz loop
    node.start_pacer()
    if live and args.allow_ip not in ("0.0.0.0",): node.cam = CameraStreamer(args.allow_ip)
    if os.environ.get("ORIN_RECORDER_ON", "0") == "1":   # optional on-robot rosbag (camera); default: data goes to the laptop
        node.recorder = EpisodeRecorder(info={"receiver_sha": node.file_sha, "speed_level": node.speed_level, "wire_hz": node.wire_hz})
        print(f"[rec] episode recorder ready: {node.recorder.root}  topics: {len(node.recorder.topics)}")
    node.start_tick_thread()
    print(f"[arm_teleop_rx] Ready and listening on {args.allow_ip}:{args.port}...")
    try: rclpy.spin(node)
    except (KeyboardInterrupt, Exception): pass
    finally:
        node.shutdown_hold()
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass
if __name__ == "__main__": main()
