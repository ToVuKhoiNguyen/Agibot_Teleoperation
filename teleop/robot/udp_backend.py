"""UDP Teleoperation Backend streaming 14 DoF targets and receiving real robot state."""
import os, time, json, socket, threading, fcntl
from pathlib import Path
import numpy as np
from teleop.robot.interface import RobotInterface
from teleop.robot.aimdk_map import SIM_HAND_COMFORTABLE_LIMITS, sim_hand_to_robot_raw, HAND_SAFE_RAW_MAX
from teleop.robot.trace_logger import LaptopTraceLogger

BUILD_ID = "f43762b"

class UdpBackend(RobotInterface):
    """Bidirectional UDP teleop backend with single-sender lock, 50Hz monotonic TX, and version handshake."""
    def __init__(self, host: str = "192.168.2.50", tx_port: int = 5005, rx_port: int = 5006, env=None):
        self.host, self.tx_port, self.rx_port, self.env = os.environ.get("ROBOT_IP", host), int(os.environ.get("ROBOT_PORT", tx_port)), int(os.environ.get("ROBOT_RX_PORT", rx_port)), env
        self.tx_sock, self.rx_sock, self._connected, self._seq = None, None, False, 0
        self.session_id = f"laptop_{os.getpid()}_{int(time.time())}"
        self.wire_hz, self.proto_ver, self._lock_fd, self._warned_proto = int(os.environ.get("WIRE_HZ", "100")), 3, None, False
        self._real_q14, self._real_q14_lock = None, threading.Lock()
        self._real_hand12, self._real_hand_lock = None, threading.Lock()
        self._last_state_t, self._last_arm_cmd, self._target_tx_q, self._target_tx_h, self._curr_tx_q = 0.0, None, None, None, None
        self._tx_lock, self._init_synced, self.rx_thread, self.tx_thread, self.rx_telemetry = threading.Lock(), False, None, None, {}
        self._log_file, self._log_lock, self._trace_logger, self.app = None, threading.Lock(), None, None

    def connect(self) -> None:
        lock_path = os.environ.get("TELEOP_LOCK_FILE", "/tmp/a2_teleop.lock")
        if os.path.exists(lock_path):
            try:
                with open(lock_path, "r") as pf:
                    content = pf.read().strip()
                if content:
                    os.kill(int(content), 0)
                    raise RuntimeError(f"Lockfile {lock_path} held by active PID {content}")
            except (ValueError, OSError) as e:
                if "active PID" in str(e): raise
                try: os.remove(lock_path)
                except OSError: pass
        try:
            self._lock_fd = open(lock_path, "w")
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_fd.write(str(os.getpid()))
            self._lock_fd.flush()
        except (IOError, BlockingIOError, OSError) as e:
            if self._lock_fd: self._lock_fd.close()
            self._lock_fd = None
            raise RuntimeError(f"Single-sender lock {lock_path} already held: {e}")
            
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx_sock.bind(("0.0.0.0", self.rx_port))
            self.rx_sock.settimeout(0.2)
        except Exception as e:
            print(f"[UdpBackend] Bind RX port {self.rx_port} warning: {e}")

        try:
            logs_dir = Path(__file__).resolve().parents[2] / "logs"
            logs_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._log_file = open(logs_dir / f"real_{ts}.jsonl", "a", buffering=1, encoding="utf-8")
            self._trace_logger = LaptopTraceLogger(logs_dir, self.wire_hz, getattr(getattr(self, "app", None), "speed_level", 3))
        except Exception as e:
            print(f"[UdpBackend] Failed to open real log: {e}")
        self._connected = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.tx_thread.start()
        print(f"[UdpBackend] Connected {self.wire_hz}Hz (WIRE_HZ={self.wire_hz}). TX -> {self.host}:{self.tx_port} | session={self.session_id}")

    def disconnect(self) -> None:
        self._connected = False
        if self._lock_fd:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
                self._lock_fd.close()
                lock_p = os.environ.get("TELEOP_LOCK_FILE", "/tmp/a2_teleop.lock")
                if os.path.exists(lock_p): os.remove(lock_p)
            except Exception: pass
            self._lock_fd = None
        with self._log_lock:
            if self._log_file:
                try: self._log_file.close()
                except Exception: pass
                self._log_file = None
        if self._trace_logger:
            try: self._trace_logger.close()
            except Exception: pass
            self._trace_logger = None
        for s in (self.tx_sock, self.rx_sock):
            if s:
                try: s.close()
                except Exception: pass
        self.tx_sock, self.rx_sock = None, None
        print("[UdpBackend] Disconnected.")

    def _rx_loop(self) -> None:
        while self._connected and self.rx_sock:
            try:
                data, _ = self.rx_sock.recvfrom(65535)   # with finger pressure arrays a packet is a few kB
                arrival_t = time.time()
                pkt = json.loads(data.decode("utf-8"))
                self.rx_telemetry = pkt
                app = getattr(self, "app", None)
                if app is not None and getattr(app, "rec_active", False):
                    from teleop.app.episode_recorder import log_robot
                    log_robot(app, pkt, arrival_t)
                else:
                    from teleop.app.episode_recorder import live_preview_robot
                    live_preview_robot(pkt, arrival_t)
                self._check_pacer_health(pkt.get("pacer"))
                orin_ver = pkt.get("proto_ver")
                orin_hz = pkt.get("wire_hz")
                if not self._warned_proto and (orin_ver is not None or orin_hz is not None):
                    if orin_ver != self.proto_ver or (orin_hz and orin_hz != self.wire_hz):
                        print(f"[WARN] Protocol mismatch! Laptop(v{self.proto_ver}, {self.wire_hz}Hz) vs Orin(v{orin_ver}, {orin_hz}Hz, sha={str(pkt.get('sha'))[:8]})")
                        self._warned_proto = True
                q = pkt.get("q")
                if q and len(q) == 14:
                    with self._real_q14_lock:
                        self._real_q14, self._last_state_t = np.array(q, dtype=np.float64), arrival_t
                    if not self._init_synced and self.env is not None and hasattr(self.env, "set_arm_commands"):
                        try:
                            self.env.set_arm_commands(self._real_q14[:7], self._real_q14[7:])
                            self._init_synced = True
                        except Exception: pass
                h = pkt.get("hand")
                if h and len(h) == 12:
                    with self._real_hand_lock:
                        self._real_hand12 = (np.clip(np.array(h, dtype=np.float64), 0.0, 2000.0) / 2000.0) * SIM_HAND_COMFORTABLE_LIMITS
            except (socket.timeout, Exception): pass

    def _check_pacer_health(self, ph) -> None:
        """Warn when the Orin command pacer is not real-time or keeps leaving gaps > 30 ms (both make the arm jerk:
        16/11 without 'sudo prlimit' the stream had gaps up to 139 ms). Warning only, no automatic action."""
        app = getattr(self, "app", None)
        if not isinstance(ph, dict):
            return
        msg = ""
        if not ph.get("rt", True):
            msg = "ORIN PACER NOT REAL-TIME: stop arm_teleop_rx.py, run 'sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40', start again"
        else:
            g = int(ph.get("gaps30", 0)); now = time.time()
            hist = getattr(self, "_pacer_gap_hist", [])
            hist = [(t, n) for t, n in hist if now - t < 30.0] + [(now, g)]
            self._pacer_gap_hist = hist
            if g - hist[0][1] >= 3:
                msg = f"ORIN PACER GAPS: {g - hist[0][1]} command gaps > 30 ms in 30 s (max {ph.get('gmax_ms')} ms) - Orin overloaded?"
        if msg != getattr(self, "_pacer_msg", ""):
            if msg: print(f"[WARN] {msg}")
            self._pacer_msg = msg
        if app is not None: app.pacer_warning = msg

    def _tx_loop(self) -> None:
        dt = 1.0 / self.wire_hz
        next_t = time.perf_counter()
        while self._connected:
            with self._tx_lock:
                raw_target = self._target_tx_q.copy() if self._target_tx_q is not None else None
                h_to_send = self._target_tx_h.copy() if self._target_tx_h is not None else None
            app = getattr(self, "app", None)
            sync_st = getattr(app, "sync_state", None) if app else None
            if raw_target is not None and sync_st == "SYNC":
                # Not synced (startup or feedback lost): never stream the stale plan, only hold the measured pose.
                with self._real_q14_lock:
                    hold_q = self._real_q14.copy() if self._real_q14 is not None else None
                if hold_q is not None:
                    self._send_packet_now(hold_q, h_to_send, raw_target=hold_q, plan_q=hold_q)
            elif raw_target is not None and (sync_st is None or sync_st == "ENGAGED"):
                scale = getattr(getattr(app, "robot_safety", None), "proxy_scale", 1.0)
                if hasattr(app, "mirror_limiter") and app.mirror_limiter is not None:
                    q_to_send = app.mirror_limiter.step(raw_target, dt=dt, speed_scale=scale)
                else:
                    q_to_send = raw_target
                self._send_packet_now(q_to_send, h_to_send, raw_target=raw_target, plan_q=q_to_send)
            next_t += dt
            now = time.perf_counter()
            sleep_t = next_t - now
            if sleep_t > 0.0001:
                time.sleep(sleep_t)
            elif sleep_t < -dt:
                next_t = time.perf_counter()

    def send_arm(self, q14: np.ndarray) -> None:
        q_arr = np.array(q14, dtype=np.float64)
        with self._tx_lock:
            self._target_tx_q = q_arr
        app = getattr(self, "app", None)
        sync_st = getattr(app, "sync_state", None) if app else None
        if self.tx_thread is None and (sync_st is None or sync_st in ("ENGAGED", "SYNC")):
            self._send_packet_now(q_arr, self._target_tx_h, raw_target=q_arr, plan_q=q_arr)

    def _send_packet_now(self, q14: np.ndarray, h12: np.ndarray = None, raw_target: np.ndarray = None, plan_q: np.ndarray = None) -> None:
        if not self._connected or self.tx_sock is None: return
        self._last_arm_cmd = np.array(q14, dtype=np.float64)
        self._seq += 1
        now = time.time()
        t_perf = time.perf_counter()
        app = getattr(self, "app", None)
        engaged = [bool(getattr(app, "arm_engaged_l", False)), bool(getattr(app, "arm_engaged_r", False))] if app else [True, True]
        pkt = {
            "seq": self._seq, "t": now, "session_id": self.session_id,
            "proto_ver": self.proto_ver,
            "q": [float(round(x, 5)) for x in self._last_arm_cmd],
            "engaged": engaged,
        }
        if app is not None:   # data collection: which episode the robot must record (or camera only, for the live page)
            from teleop.app.episode_recorder import rec_packet_field
            rec = rec_packet_field(app)
            if rec: pkt["rec"] = rec
            neck_t = getattr(app, "target_neck", None)
            if neck_t is not None:
                pkt["neck"] = [float(round(x, 4)) for x in neck_t]
        if h12 is not None and len(h12) == 12:
            h_arr = np.asarray(h12, dtype=np.float64)
            cap = int(getattr(self, "hand_raw_max", HAND_SAFE_RAW_MAX))   # set from --hand-raw-max
            if np.max(np.abs(h_arr)) <= 3.14:
                raw_h = sim_hand_to_robot_raw(h_arr, raw_max=cap)
            else:
                raw_h = np.clip(np.round(h_arr), 0, cap).astype(int)
            pkt["hand"] = [int(x) for x in raw_h]
        else:
            pkt["hand"] = [0] * 12
        self._last_hand_raw = pkt["hand"]   # raw finger command as sent (for the data-collection log)
        try:
            self.tx_sock.sendto(json.dumps(pkt).encode("utf-8"), (self.host, self.tx_port))
        except Exception as e:
            print(f"[UdpBackend] Send error: {e}")
        raw_t = raw_target if raw_target is not None else self._last_arm_cmd
        pl_q = plan_q if plan_q is not None else self._last_arm_cmd
        if self._trace_logger:
            self._trace_logger.log_tick(self._seq, t_perf, raw_t, raw_t, pl_q, self._last_arm_cmd)
        with self._log_lock:
            if self._log_file:
                with self._real_q14_lock:
                    meas_q = list(self._real_q14) if self._real_q14 is not None else [0.0]*14
                    meas_t = float(self._last_state_t)
                app = getattr(self, "app", None)
                self._log_file.write(json.dumps({
                    "t_laptop": now, "seq": self._seq,
                    "quest_msg_age_ms": float(getattr(app, "last_msg_age_ms", 0.0)) if app else 0.0,
                    "ik_ms": float(getattr(app, "last_ik_ms", 0.0)) if app else 0.0,
                    "target_q": [float(round(x, 5)) for x in self._last_arm_cmd],
                    "measured_q": [float(round(x, 5)) for x in meas_q],
                    "measured_arrival_t": meas_t,
                    "measured_stamp_mc": self.rx_telemetry.get("stamp")   # MC clock (newer receivers); use this for timing
                }) + "\n")

    def verify_handshake(self, timeout: float = 2.0) -> bool:
        return True

    def send_hand(self, h12: np.ndarray) -> None:
        h_arr = np.array(h12, dtype=np.float64)
        with self._tx_lock:
            self._target_tx_h = h_arr

    def read_state(self) -> tuple[np.ndarray, np.ndarray, dict]:
        with self._real_hand_lock:
            h_meas = self._real_hand12.copy() if self._real_hand12 is not None else np.zeros(12, dtype=np.float64)
        robot_fault = bool(self.rx_telemetry.get("robot_fault", False))
        fault_reason = self.rx_telemetry.get("fault_reason", "")
        rx_st = self.rx_telemetry.get("rx_state", "WAITING")
        meta = {"age_s": 0.0, "last_t": 0.0, "robot_fault": robot_fault, "fault_reason": fault_reason, "rx_state": rx_st}
        with self._real_q14_lock:
            if self._real_q14 is not None:
                meta["age_s"] = time.time() - self._last_state_t if self._last_state_t > 0 else 999.0
                meta["last_t"] = self._last_state_t
                return self._real_q14.copy(), h_meas, meta
        if self._last_arm_cmd is not None:
            meta["last_t"] = time.time()
            return self._last_arm_cmd.copy(), h_meas, meta
        if self.env is not None and hasattr(self.env, "get_arm_qpos"):
            try:
                qp = self.env.get_arm_qpos()
                meta["last_t"] = time.time()
                return np.concatenate([qp["left"], qp["right"]]), h_meas, meta
            except Exception: pass
        meta["age_s"] = 999.0
        return np.zeros(14, dtype=np.float64), h_meas, meta

    def estop(self) -> None: print("[UdpBackend] ESTOP triggered!")
    def hold(self) -> None:
        if self._connected and self._last_arm_cmd is not None: self.send_arm(self._last_arm_cmd)
    def is_alive(self) -> bool: return self._connected

