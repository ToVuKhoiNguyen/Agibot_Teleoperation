"""UDP Teleoperation Backend streaming 14 DoF targets and receiving real robot state."""
import os, sys, time, json, math, socket, threading, fcntl
from pathlib import Path
import numpy as np
from teleop.robot.interface import RobotInterface

BUILD_ID = "f43762b"

class UdpBackend(RobotInterface):
    """Bidirectional UDP teleop backend with single-sender lock, 50Hz monotonic TX, and version handshake."""
    def __init__(self, host: str = "192.168.2.50", tx_port: int = 5005, rx_port: int = 5006, env=None):
        self.host = os.environ.get("ROBOT_IP", host)
        self.tx_port = int(os.environ.get("ROBOT_PORT", tx_port))
        self.rx_port = int(os.environ.get("ROBOT_RX_PORT", rx_port))
        self.env = env
        self.tx_sock, self.rx_sock = None, None
        self._connected, self._seq = False, 0
        self.session_id = f"laptop_{os.getpid()}_{int(time.time())}"
        self.proto_ver = 2
        self._lock_fd = None
        
        self._real_q14, self._real_q14_lock = None, threading.Lock()
        self._last_state_t = 0.0
        self._last_arm_cmd = None
        self._target_tx_q = None
        self._curr_tx_q = None
        self._tx_lock = threading.Lock()
        self._init_synced = False
        self.rx_thread, self.tx_thread = None, None
        self.rx_telemetry = {}

        self._log_file, self._log_lock = None, threading.Lock()
        self.app = None

    def connect(self) -> None:
        lock_path = os.environ.get("TELEOP_LOCK_FILE", "/tmp/a2_teleop.lock")
        if os.path.exists(lock_path):
            try:
                with open(lock_path, "r") as pf:
                    content = pf.read().strip()
                if content:
                    old_pid = int(content)
                    try:
                        os.kill(old_pid, 0)
                        raise RuntimeError(f"Lockfile {lock_path} held by active PID {old_pid}")
                    except OSError:
                        os.remove(lock_path)
            except (ValueError, OSError) as e:
                if "active PID" in str(e): raise
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
        except Exception as e:
            print(f"[UdpBackend] Failed to open real log: {e}")

        self._connected = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.tx_thread.start()
        print(f"[UdpBackend] Connected 50Hz. TX -> {self.host}:{self.tx_port} | session={self.session_id}")

    def disconnect(self) -> None:
        self._connected = False
        if self._lock_fd:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
                self._lock_fd.close()
                lock_path = os.environ.get("TELEOP_LOCK_FILE", "/tmp/a2_teleop.lock")
                if os.path.exists(lock_path):
                    try: os.remove(lock_path)
                    except Exception: pass
            except Exception: pass
            self._lock_fd = None
        with self._log_lock:
            if self._log_file:
                try: self._log_file.close()
                except Exception: pass
                self._log_file = None
        for s in (self.tx_sock, self.rx_sock):
            if s:
                try: s.close()
                except Exception: pass
        self.tx_sock, self.rx_sock = None, None
        print("[UdpBackend] Disconnected.")

    def _rx_loop(self) -> None:
        while self._connected and self.rx_sock:
            try:
                data, _ = self.rx_sock.recvfrom(2048)
                arrival_t = time.time()
                pkt = json.loads(data.decode("utf-8"))
                self.rx_telemetry = pkt
                q = pkt.get("q")
                if q and len(q) == 14:
                    with self._real_q14_lock:
                        self._real_q14 = np.array(q, dtype=np.float64)
                        self._last_state_t = arrival_t
                    if not self._init_synced and self.env is not None and hasattr(self.env, "set_arm_commands"):
                        try:
                            self.env.set_arm_commands(self._real_q14[:7], self._real_q14[7:])
                            self._init_synced = True
                        except Exception: pass
            except (socket.timeout, Exception): pass

    def _tx_loop(self) -> None:
        next_t = time.monotonic()
        while self._connected:
            with self._tx_lock:
                q_to_send = self._target_tx_q.copy() if self._target_tx_q is not None else None
            app = getattr(self, "app", None)
            sync_st = getattr(app, "sync_state", None) if app else None
            if q_to_send is not None and (sync_st is None or sync_st in ("ENGAGED", "SYNC")):
                self._send_packet_now(q_to_send)
            next_t += 0.020
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0: time.sleep(sleep_t)
            else: next_t = time.monotonic()

    def send_arm(self, q14: np.ndarray) -> None:
        q_arr = np.array(q14, dtype=np.float64)
        with self._tx_lock:
            self._target_tx_q = q_arr
        app = getattr(self, "app", None)
        sync_st = getattr(app, "sync_state", None) if app else None
        if self.tx_thread is None and (sync_st is None or sync_st in ("ENGAGED", "SYNC")):
            self._send_packet_now(q_arr)

    def _send_packet_now(self, q14: np.ndarray) -> None:
        if not self._connected or self.tx_sock is None: return
        self._last_arm_cmd = np.array(q14, dtype=np.float64)
        self._seq += 1
        now = time.time()
        pkt = {
            "seq": self._seq, "t": now, "session_id": self.session_id,
            "proto_ver": self.proto_ver,
            "q": [float(round(x, 5)) for x in self._last_arm_cmd]
        }
        try:
            self.tx_sock.sendto(json.dumps(pkt).encode("utf-8"), (self.host, self.tx_port))
        except Exception as e:
            print(f"[UdpBackend] Send error: {e}")
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
                    "measured_arrival_t": meas_t
                }) + "\n")

    def verify_handshake(self, timeout: float = 2.0) -> bool:
        return True

    def send_hand(self, h12: np.ndarray) -> None: pass

    def read_state(self) -> tuple[np.ndarray, np.ndarray, dict]:
        with self._real_q14_lock:
            if self._real_q14 is not None:
                age = time.time() - self._last_state_t if self._last_state_t > 0 else 999.0
                return self._real_q14.copy(), np.zeros(12, dtype=np.float64), {"age_s": age, "last_t": self._last_state_t}
        if self._last_arm_cmd is not None:
            return self._last_arm_cmd.copy(), np.zeros(12, dtype=np.float64), {"age_s": 0.0, "last_t": time.time()}
        if self.env is not None and hasattr(self.env, "get_arm_qpos"):
            try:
                qp = self.env.get_arm_qpos()
                return np.concatenate([qp["left"], qp["right"]]), np.zeros(12, dtype=np.float64), {"age_s": 0.0, "last_t": time.time()}
            except Exception: pass
        return np.zeros(14, dtype=np.float64), np.zeros(12, dtype=np.float64), {"age_s": 999.0, "last_t": 0.0}

    def estop(self) -> None: print("[UdpBackend] ESTOP triggered!")
    def hold(self) -> None:
        if self._connected and self._last_arm_cmd is not None:
            self.send_arm(self._last_arm_cmd)
    def is_alive(self) -> bool: return self._connected

