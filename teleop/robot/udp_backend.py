"""UDP Teleoperation Backend streaming 14 DoF targets and receiving real robot state."""
import os
import sys
import time
import json
import socket
import threading
from pathlib import Path
import numpy as np
from teleop.robot.interface import RobotInterface

class UdpBackend(RobotInterface):
    """Bidirectional UDP teleop backend with laptop-side logging:
    - Streams arm targets to Orin arm_teleop_rx.py (port 5005)
    - Receives actual robot joint state from Orin (port 5006)
    - Logs (no secrets): t_laptop, seq, quest msg age, IK ms, target q[14], measured q[14]
    """
    def __init__(self, host: str = "192.168.2.50", tx_port: int = 5005, rx_port: int = 5006, token: str = "", env=None):
        self.host = host
        self.tx_port = tx_port
        self.rx_port = rx_port
        self.token = token
        self.env = env
        
        self.tx_sock = None
        self.rx_sock = None
        self._connected = False
        self._seq = 0
        
        self._real_q14 = None
        self._real_q14_lock = threading.Lock()
        self._last_state_t = 0.0
        self._last_arm_cmd = None
        self._init_synced = False
        self.rx_thread = None

        self._log_file = None
        self._log_lock = threading.Lock()
        self.app = None  # Hooked by app if present

    def connect(self) -> None:
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rx_sock.bind(("0.0.0.0", self.rx_port))
            self.rx_sock.settimeout(0.2)
        except Exception as e:
            print(f"[UdpBackend] Bind RX port {self.rx_port} warning: {e}")

        # Setup logging to logs/real_<timestamp>.jsonl
        try:
            logs_dir = Path(__file__).resolve().parents[2] / "logs"
            logs_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            log_path = logs_dir / f"real_{ts}.jsonl"
            self._log_file = open(log_path, "a", buffering=1, encoding="utf-8")
            print(f"[UdpBackend] Logging laptop telemetry to {log_path}")
        except Exception as e:
            print(f"[UdpBackend] Failed to open real log: {e}")

        self._connected = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        print(f"[UdpBackend] Connected. TX -> {self.host}:{self.tx_port} | RX <- 0.0.0.0:{self.rx_port}")

    def disconnect(self) -> None:
        self._connected = False
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
                q = pkt.get("q")
                if q and len(q) == 14:
                    with self._real_q14_lock:
                        self._real_q14 = np.array(q, dtype=np.float64)
                        self._last_state_t = arrival_t
                    if not self._init_synced and self.env is not None and hasattr(self.env, "set_arm_commands"):
                        try:
                            self.env.set_arm_commands(self._real_q14[:7], self._real_q14[7:])
                            self._init_synced = True
                        except Exception:
                            pass
            except socket.timeout:
                pass
            except Exception:
                pass

    def send_arm(self, q14: np.ndarray) -> None:
        if not self._connected or self.tx_sock is None:
            return
        self._last_arm_cmd = np.array(q14, dtype=np.float64)
        self._seq += 1
        now = time.time()
        
        # Strip/omit secret token in any local logs
        pkt = {
            "token": self.token,
            "seq": self._seq,
            "t": now,
            "q": [float(round(x, 5)) for x in self._last_arm_cmd]
        }
        try:
            payload = json.dumps(pkt).encode("utf-8")
            self.tx_sock.sendto(payload, (self.host, self.tx_port))
        except Exception as e:
            print(f"[UdpBackend] Send error: {e}")

        # LAPTOP-SIDE LOGGING: t_laptop, seq, quest msg age, IK ms, target q[14], measured q[14]
        with self._log_lock:
            if self._log_file:
                with self._real_q14_lock:
                    meas_q = list(self._real_q14) if self._real_q14 is not None else [0.0]*14
                    meas_t = float(self._last_state_t)
                app = getattr(self, "app", None)
                quest_age = float(getattr(app, "last_msg_age_ms", 0.0)) if app else 0.0
                ik_ms = float(getattr(app, "last_ik_ms", 0.0)) if app else 0.0
                log_entry = {
                    "t_laptop": now,
                    "seq": self._seq,
                    "quest_msg_age_ms": quest_age,
                    "ik_ms": ik_ms,
                    "target_q": [float(round(x, 5)) for x in self._last_arm_cmd],
                    "measured_q": [float(round(x, 5)) for x in meas_q],
                    "measured_arrival_t": meas_t
                }
                self._log_file.write(json.dumps(log_entry) + "\n")

    def send_hand(self, h12: np.ndarray) -> None:
        pass

    def read_state(self) -> tuple[np.ndarray, np.ndarray, dict]:
        with self._real_q14_lock:
            if self._real_q14 is not None:
                return self._real_q14.copy(), np.zeros(12, dtype=np.float64), {}
        if self._last_arm_cmd is not None:
            return self._last_arm_cmd.copy(), np.zeros(12, dtype=np.float64), {}
        if self.env is not None and hasattr(self.env, "get_arm_qpos"):
            try:
                qp = self.env.get_arm_qpos()
                return np.concatenate([qp["left"], qp["right"]]), np.zeros(12, dtype=np.float64), {}
            except Exception:
                pass
        return np.zeros(14, dtype=np.float64), np.zeros(12, dtype=np.float64), {}

    def estop(self) -> None:
        print("[UdpBackend] ESTOP triggered!")

    def hold(self) -> None:
        if self._connected and self._last_arm_cmd is not None:
            self.send_arm(self._last_arm_cmd)

    def is_alive(self) -> bool:
        return self._connected
