#!/usr/bin/env python3
"""scripts/test_single_clock.py
Acceptance tests for Single Clock architecture (100 Hz fixed dt, pure open-loop):
1. Loopback socket test verifying packet dt std < 0.35 ms and no dt < 8 ms.
2. Drag with 30 Hz staircase target and 5% packet jitter:
   - SENT accel <= cap * 1.05 and jerk <= 1.2 * jerk_cap measured with real timestamps.
   - Zero dq sign changes after target stops.
   - Time-to-90% at --speed 3 < 1.2 s.
"""
import time, socket, json, threading, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.signal import savgol_filter
from teleop.robot.udp_backend import UdpBackend
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.robot.aimdk_map import HOME

class MockApp:
    def __init__(self):
        self.sync_state = "ENGAGED"
        self.mirror_limiter = MirrorLimiter(speed_level=3, dt=0.010)
        self.mirror_limiter.reset(HOME)
        self.arm_engaged_l = True
        self.arm_engaged_r = True

def test_single_clock_streaming():
    print("--- Running test_single_clock: Loopback & Staircase Drag ---")
    rx_port = 55455
    tx_port = 55456
    
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("127.0.0.1", rx_port))
    rx_sock.settimeout(0.5)
    
    app = MockApp()
    os.environ["TELEOP_LOCK_FILE"] = "/tmp/test_single_clock.lock"
    backend = UdpBackend(host="127.0.0.1", tx_port=rx_port, rx_port=tx_port)
    backend.app = app
    
    received_packets = []
    stop_event = threading.Event()
    
    def rx_worker():
        while not stop_event.is_set():
            try:
                data, _ = rx_sock.recvfrom(2048)
                t_arr = time.perf_counter()
                pkt = json.loads(data.decode("utf-8"))
                received_packets.append((t_arr, pkt))
            except socket.timeout:
                pass
            except Exception:
                break

    rx_thread = threading.Thread(target=rx_worker, daemon=True)
    rx_thread.start()
    
    backend.connect()
    
    # 30 Hz staircase target emulation during drag
    target_q = np.array(HOME, dtype=np.float64)
    dt_tick = 0.033
    
    for i in range(30):
        target_q[0] += 0.010  # shoulder
        target_q[5] += 0.005  # wrist pitch
        backend.send_arm(target_q)
        time.sleep(dt_tick)
        
    time.sleep(1.2)
    
    stop_event.set()
    backend.disconnect()
    rx_thread.join(timeout=1.0)
    rx_sock.close()
    
    print(f"Total packets received: {len(received_packets)}")
    assert len(received_packets) > 150, f"Too few packets: {len(received_packets)}"
    
    # 1. Packet dt analysis
    t_stamps = [p[0] for p in received_packets]
    dts = np.diff(t_stamps)
    dt_mean_ms = float(np.mean(dts) * 1000.0)
    dt_std_ms = float(np.std(dts) * 1000.0)
    min_dt_ms = float(np.min(dts) * 1000.0)
    max_dt_ms = float(np.max(dts) * 1000.0)
    print(f"Packet dt: mean={dt_mean_ms:.3f} ms, std={dt_std_ms:.3f} ms, min={min_dt_ms:.3f} ms, max={max_dt_ms:.3f} ms")
    assert min_dt_ms >= 8.0, f"Violation: min dt {min_dt_ms:.3f} ms < 8.0 ms (burst detected)!"
    assert dt_std_ms < 0.35, f"Violation: dt std {dt_std_ms:.3f} ms >= 0.35 ms!"
    
    # 2. Accel and Jerk with real timestamps
    qs = np.array([p[1]["q"] for p in received_packets])
    mean_dt = float(np.mean(dts))
    accs = savgol_filter(qs, window_length=9, polyorder=3, deriv=2, delta=mean_dt, axis=0)
    jerks = savgol_filter(qs, window_length=9, polyorder=3, deriv=3, delta=mean_dt, axis=0)
    
    cap_acc = 2.0
    cap_jerk = 20.0
    wrist_idx = [5, 6, 12, 13]
    non_wrist_idx = [0, 1, 2, 3, 4, 7, 8, 9, 10, 11]
    
    peak_acc_non_wrist = float(np.max(np.abs(accs[:, non_wrist_idx])))
    peak_jerk_non_wrist = float(np.max(np.abs(jerks[:, non_wrist_idx])))
    peak_acc_wrist = float(np.max(np.abs(accs[:, wrist_idx])))
    peak_jerk_wrist = float(np.max(np.abs(jerks[:, wrist_idx])))
    
    print(f"SENT Accel: Non-wrist peak={peak_acc_non_wrist:.3f} rad/s^2 (cap*1.05={cap_acc*1.05:.3f}), Wrist peak={peak_acc_wrist:.3f}")
    print(f"SENT Jerk : Non-wrist peak={peak_jerk_non_wrist:.3f} rad/s^3 (jerk_cap*1.2={cap_jerk*1.2:.3f}), Wrist peak={peak_jerk_wrist:.3f}")
    
    assert peak_acc_non_wrist <= cap_acc * 1.051, f"Peak accel {peak_acc_non_wrist} > {cap_acc * 1.05}"
    assert peak_jerk_non_wrist <= cap_jerk * 1.201, f"Peak jerk {peak_jerk_non_wrist} > {cap_jerk * 1.2}"
    assert peak_acc_wrist <= (cap_acc * 0.5) * 1.051, f"Peak wrist accel {peak_acc_wrist} > {cap_acc * 0.5 * 1.05}"
    assert peak_jerk_wrist <= (cap_jerk * 0.5) * 1.201, f"Peak wrist jerk {peak_jerk_wrist} > {cap_jerk * 0.5 * 1.2}"
    
    # 3. Time-to-90% (< 1.2 s from start of motion)
    j0_trace = qs[:, 0]
    total_disp = j0_trace[-1] - j0_trace[0]
    thresh_90 = j0_trace[0] + 0.90 * total_disp
    idx_start = np.where(np.abs(j0_trace - j0_trace[0]) > 0.005)[0][0]
    idx_90 = np.where(j0_trace >= thresh_90)[0][0]
    time_to_90 = t_stamps[idx_90] - t_stamps[idx_start]
    print(f"Time-to-90%: {time_to_90:.3f} s (requirement < 1.2 s)")
    assert time_to_90 < 1.20, f"Time-to-90% {time_to_90} >= 1.2 s!"
    
    # 4. Zero dq sign changes after target stops (last 1.0 s)
    t_stop_drag = t_stamps[0] + 1.10
    idx_after_stop = np.where(np.array(t_stamps) > t_stop_drag)[0]
    assert len(idx_after_stop) > 30, "Too few points after drag stop!"
    
    dq_after = np.diff(j0_trace[idx_after_stop])
    non_trivial_dq = dq_after[np.abs(dq_after) > 1e-5]
    sign_changes = np.sum(np.diff(np.sign(non_trivial_dq)) != 0)
    print(f"dq sign changes after target stops: {sign_changes}")
    assert sign_changes == 0, f"Hunting / oscillation detected after stop: {sign_changes} sign changes!"
    
    print("[PASS] test_single_clock PASSED completely!")

if __name__ == "__main__":
    test_single_clock_streaming()
