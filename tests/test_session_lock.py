#!/usr/bin/env python3
"""tests/test_session_lock.py - Loopback tests for Orin session lock and rx_state."""
import os, sys, time, socket, json, threading
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np

def test_session_lock_behavior():
    print("--- Running test_session_lock_behavior ---")
    
    # Mock Orin node logic
    lock_state = {
        "session_lock": None,
        "last_pkt_time": 0.0,
        "foreign_count": 0,
        "holding": False,
        "has_published_engaged": False,
        "target_engaged": [False, False],
        "curr_cmd_q": [0.0]*14,
        "meas_q": [0.5]*14,
        "pub_count": 0
    }
    
    def on_packet(addr, p):
        now = time.monotonic()
        sender_id = (addr[0], p.get("session_id", "default"))
        if lock_state["session_lock"] is not None and (now - lock_state["last_pkt_time"] > 2.0):
            lock_state["session_lock"] = None
            lock_state["holding"] = False
            lock_state["has_published_engaged"] = False
            lock_state["target_engaged"] = [False, False]

        if lock_state["session_lock"] is None:
            lock_state["session_lock"] = sender_id
        elif lock_state["session_lock"] != sender_id:
            lock_state["foreign_count"] += 1
            lock_state["holding"] = True
            return "HOLDING"

        lock_state["last_pkt_time"] = now
        eng = p.get("engaged", [False, False])
        lock_state["target_engaged"] = eng

        # Control tick check
        if not lock_state["has_published_engaged"] and any(eng):
            lock_state["curr_cmd_q"] = list(lock_state["meas_q"])
            lock_state["has_published_engaged"] = True
            lock_state["pub_count"] += 1
            return "RX_OK"
        elif lock_state["has_published_engaged"] and any(eng):
            lock_state["pub_count"] += 1
            return "RX_OK"
        return "NOT_ENGAGED"

    # Test 1: First session connects
    res1 = on_packet(("192.168.2.75", 5005), {"session_id": "laptop_sess1", "q": [0.0]*14, "engaged": [True, True]})
    assert res1 == "RX_OK"
    assert lock_state["curr_cmd_q"] == lock_state["meas_q"]
    assert lock_state["pub_count"] == 1
    print("[PASS] Session 1 connected & first command synced to measured pose.")

    # Test 2: Second IP/session attempts to send while lock is held => rejected and counted
    res2 = on_packet(("192.168.2.99", 5005), {"session_id": "rogue_sess", "q": [0.0]*14, "engaged": [True, True]})
    assert res2 == "HOLDING"
    assert lock_state["foreign_count"] == 1
    assert lock_state["holding"] is True
    print("[PASS] Second IP during lock correctly rejected & counted.")

    # Test 3: Laptop restarts (silence > 2.0 s), new session connects with Engage
    time.sleep(2.05)
    res3 = on_packet(("192.168.2.75", 5005), {"session_id": "laptop_sess2", "q": [0.0]*14, "engaged": [True, True]})
    assert res3 == "RX_OK"
    assert lock_state["session_lock"] == ("192.168.2.75", "laptop_sess2")
    assert lock_state["holding"] is False
    assert lock_state["curr_cmd_q"] == lock_state["meas_q"]
    print("[PASS] After 2s silence + new Engage, session lock released and arm publishes with command == measured.")

    # Test 4: rx_state reaches UdpBackend read_state
    from teleop.robot.udp_backend import UdpBackend
    backend = UdpBackend(host="127.0.0.1", tx_port=55123, rx_port=55124)
    backend.rx_telemetry = {"q": [0.1]*14, "rx_state": "REJECT:stale_target"}
    backend._real_q14 = np.array([0.1]*14)
    backend._last_state_t = time.time()
    _, _, meta = backend.read_state()
    assert meta["rx_state"] == "REJECT:stale_target"
    print(f"[PASS] rx_state '{meta['rx_state']}' successfully parsed in laptop backend meta.")

if __name__ == "__main__":
    test_session_lock_behavior()
