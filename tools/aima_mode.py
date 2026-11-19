#!/usr/bin/env python3
"""
tools/aima_mode.py
Standalone tool to configure A2 robot operating modes and motion player state.
NOT imported by main.py. By default prints exact HTTP JSON without network activity.
"""

import os
import sys
import json
import argparse
import urllib.request
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.robot.aimdk_map import (
    A2_DEFAULT_IP,
    A2_MOTION_PORT,
    A2_MOTION_PLAYER_PORT,
    ACTION_ID_SERVO_402,
    ACTION_NAME_SERVO_402,
    ACTION_ID_STAND_DEFAULT,
    RPC_SET_ACTION_ENDPOINT,
    RPC_SET_ACTION_402_PAYLOAD,
    RPC_STOP_MOTION_PLAYER_ENDPOINT,
    RPC_STOP_MOTION_PLAYER_PAYLOAD,
    CMD_AIMA_STOP_MOTION_PLAYER,
)

CONFIRMATION_PHRASE = "GANTRY AND ESTOP READY"


def format_http_preview(endpoint: str, payload: dict, description: str) -> str:
    lines = [
        f"--- {description} ---",
        f"POST {endpoint}",
        "Content-Type: application/json",
        "Timeout: 1000",
        json.dumps(payload, indent=2),
    ]
    return "\n".join(lines)


def send_rpc(endpoint: str, payload: dict, timeout_s: float = 1.0) -> dict:
    req = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json", "timeout": str(int(timeout_s * 1000))},
    )
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main():
    parser = argparse.ArgumentParser(description="AGIBOT A2 Ultra Mode Management Tool")
    parser.add_argument("--mode", type=str, default="servo", choices=["servo", "restore"], help="Mode to configure ('servo' = Action 402, 'restore' = Action 400 default)")
    parser.add_argument("--execute", action="store_true", help="Actually send HTTP requests (requires confirmation phrase)")
    parser.add_argument("--ip", type=str, default=os.environ.get("AGIBOT_RPC_HOST", A2_DEFAULT_IP), help=f"Robot control IP (default: {A2_DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=int(os.environ.get("AGIBOT_RPC_PORT", str(A2_MOTION_PORT))), help=f"Robot motion RPC port (default: {A2_MOTION_PORT})")
    args = parser.parse_args()

    stop_mp_url = f"http://{args.ip}:{A2_MOTION_PLAYER_PORT}/rpc/aimdk.protocol.MotionPlayerService/SendMotionCommand"
    set_action_url = f"http://{args.ip}:{args.port}/rpc/aimdk.protocol.McActionService/SetAction"

    action_id = ACTION_ID_SERVO_402 if args.mode == "servo" else ACTION_ID_STAND_DEFAULT
    action_label = ACTION_NAME_SERVO_402 if args.mode == "servo" else "McAction_RL_JOINT_DEFAULT (Restore)"

    action_payload = {
        "header": {"timestamp": {"seconds": 0, "nanos": 0}},
        "command": {"action": action_id, "ext_action": action_label},
    }

    # Step 1: Preview or send stop motion_player
    p1 = format_http_preview(stop_mp_url, RPC_STOP_MOTION_PLAYER_PAYLOAD, f"Step 1: Stop Onboard motion_player (or run '{CMD_AIMA_STOP_MOTION_PLAYER}')")
    # Step 2: Preview or send SetAction
    p2 = format_http_preview(set_action_url, action_payload, f"Step 2: SetAction -> {action_label} ({action_id})")

    print("=" * 70)
    print("AGIBOT A2 ULTRA MODE MANAGEMENT PREVIEW")
    print("=" * 70)
    print(p1)
    print()
    print(p2)
    print("=" * 70)

    if not args.execute:
        print("[DRY-RUN PREVIEW] Zero network packets transmitted.")
        print(f"To execute on hardware, re-run with: --execute (requires '{CONFIRMATION_PHRASE}')")
        return 0

    # Safety locks for execution
    if os.environ.get("AGIBOT_ALLOW_REAL") != "1":
        print("[ERROR] Hardware execution blocked: AGIBOT_ALLOW_REAL=1 required in environment.")
        return 1

    confirm = input(f"Type '{CONFIRMATION_PHRASE}' to send HTTP requests to {args.ip}: ")
    if confirm.strip() != CONFIRMATION_PHRASE:
        print("[ABORT] Confirmation phrase mismatch. Operation cancelled.")
        return 1

    print(f"[INFO] Dispatching RPC requests to {args.ip}...")
    try:
        r1 = send_rpc(stop_mp_url, RPC_STOP_MOTION_PLAYER_PAYLOAD)
        print(f"Step 1 Response: {r1}")
        r2 = send_rpc(set_action_url, action_payload)
        print(f"Step 2 Response: {r2}")
        print("[SUCCESS] Mode configuration commands delivered successfully.")
    except Exception as e:
        print(f"[ERROR] RPC delivery failed: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

