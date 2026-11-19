#!/usr/bin/env python3
"""
sil/run_fake_orin.py - Runs the UNMODIFIED scripts/arm_teleop_rx.py as a fake Orin.

Only two things differ from the real Orin, both outside the receiver file:
  1. rpc_get_action() is redirected from 192.168.100.100:56322 to the fake MC's mock
     (127.0.0.1:SIL_RPC_PORT). The function body is the original one.
  2. --allow-ip defaults to 127.0.0.1 (loopback SIL), which makes the receiver bind 127.0.0.1.

Import layout mirrors the Orin's flat directory (mc/): only scripts/ is on sys.path, so the
receiver resolves `orin_teleop_core`, `orin_trace_logger`, `async_writer` exactly as deployed.
The receiver computes its own SHA256 from __file__, so the SHA in feedback/logs is the real one.

Usage (normally via sil/sil_up.sh):  python3 sil/run_fake_orin.py [--speed N] [--dry] [--expect-sha SHA]
"""
import os
import sys
import hashlib
import argparse

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(REPO, "scripts")
DEPLOYED = ["arm_teleop_rx.py", "orin_teleop_core.py", "orin_trace_logger.py", "async_writer.py"]


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--expect-sha", default=os.environ.get("SIL_EXPECT_SHA"))
    known, passthrough = ap.parse_known_args()

    dom = os.environ.get("ROS_DOMAIN_ID", "0")
    if dom == "232" or os.environ.get("ROS_LOCALHOST_ONLY", "0") != "1":
        print(f"FATAL: refusing to run fake Orin with ROS_DOMAIN_ID={dom} / ROS_LOCALHOST_ONLY="
              f"{os.environ.get('ROS_LOCALHOST_ONLY')} (must be isolated from the real robot)")
        sys.exit(2)

    print("[fake_orin] deployed file SHA256 (compare with the Orin copy before a real session):")
    for fn in DEPLOYED:
        print(f"    {sha256(os.path.join(SCRIPTS, fn))}  {fn}")
    rx_sha = sha256(os.path.join(SCRIPTS, "arm_teleop_rx.py"))
    if known.expect_sha and rx_sha != known.expect_sha:
        print(f"FATAL: arm_teleop_rx.py SHA {rx_sha} != expected {known.expect_sha}")
        sys.exit(3)

    # Flat import layout like /agibot/data/home/agi/Desktop/mc/
    sys.path[:] = [p for p in sys.path if os.path.abspath(p or ".") not in (REPO, os.path.join(REPO, "sil"))]
    sys.path.insert(0, SCRIPTS)
    import orin_teleop_core as core  # noqa: E402
    import arm_teleop_rx as rx       # noqa: E402

    rpc_port = int(os.environ.get("SIL_RPC_PORT", "56322"))
    orig_rpc = core.rpc_get_action

    def rpc_get_action_sil(host="192.168.100.100", port=56322):
        return orig_rpc(host="127.0.0.1", port=rpc_port)

    core.rpc_get_action = rpc_get_action_sil
    rx.rpc_get_action = rpc_get_action_sil  # `from orin_teleop_core import *` copied the name

    if not any(a.startswith("--allow-ip") for a in passthrough):
        passthrough = ["--allow-ip", "127.0.0.1"] + passthrough
    sys.argv = [rx.__file__] + passthrough
    print(f"[fake_orin] exec arm_teleop_rx.main() argv={passthrough} cwd={os.getcwd()}", flush=True)
    rx.main()


if __name__ == "__main__":
    main()
