#!/usr/bin/env python3
"""
sil/sil_ctl.py - Fault injection / status CLI for the SIL fake MC (stdlib only, any Python).

  python sil/sil_ctl.py status
  python sil/sil_ctl.py freeze on|off        # stale feedback
  python sil/sil_ctl.py dual on|off          # second publisher on arm_joint_command
  python sil/sil_ctl.py action 10            # GetAction answers something other than 402
  python sil/sil_ctl.py burst 3              # bursty state delivery
  python sil/sil_ctl.py push <joint> <rad>   # external disturbance
"""
import os
import sys
import json
import socket

CTL_PORT = int(os.environ.get("SIL_CTL_PORT", "5099"))
# a2sim (sil/a2sim): the robot lives in its own network namespace at 192.168.2.50; loopback SIL: 127.0.0.1
CTL_HOST = os.environ.get("SIL_CTL_HOST") or ("192.168.2.50" if os.path.exists("/sys/class/net/a2sim-host") else "127.0.0.1")


def ctl(req: dict, timeout: float = 1.0) -> dict:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(req).encode(), (CTL_HOST, CTL_PORT))
        data, _ = s.recvfrom(65535)
        return json.loads(data.decode())
    finally:
        s.close()


def main(argv):
    if not argv:
        print(__doc__); return 1
    c, a = argv[0], argv[1:]
    onoff = lambda: (a[0].lower() in ("on", "1", "true")) if a else True  # noqa: E731
    req = {
        "status": lambda: {"cmd": "status"},
        "freeze": lambda: {"cmd": "freeze_state", "on": onoff()},
        "dual": lambda: {"cmd": "dual_publisher", "on": onoff()},
        "action": lambda: {"cmd": "set_action", "value": a[0]},
        "burst": lambda: {"cmd": "state_burst", "n": int(a[0])},
        "push": lambda: {"cmd": "push", "joint": int(a[0]), "delta": float(a[1])},
    }.get(c)
    if req is None:
        print(__doc__); return 1
    print(json.dumps(ctl(req()), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
