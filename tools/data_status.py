#!/usr/bin/env python3
"""tools/data_status.py - One screen: what data do I have, where is it, what happened to it (laptop, teleop venv).

    venv/bin/python tools/data_status.py                 # everything
    venv/bin/python tools/data_status.py --task pick_cup # one task
    venv/bin/python tools/data_status.py --quick         # skip the quality check (faster with many episodes)

RAW EPISODES  recordings/<task>/<date>/ep_*   what the robot / operator recorded (keep: everything can be re-exported)
DATASETS      lerobot_datasets/<name>      LeRobot v3.0, what the model trains on
For every raw episode: duration, OK / CHECK / BAD (tools/check_episode.py), discarded or not, and which dataset
(and episode number in it) it was exported to, or "NOT EXPORTED" with the likely reason.
Only reads. Nothing is changed.
"""
import argparse, importlib.util, json, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATA, DATASETS = REPO / "recordings", REPO / "lerobot_datasets"


def source_dir(rel: str) -> Path:
    p = REPO / rel
    if not p.exists() and rel.startswith("data/") and (REPO / "recordings" / rel[5:]).exists(): p = REPO / "recordings" / rel[5:]
    return p


def du(p: Path) -> int:
    return sum(f.stat().st_size for f in p.rglob("*") if f.is_file()) if p.exists() else 0


def human(n: float) -> str:
    for u in ("B", "KB", "MB", "GB"):
        if n < 1024 or u == "GB": return f"{n:.0f} {u}" if u == "B" else f"{n:.1f} {u}"
        n /= 1024


def load_check():
    spec = importlib.util.spec_from_file_location("check_episode", REPO / "tools" / "check_episode.py")
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task"); ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()

    # ---- datasets and what is in them
    ds_rows, exported = [], {}
    if DATASETS.exists():
        for d in sorted(DATASETS.iterdir()):
            info = d / "meta" / "info.json"
            if not info.exists(): continue
            try: i = json.loads(info.read_text())
            except Exception: continue
            src = d / "meta" / "a2_sources.json"
            srcs = json.loads(src.read_text())["episodes"] if src.exists() else None
            ds_rows.append((d, i, srcs))
            for e in srcs or []:
                exported.setdefault(source_dir(e["source"]).resolve(), []).append((d.name, e["episode_index"]))

    # ---- raw episodes
    chk = None if a.quick else load_check()
    eps = sorted(p.parent for p in DATA.glob(f"{a.task or '*'}/*/ep_*/meta.json")) if DATA.exists() else []
    print(f"\nRAW EPISODES   {DATA}   (laptop; ghi bằng nút B)")
    if not eps: print("  (chưa có episode nào)")
    counts = {"OK": 0, "CHECK": 0, "BAD": 0, "discarded": 0, "not_exported": 0}
    cur = None
    for ep in eps:
        task, day = ep.parent.parent.name, ep.parent.name
        if (task, day) != cur:
            cur = (task, day); print(f"\n  {task} / {day}")
        try: meta = json.loads((ep / "meta.json").read_text())
        except Exception: meta = {}
        dur = meta.get("duration_s")
        disc = bool(meta.get("discarded"))
        verdict = "-"
        if chk is not None and not disc and dur:
            try:
                r = chk.analyse(ep); verdict = "BAD" if r["bad"] else ("CHECK" if r["warn"] else "OK")
            except Exception: verdict = "?"
        where = exported.get(ep.resolve())
        if disc: state = f"BỎ ({meta.get('discard_reason', '')})"; counts["discarded"] += 1
        else:
            if verdict in counts: counts[verdict] += 1
            state = verdict
        if where: dest = "-> " + ", ".join(f"lerobot_datasets/{n} #{k}" for n, k in where)
        elif disc: dest = "(không xuất vì đã bỏ)"
        elif not dur: dest = "NOT EXPORTED: tập chưa đóng (đang ghi hoặc laptop tắt giữa chừng)"
        else: dest = "NOT EXPORTED: chưa tự xuất (xem dòng LeRobot trên thẻ Data Recording) hoặc xuất trước 17/11"; counts["not_exported"] += 1
        instr = (meta.get("instruction") or "")[:40]
        print(f"    {ep.name:20} {('%.0f s' % dur) if dur else '  ? ':>6}  {state:22} {human(du(ep)):>9}  \"{instr}\"  {dest}")

    print(f"\nDATASETS   {DATASETS}   (LeRobot v3.0, dùng để train)")
    if not ds_rows: print("  (chưa có dataset nào)")
    for d, i, srcs in ds_rows:
        cams = [k.split(".")[-1] for k, v in i.get("features", {}).items() if v.get("dtype") in ("video", "image")]
        print(f"  lerobot_datasets/{d.name:20} {i.get('total_episodes')} tập, {i.get('total_frames')} khung, {i.get('fps')} fps, "
              f"{human(du(d))}, camera: {', '.join(cams) or '-'}{'' if srcs is not None else '   (không có nguồn: xuất trước 17/11, chỉ xem)'}")

    print(f"\nTổng: {len(eps)} tập thô ({human(du(DATA))}), {len(ds_rows)} dataset ({human(du(DATASETS))}).  "
          f"OK {counts['OK']} · CHECK {counts['CHECK']} · BAD {counts['BAD']} · bỏ {counts['discarded']} · chưa xuất {counts['not_exported']}")
    print("Xem: panel 8012 -> nút Dataset. Tập thô: venv/bin/python tools/view_episode.py --task <task>")


if __name__ == "__main__":
    sys.exit(main())
