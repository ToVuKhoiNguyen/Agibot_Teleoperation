#!/usr/bin/env bash
# tools/pull_episodes.sh - copy recorded episodes from the Orin into the laptop's recordings/ tree.
# Robot:  /agibot/data/home/agi/Desktop/mc/data/<task>/<date>/<episode>/{robot/,meta.json,ros2bag.log}
# Laptop: recordings/<task>/<date>/<episode>/{laptop.jsonl,meta.json}  +  robot/, robot_meta.json, ros2bag.log (after this script)
# Usage:  tools/pull_episodes.sh            # everything
#         tools/pull_episodes.sh pick_cube  # one task
#         tools/pull_episodes.sh pick_cube 2026-11-13
# Nothing is deleted on the robot; free its disk by hand once the copy is verified.
set -euo pipefail
ROBOT="${ROBOT:-agi@192.168.2.50}"
REMOTE_ROOT="${REMOTE_ROOT:-/agibot/data/home/agi/Desktop/mc/data}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUB="${1:-}"; [[ -n "${2:-}" ]] && SUB="$SUB/$2"
mkdir -p "$HERE/recordings/$SUB"
echo "[pull] $ROBOT:$REMOTE_ROOT/$SUB/  ->  $HERE/recordings/$SUB/"
# the robot's meta.json is stored as robot_meta.json so it does not overwrite the laptop's meta.json
rsync -av --partial --info=progress2 --exclude 'meta.json' "$ROBOT:$REMOTE_ROOT/$SUB/" "$HERE/recordings/$SUB/"
rsync -av --include '*/' --include 'meta.json' --exclude '*' "$ROBOT:$REMOTE_ROOT/$SUB/" "$HERE/recordings/.robot_meta_tmp/$SUB/"
( cd "$HERE/recordings/.robot_meta_tmp" && find . -name meta.json | while read -r f; do
    d="$HERE/recordings/$(dirname "$f")"; mkdir -p "$d"; cp "$f" "$d/robot_meta.json"; done )
rm -rf "$HERE/recordings/.robot_meta_tmp"
echo "[pull] done. Episodes on this laptop:"; find "$HERE/recordings" -maxdepth 3 -mindepth 3 -type d | sort | tail -20
