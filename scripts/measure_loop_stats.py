"""
scripts/measure_loop_stats.py
Measures loop Hz, max step ms, and jitter over 5 seconds of headless execution.
"""

import sys
import os
import time
import numpy as np

# Ensure project root is in sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import UnifiedTeleopApp


def measure_stats(duration_s=5.0):
    app = UnifiedTeleopApp(render_viewer=False, no_adb=True, no_quest_launch=True, port=8013)
    target_dt = app.control_dt  # 0.005 (200 Hz)

    step_durations = []
    loop_intervals = []

    print(f"[BENCHMARK] Warming up and measuring {duration_s:.1f}s of headless loop...")
    t_start = time.perf_counter()
    next_step_time = t_start
    last_loop_time = t_start

    step = 0
    try:
        while time.perf_counter() - t_start < duration_s:
            t0 = time.perf_counter()
            # Execute one step equivalent
            app._update_sequence(target_dt)
            app._update_walk_demo()
            app._update_hand_sweep(target_dt)
            app._update_homing(target_dt)
            app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)

            t1 = time.perf_counter()
            step_durations.append((t1 - t0) * 1000.0)

            # Fixed-rate sleep
            next_step_time += target_dt
            now_perf = time.perf_counter()
            sleep_t = next_step_time - now_perf
            if sleep_t > 0.0005:
                time.sleep(sleep_t)
            elif sleep_t < -0.05:
                next_step_time = now_perf

            t_end = time.perf_counter()
            loop_intervals.append(t_end - last_loop_time)
            last_loop_time = t_end
            step += 1
    finally:
        app.close()

    total_time = loop_intervals[-1] if not loop_intervals else (time.perf_counter() - t_start)
    loop_hz = len(loop_intervals) / (time.perf_counter() - t_start)
    max_step_ms = float(np.max(step_durations))
    mean_step_ms = float(np.mean(step_durations))
    jitter_ms = float(np.std(loop_intervals) * 1000.0)

    print(f"[BENCHMARK RESULTS] Over {step} steps ({time.perf_counter() - t_start:.2f}s):")
    print(f"  Loop rate : {loop_hz:.1f} Hz")
    print(f"  Step time : mean {mean_step_ms:.2f} ms | max {max_step_ms:.2f} ms")
    print(f"  Jitter    : {jitter_ms:.2f} ms")

    return {
        "loop_hz": loop_hz,
        "mean_step_ms": mean_step_ms,
        "max_step_ms": max_step_ms,
        "jitter_ms": jitter_ms,
    }


if __name__ == "__main__":
    measure_stats()
