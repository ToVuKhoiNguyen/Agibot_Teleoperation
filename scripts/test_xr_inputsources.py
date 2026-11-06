#!/usr/bin/env python3
"""
scripts/test_xr_inputsources.py
Verifies WebXR XRInputSourceArray handling with non-Array iterable mock:
  - Mock session.inputSources with ONLY length, [i], forEach, Symbol.iterator (no .some, .map, etc.)
  - Test Controller input source (auto-selects 'controllers', rate > 0)
  - Test Hand input source (auto-selects 'hands', rate > 0)
  - ~60 frames loop execution, 0 console errors, screenshots saved
"""

import sys, time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.utils.server_process import ensure_server_running, stop_spawned_server


def run_test():
    print("=" * 80)
    print("RUNNING WEBXR NON-ARRAY XRINPUTSOURCEARRAY ACCEPTANCE TEST")
    print("=" * 80)

    spawned_proc, did_spawn = None, False
    try:
        spawned_proc, did_spawn = ensure_server_running(timeout_s=15.0)
        from playwright.sync_api import sync_playwright

        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=["--ignore-certificate-errors"])
            ctx = browser.new_context(ignore_https_errors=True)
            page = ctx.new_page()

            console_errs = []
            page.on("console", lambda m: console_errs.append(m.text) if m.type == "error" else None)
            page.set_viewport_size({"width": 1440, "height": 900})
            page.goto("https://localhost:8012/", timeout=10000)
            page.wait_for_selector("#viewport-container", timeout=8000)

            # Inject WebXR mock with non-Array iterable inputSources
            res = page.evaluate('''async () => {
                function createNonArraySources(items) {
                    const obj = {
                        length: items.length,
                        forEach(cb) { for (let i = 0; i < items.length; i++) cb(items[i], i, this); },
                        [Symbol.iterator]() {
                            let idx = 0;
                            return {
                                next: () => idx < items.length ? { value: items[idx++], done: false } : { done: true }
                            };
                        }
                    };
                    items.forEach((item, i) => { obj[i] = item; });
                    Object.setPrototypeOf(obj, Object.prototype);
                    return obj;
                }

                // Verify it is not an Array
                const sample = createNonArraySources([]);
                if (Array.isArray(sample) || typeof sample.some !== 'undefined') {
                    throw new Error("Mock inputSources must be non-Array without .some");
                }

                const names = ['wrist', 'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
                    'index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip',
                    'middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip',
                    'ring-finger-metacarpal', 'ring-finger-phalanx-proximal', 'ring-finger-phalanx-intermediate', 'ring-finger-phalanx-distal', 'ring-finger-tip',
                    'pinky-finger-metacarpal', 'pinky-finger-phalanx-proximal', 'pinky-finger-phalanx-intermediate', 'pinky-finger-phalanx-distal', 'pinky-finger-tip'];
                const handMap = new Map();
                names.forEach((n, idx) => handMap.set(n, { name: n, index: idx }));

                const ctrlSource = {
                    handedness: 'right',
                    targetRayMode: 'tracked-pointer',
                    gripSpace: {},
                    targetRaySpace: {},
                    gamepad: {
                        buttons: [
                            { value: 0.0, pressed: false },
                            { value: 0.0, pressed: false },
                            { value: 0.0, pressed: false },
                            { value: 0.0, pressed: false },
                            { value: 0.0, pressed: false },
                            { value: 0.0, pressed: false }
                        ],
                        axes: [0.0, 0.0, 0.0, 0.0]
                    }
                };

                const handSource = {
                    handedness: 'left',
                    targetRayMode: 'tracked-pointer',
                    hand: handMap
                };

                const session = new EventTarget();
                session.enabledFeatures = ['local-floor', 'hand-tracking'];
                session.requestReferenceSpace = (t) => Promise.resolve({});
                let currentItems = [ctrlSource];
                session.inputSources = createNonArraySources(currentItems);

                let frameCount = 0;
                session.requestAnimationFrame = (cb) => {
                    setTimeout(() => {
                        frameCount++;
                        cb(performance.now(), {
                            getViewerPose: () => ({
                                transform: { position: { x: 0, y: 1.5, z: 0 }, orientation: { w: 1, x: 0, y: 0, z: 0 } }
                            }),
                            getPose: () => ({
                                transform: { position: { x: 0.2, y: 1.2, z: -0.4 }, orientation: { w: 1, x: 0, y: 0, z: 0 } }
                            }),
                            getJointPose: (j) => ({
                                transform: { position: { x: 0.205, y: 1.30, z: -0.45 - ((j.index||0)>0?0.05:0) }, orientation: { w: 1, x: 0, y: 0, z: 0 } }
                            })
                        });
                    }, 16);
                };
                session.end = () => Promise.resolve();

                await onXRSessionStarted(session, 'immersive-vr');

                // --- Phase 1: Controller Source (~60 frames) ---
                session.inputSources = createNonArraySources([ctrlSource]);
                if (typeof onXRInputSourcesChange === 'function') {
                    onXRInputSourcesChange({ session: session });
                }
                for (let i = 0; i < 65; i++) {
                    await new Promise(r => setTimeout(r, 16));
                }
                const ctrlRes = {
                    rate: window.handPacketsSec,
                    mode: window.confirmedInputMode || window.currentInputMode,
                    err: window.lastXRError,
                    frames: frameCount
                };

                // --- Phase 2: Hand Source (~60 frames) ---
                session.inputSources = createNonArraySources([handSource]);
                if (typeof onXRInputSourcesChange === 'function') {
                    onXRInputSourcesChange({ session: session });
                }
                for (let i = 0; i < 65; i++) {
                    await new Promise(r => setTimeout(r, 16));
                }
                const handRes = {
                    rate: window.handPacketsSec,
                    mode: window.confirmedInputMode || window.currentInputMode,
                    err: window.lastXRError,
                    frames: frameCount
                };

                // Phase 3: Stress test 300 frames with corrupted/missing data (undefined handedness, null hand/gamepad, missing joints, null poses)
                let edgeCases = [
                    {},
                    { handedness: undefined, targetRayMode: 'tracked-pointer' },
                    { handedness: 'left', hand: null, targetRayMode: 'tracked-pointer' },
                    { handedness: 'right', gamepad: null, targetRayMode: 'tracked-pointer' },
                    { handedness: 'left', hand: new Map([['wrist', {name: 'wrist', index: 0}]]), targetRayMode: 'tracked-pointer' }
                ];
                for (let c = 0; c < edgeCases.length; c++) {
                    session.inputSources = createNonArraySources([edgeCases[c]]);
                    if (typeof onXRInputSourcesChange === 'function') onXRInputSourcesChange({ session: session });
                    for (let f = 0; f < 60; f++) {
                        await new Promise(r => setTimeout(r, 8));
                    }
                }

                return { ctrl: ctrlRes, hand: handRes };
            }''')

            shot_dir = Path("/tmp/shots")
            shot_dir.mkdir(parents=True, exist_ok=True)
            shot_file = shot_dir / "xr_inputsources.png"
            page.screenshot(path=str(shot_file))
            browser.close()

            print(f"  Console errors: {len(console_errs)}")
            if console_errs:
                print(f"  [ERROR] Captured errors: {console_errs}")

            ctrl = res["ctrl"]
            hand = res["hand"]
            print(f"  [CONTROLLER] mode={ctrl['mode']}, rate={ctrl['rate']} pkts/s, err={ctrl['err']}, frames={ctrl['frames']}")
            print(f"  [HAND]       mode={hand['mode']}, rate={hand['rate']} pkts/s, err={hand['err']}, frames={hand['frames']}")

            assert len(console_errs) == 0, f"Expected 0 console errors, got {len(console_errs)}: {console_errs}"
            assert ctrl["mode"] == "controllers", f"Expected mode controllers, got {ctrl['mode']}"
            assert ctrl["rate"] > 0, f"Expected controller rate > 0, got {ctrl['rate']}"
            assert not ctrl["err"], f"Controller error: {ctrl['err']}"

            assert hand["mode"] == "hands", f"Expected mode hands, got {hand['mode']}"
            assert hand["rate"] > 0, f"Expected hand rate > 0, got {hand['rate']}"
            assert not hand["err"], f"Hand error: {hand['err']}"

            assert shot_file.exists() and shot_file.stat().st_size > 5000
            print(f"  [PASS] Screenshot saved: {shot_file} ({shot_file.stat().st_size} bytes)")
            print("\n" + "=" * 80)
            print("ALL XRINPUTSOURCEARRAY ACCEPTANCE TESTS PASSED!")
            print("=" * 80)
            return True

    except Exception as e:
        print(f"[FAIL] test_xr_inputsources failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        if did_spawn and spawned_proc:
            stop_spawned_server(spawned_proc)


if __name__ == "__main__":
    success = run_test()
    sys.exit(0 if success else 1)

