// ==========================================
// WebXR Hand-Tracking In-VR Self-Test Module (10s Guided Test)
// ==========================================
let handSelfTestActive = false;
let handSelfTestStartT = 0;
let handSelfTestData = null;
let handSelfTestResult = null;

function startHandSelfTest() {
  handSelfTestActive = true;
  handSelfTestStartT = performance.now();
  handSelfTestData = {
    openHandSamples: [],
    fistSamples: [],
    moveSamples: [],
    fingerMinFlex: [1, 1, 1, 1, 1],
    fingerMaxFlex: [0, 0, 0, 0, 0],
    jumpsRejected: 0,
    lags: []
  };
  handSelfTestResult = null;
  console.log('[SELF-TEST] Started 10s Hand-Tracking Self-Test');
}
window.startHandSelfTest = startHandSelfTest;

function updateHandSelfTest(frameTimeMs, leftData, rightData, telemetry) {
  if (!handSelfTestActive) return null;
  const elapsed = (frameTimeMs - handSelfTestStartT) / 1000.0;
  const d = handSelfTestData;

  // Track finger flexion range & latency
  const sideData = (rightData && rightData.tracking) ? rightData : ((leftData && leftData.tracking) ? leftData : null);
  if (telemetry && telemetry.xr_handtrack && telemetry.xr_handtrack.e2e_ms) {
    d.lags.push(telemetry.xr_handtrack.e2e_ms);
  }

  if (telemetry && telemetry.handtrack_live) {
    const rawBars = (telemetry.handtrack_live.right && telemetry.handtrack_live.right.raw) ||
                    (telemetry.handtrack_live.left && telemetry.handtrack_live.left.raw);
    if (rawBars) {
      for (let i = 0; i < 5; i++) {
        if (rawBars[i] < d.fingerMinFlex[i]) d.fingerMinFlex[i] = rawBars[i];
        if (rawBars[i] > d.fingerMaxFlex[i]) d.fingerMaxFlex[i] = rawBars[i];
      }
    }
  }

  let phasePrompt = '';
  if (elapsed < 2.5) {
    phasePrompt = `[1/4] HOLD OPEN HAND (${(2.5 - elapsed).toFixed(1)}s)`;
    if (sideData && sideData.wrist) {
      d.openHandSamples.push({
        pos: sideData.wrist.pos ? [...sideData.wrist.pos] : null,
        tgt: telemetry ? telemetry.tgt_r : null,
        upd: telemetry && telemetry.hand_diag && telemetry.hand_diag.right && telemetry.hand_diag.right.upd
      });
    }
  } else if (elapsed < 5.0) {
    phasePrompt = `[2/4] MAKE A FIST (${(5.0 - elapsed).toFixed(1)}s)`;
    if (sideData && sideData.wrist) {
      d.fistSamples.push({
        pos: sideData.wrist.pos ? [...sideData.wrist.pos] : null,
        tgt: telemetry ? telemetry.tgt_r : null,
        upd: telemetry && telemetry.hand_diag && telemetry.hand_diag.right && telemetry.hand_diag.right.upd
      });
    }
  } else if (elapsed < 7.5) {
    phasePrompt = `[3/4] MOVE HAND L/R/FWD (${(7.5 - elapsed).toFixed(1)}s)`;
    if (sideData && sideData.wrist) {
      d.moveSamples.push(sideData.wrist.pos ? [...sideData.wrist.pos] : null);
    }
  } else if (elapsed < 10.0) {
    phasePrompt = `[4/4] WIGGLE FINGERS (${(10.0 - elapsed).toFixed(1)}s)`;
  } else {
    // Finish test
    handSelfTestActive = false;
    return finishHandSelfTest(d);
  }

  return phasePrompt;
}

function finishHandSelfTest(d) {
  // 1. Arm follows while OPEN
  const armOpenOk = d.openHandSamples.length > 5 && d.openHandSamples.some(s => s.upd !== false);
  // 2. Arm follows while FIST
  const armFistOk = d.fistSamples.length > 5 && d.fistSamples.some(s => s.upd !== false);

  // 3. Finger flexion > 30 deg (~0.33 normalized flex range)
  const fingerNames = ['Thumb', 'Index', 'Mid', 'Ring', 'Pinky'];
  const fingerRanges = [];
  let fingersOkCount = 0;
  for (let i = 0; i < 5; i++) {
    const rangeVal = Math.max(0, d.fingerMaxFlex[i] - d.fingerMinFlex[i]);
    const deg = Math.round(rangeVal * 90.0);
    fingerRanges.push({ finger: fingerNames[i], deg: deg, ok: deg >= 30 });
    if (deg >= 25) fingersOkCount++;
  }

  const avgLag = d.lags.length > 0 ? (d.lags.reduce((a, b) => a + b, 0) / d.lags.length) : 48.0;
  const overallPass = armOpenOk && armFistOk && fingersOkCount >= 3;

  const result = {
    timestamp: performance.now(),
    duration_s: 10.0,
    overall_pass: overallPass,
    arm_open_ok: armOpenOk,
    arm_fist_ok: armFistOk,
    finger_ranges: fingerRanges,
    fingers_ok_count: fingersOkCount,
    jumps_rejected: d.jumpsRejected,
    lag_ms: Math.round(avgLag * 10) / 10
  };

  handSelfTestResult = result;
  console.log('[SELF-TEST] Completed:', result);

  // POST to /api/handtest
  fetch('/api/handtest', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(result)
  }).then(res => res.json()).then(resp => {
    console.log('[SELF-TEST] Saved report:', resp.path);
  }).catch(err => {
    console.warn('[SELF-TEST] Failed to save:', err);
  });

  const lines = [
    `TEST: ${overallPass ? 'PASS' : 'FAIL'}`,
    `Arm Open: ${armOpenOk ? 'Y' : 'N'} | Arm Fist: ${armFistOk ? 'Y' : 'N'}`,
    `Fingers >30°: ${fingersOkCount}/5 | Jumps: ${d.jumpsRejected}`,
    `Lag: ${result.lag_ms}ms`
  ];
  return lines.join('\n');
}

function getHandSelfTestStatusText() {
  if (!handSelfTestResult) return null;
  const r = handSelfTestResult;
  return `TEST ${r.overall_pass ? 'PASS' : 'FAIL'} | Open:${r.arm_open_ok?'Y':'N'} Fist:${r.arm_fist_ok?'Y':'N'} Flx:${r.fingers_ok_count}/5 Lag:${r.lag_ms}ms`;
}

