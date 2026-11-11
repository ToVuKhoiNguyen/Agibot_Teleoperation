
const _tmpQ2 = new THREE.Quaternion();

// 1-frame buffer linear interpolation for smooth 60 Hz streaming without teleporting
function applyLatestMovingPoses(dtSec) {
  if (typeof poseBuffer === 'undefined' || poseBuffer.length === 0) {
    if (!latestMovingPoses) return;
    for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
      const obj = geomObjectMap[MOVING_GEOM_IDS[i]];
      const tgt = latestMovingPoses[i];
      if (obj && tgt) {
        obj.position.set(tgt.x, tgt.y, tgt.z);
        obj.quaternion.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
      }
    }
    return;
  }

  // Buffer delay: 1 frame at 60 Hz = 16.7 ms
  const renderTime = performance.now() - 16.7;

  if (poseBuffer.length === 1 || renderTime <= poseBuffer[0].t) {
    const poses = poseBuffer[0].poses;
    for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
      const obj = geomObjectMap[MOVING_GEOM_IDS[i]];
      const tgt = poses[i];
      if (obj && tgt) {
        obj.position.set(tgt.x, tgt.y, tgt.z);
        obj.quaternion.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
      }
    }
    return;
  }

  const lastIdx = poseBuffer.length - 1;
  if (renderTime >= poseBuffer[lastIdx].t) {
    const poses = poseBuffer[lastIdx].poses;
    for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
      const obj = geomObjectMap[MOVING_GEOM_IDS[i]];
      const tgt = poses[i];
      if (obj && tgt) {
        obj.position.set(tgt.x, tgt.y, tgt.z);
        obj.quaternion.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
      }
    }
    return;
  }

  // Find spanning interval [s0, s1]
  let s0 = poseBuffer[0];
  let s1 = poseBuffer[1];
  for (let i = 0; i < lastIdx; i++) {
    if (poseBuffer[i].t <= renderTime && renderTime <= poseBuffer[i + 1].t) {
      s0 = poseBuffer[i];
      s1 = poseBuffer[i + 1];
      break;
    }
  }

  const span = Math.max(1e-3, s1.t - s0.t);
  const alpha = Math.max(0.0, Math.min(1.0, (renderTime - s0.t) / span));

  for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
    const obj = geomObjectMap[MOVING_GEOM_IDS[i]];
    if (!obj) continue;
    const p0 = s0.poses[i];
    const p1 = s1.poses[i];
    if (!p0 || !p1) continue;

    obj.position.x = p0.x + (p1.x - p0.x) * alpha;
    obj.position.y = p0.y + (p1.y - p0.y) * alpha;
    obj.position.z = p0.z + (p1.z - p0.z) * alpha;

    _tmpQ.set(p0.qx, p0.qy, p0.qz, p0.qw);
    _tmpQ2.set(p1.qx, p1.qy, p1.qz, p1.qw);
    obj.quaternion.copy(_tmpQ).slerp(_tmpQ2, alpha);
  }
}

let lastRenderTime = performance.now();
let renderFrameCount = 0;
let lastFpsUpdateTime = performance.now();
let lastFrameDurationMs = 0;
let frameGapsLast5s = [];

function animate(currentTime) {
  if (inXRSession) return; // In XR session, onXRFrame handles rendering
  requestAnimationFrame(animate);

  const frameDeltaMs = currentTime - lastRenderTime;
  if (frameDeltaMs > 50 && lastRenderTime > 0) {
    frameGapsLast5s.push(currentTime);
  }
  while (frameGapsLast5s.length > 0 && currentTime - frameGapsLast5s[0] > 5000) {
    frameGapsLast5s.shift();
  }

  const dtSec = Math.min(0.05, Math.max(0.001, frameDeltaMs / 1000.0));
  lastRenderTime = currentTime;

  // Apply latest poses with zero buffer delay
  if (latestMovingPoses) {
    applyLatestMovingPoses(dtSec);
  }

  // Active rendering while robot moves; on-demand when idle > 1.0s
  const isRecalling = (typeof isGizmoRecalling !== 'undefined' && isGizmoRecalling());
  const isMoving = (currentTime - lastMovementTime < 1000);
  if (isMoving || sceneNeedsRender || orbitControls.state !== -1 || isDraggingGizmo || isRecalling) {
    if (typeof updateGizmoRecallAnimation === 'function') updateGizmoRecallAnimation(currentTime);
    if (typeof updateGhostReachVisuals === 'function') updateGhostReachVisuals();
    const t0 = performance.now();
    orbitControls.update();
    renderer.render(scene, camera);
    lastFrameDurationMs = performance.now() - t0;
    renderFrameCount++;
    if (!isMoving && orbitControls.state === -1 && !isDraggingGizmo) {
      sceneNeedsRender = false;
    }
  }

  // Update debug stats periodically
  if (currentTime - lastFpsUpdateTime >= 250) {
    const elapsed = currentTime - lastFpsUpdateTime;
    const currentFps = Math.round((renderFrameCount * 1000) / elapsed);
    renderFrameCount = 0;
    lastFpsUpdateTime = currentTime;
    if (typeof showStats !== 'undefined' && showStats) {
      const statsEl = document.getElementById('stats-overlay');
      if (statsEl) {
        const calls = renderer.info.render.calls;
        const tris = renderer.info.render.triangles;
        const dpr = renderer.getPixelRatio().toFixed(1);
        const msgs = (typeof measuredStateRateHz !== 'undefined') ? measuredStateRateHz.toFixed(0) : '0';
        const age = (typeof lastMessageAgeMs !== 'undefined') ? lastMessageAgeMs.toFixed(0) : '0';
        statsEl.innerText = `${currentFps} FPS | ${lastFrameDurationMs.toFixed(1)} ms | ${calls} calls | ${tris.toLocaleString()} tris | DPR ${dpr} | ${msgs} msgs/s | ${age} ms`;
      }
    }

    if (isDebug) {
      const fpsEl = document.getElementById('dbg-fps');
      const timeEl = document.getElementById('dbg-frametime');
      const callsEl = document.getElementById('dbg-calls');
      const trisEl = document.getElementById('dbg-tris');
      const dprEl = document.getElementById('dbg-dpr');
      const rateEl = document.getElementById('dbg-rate');
      const ageEl = document.getElementById('dbg-age');
      const gapsEl = document.getElementById('dbg-gaps');
      const loopEl = document.getElementById('dbg-loop-hz');
      if (fpsEl) fpsEl.innerText = currentFps;
      if (gapsEl) gapsEl.innerText = frameGapsLast5s.length;
      if (timeEl) timeEl.innerText = lastFrameDurationMs.toFixed(1);
      if (callsEl) callsEl.innerText = renderer.info.render.calls;
      if (trisEl) trisEl.innerText = renderer.info.render.triangles.toLocaleString();
      if (dprEl) dprEl.innerText = renderer.getPixelRatio().toFixed(1);
      if (loopEl && typeof latestStatusData !== 'undefined' && latestStatusData.loop_hz) {
        loopEl.innerText = Math.round(latestStatusData.loop_hz);
      }
      if (rateEl) rateEl.innerText = measuredStateRateHz.toFixed(0);
      if (ageEl) ageEl.innerText = lastMessageAgeMs.toFixed(0);
    }
  }
}

