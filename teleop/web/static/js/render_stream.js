
// Zero-delay direct pose application + fast exponential smoothing (tau ~ 25 ms)
function applyLatestMovingPoses(dtSec) {
  if (!latestMovingPoses) return;
  const factor = Math.min(1.0, dtSec / 0.025); // fast convergence

  for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
    const gid = MOVING_GEOM_IDS[i];
    const obj = geomObjectMap[gid];
    if (!obj) continue;

    const tgt = latestMovingPoses[i];
    if (!tgt) continue;

    if (factor >= 0.95) {
      obj.position.set(tgt.x, tgt.y, tgt.z);
      obj.quaternion.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
    } else {
      // Lerp position toward latest target
      obj.position.x += (tgt.x - obj.position.x) * factor;
      obj.position.y += (tgt.y - obj.position.y) * factor;
      obj.position.z += (tgt.z - obj.position.z) * factor;
      // Slerp quaternion toward latest target
      _tmpQ.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
      obj.quaternion.slerp(_tmpQ, factor);
    }
  }
}

let lastRenderTime = performance.now();
let renderFrameCount = 0;
let lastFpsUpdateTime = performance.now();
let lastFrameDurationMs = 0;

function animate(currentTime) {
  if (inXRSession) return; // In XR session, onXRFrame handles rendering
  requestAnimationFrame(animate);

  const dtSec = Math.min(0.05, Math.max(0.001, (currentTime - lastRenderTime) / 1000.0));
  lastRenderTime = currentTime;

  // Apply latest poses with zero buffer delay
  if (latestMovingPoses) {
    applyLatestMovingPoses(dtSec);
  }

  // Active rendering while robot moves; on-demand when idle > 1.0s
  const isMoving = (currentTime - lastMovementTime < 1000);
  if (isMoving || sceneNeedsRender || orbitControls.state !== -1 || isDraggingGizmo) {
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
    if (isDebug) {
      const fpsEl = document.getElementById('dbg-fps');
      const timeEl = document.getElementById('dbg-frametime');
      const callsEl = document.getElementById('dbg-calls');
      const trisEl = document.getElementById('dbg-tris');
      const dprEl = document.getElementById('dbg-dpr');
      const rateEl = document.getElementById('dbg-rate');
      const ageEl = document.getElementById('dbg-age');
      if (fpsEl) fpsEl.innerText = currentFps;
      if (timeEl) timeEl.innerText = lastFrameDurationMs.toFixed(1);
      if (callsEl) callsEl.innerText = renderer.info.render.calls;
      if (trisEl) trisEl.innerText = renderer.info.render.triangles.toLocaleString();
      if (dprEl) dprEl.innerText = renderer.getPixelRatio().toFixed(1);
      if (rateEl) rateEl.innerText = measuredStateRateHz.toFixed(0);
      if (ageEl) ageEl.innerText = lastMessageAgeMs.toFixed(0);
    }
  }
}

