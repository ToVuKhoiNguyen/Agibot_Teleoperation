// ==========================================
// WebXR Immersive-VR Session Management
// ==========================================
function getInputSources(s) {
  if (!s) return [];
  const raw = s.inputSources !== undefined ? s.inputSources : s;
  if (!raw) return [];
  return Array.isArray(raw) ? raw : Array.from(raw);
}
window.getInputSources = getInputSources;

let xrVisibilityHold = false;
let xrRequireButtonResume = false;

function checkXRSupport() {
  const container = document.getElementById('vr-corner-container');
  const btnCorner = document.getElementById('btn-vr-corner');
  const btnHud = document.getElementById('btn-enter-vr');
  if (container) container.style.display = 'none';
  if (btnCorner) btnCorner.style.display = 'none';
  if (btnHud) btnHud.style.display = 'none';
  if (!window.isSecureContext || !navigator.xr) return;

  navigator.xr.isSessionSupported('immersive-vr').then(supported => {
    if (supported) {
      if (container) container.style.display = 'flex';
      if (btnCorner) btnCorner.style.display = 'block';
      if (btnHud) btnHud.style.display = 'inline-block';
    }
  }).catch(err => {
    console.warn('isSessionSupported(immersive-vr) error:', err);
  });
}
window.checkXRSupport = checkXRSupport;
window.addEventListener('DOMContentLoaded', checkXRSupport);

function toggleXRRotation(checked) {
  xrRotationEnabled = Boolean(checked);
}

function enterVRDirectly() {
  if (xrSession) {
    xrSession.end();
    return;
  }
  if (!navigator.xr) return;

  const errOverlay = document.getElementById('vr-error-overlay');
  if (errOverlay) errOverlay.style.display = 'none';

  // Synchronous requestSession maintaining user gesture
  navigator.xr.requestSession('immersive-vr', {
    optionalFeatures: ['local-floor', 'hand-tracking']
  }).then(session => {
    onXRSessionStarted(session, 'immersive-vr');
  }).catch(err => {
    console.error('Failed to start WebXR session:', err);
    const msg = (err && (err.name ? (err.name + ': ' + err.message) : err.message)) || 'Unknown VR Error';
    if (errOverlay) {
      errOverlay.innerText = msg;
      errOverlay.style.display = 'block';
    }
  });
}

function toggleXRSession() {
  enterVRDirectly();
}

function onXRVisibilityChange() {
  const state = xrSession ? xrSession.visibilityState : 'hidden';
  if (state === 'hidden' || state === 'visible-blurred') {
    xrVisibilityHold = true;
    xrRequireButtonResume = true;
  } else if (state === 'visible') {
    xrVisibilityHold = true;
    xrRequireButtonResume = true;
  }
}

let xrDiagSprite = null;
window.lastXRError = null;
window.handPacketsSec = 0;
let handPacketsCounter = 0;
window.recordHandPacketSent = function() { handPacketsCounter++; };

setInterval(() => {
  window.handPacketsSec = handPacketsCounter;
  handPacketsCounter = 0;
  if (inXRSession || window.lastXRError) {
    console.log(`[XR-1S] pkts/s=${window.handPacketsSec} mode=${window.confirmedInputMode || 'none'} bld=${window.BUILD_ID || 'dev'} err=${window.lastXRError || 'None'}`);
    if (typeof updateXRDiagOverlay === 'function') updateXRDiagOverlay();
  }
}, 1000);

function updateXRDiagOverlay() {
  const info = window.xrSessionStartInfo || {};
  const granted = info.granted || 'unexposed';
  const opt = info.requestedHand ? 'hand-tracking, local-floor' : 'none';
  const srcs = getInputSources(xrSession);
  const srcStr = srcs.map(s => `${(s && s.handedness) || 'none'}: hand=${Boolean(s && s.hand)} gp=${Boolean(s && s.gamepad)} ray=${(s && s.targetRayMode) || 'none'}`).join(' | ') || 'none';
  const conf = window.confirmedInputMode || 'none';
  const bld = window.BUILD_ID || 'dev';
  const err = window.lastXRError || 'None';
  const fullText = `XR: granted=[${granted}] opt=[${opt}]\nSrcs(${srcs.length}): ${srcStr}\nRate: ${window.handPacketsSec} pkts/s | Conf: ${conf} | Bld: ${bld}\nLast Err: ${err}`;

  let el = document.getElementById('xr-diag-overlay');
  if (!el) {
    el = document.createElement('div');
    el.id = 'xr-diag-overlay';
    el.style.cssText = 'position:fixed;bottom:8px;left:8px;z-index:99999;background:rgba(10,16,24,0.92);border:1px solid #00e5ff;color:#e0f7fa;font-family:monospace;font-size:11px;padding:6px 10px;border-radius:4px;max-width:560px;pointer-events:none;line-height:1.4;white-space:pre-wrap;';
    document.body.appendChild(el);
  }
  el.innerText = fullText;
  el.style.borderColor = (err !== 'None') ? '#ff1744' : '#00e5ff';

  if (inXRSession && scene && typeof headYaw !== 'undefined') {
    if (!xrDiagSprite) {
      xrDiagSprite = createTextSprite('XR DIAG', '#00e5ff');
      scene.add(xrDiagSprite);
    }
    const hp = (typeof window.lastHeadPos !== 'undefined' && window.lastHeadPos) ? window.lastHeadPos : { x: 0, y: 1.5, z: 0 };
    const fwdX = -Math.sin(headYaw), fwdZ = -Math.cos(headYaw);
    xrDiagSprite.position.set(hp.x + 0.8 * fwdX, hp.y - 0.28, hp.z + 0.8 * fwdZ);
    if (typeof updateSpriteText === 'function') {
      updateSpriteText(xrDiagSprite, fullText, (err !== 'None') ? '#b71c1c' : '#006064');
    }
    xrDiagSprite.visible = true;
  }
}
window.updateXRDiagOverlay = updateXRDiagOverlay;

function logSessionStartDiag(session) {
  const granted = session.enabledFeatures ? Array.from(session.enabledFeatures).join(', ') : 'unexposed';
  const srcs = getInputSources(session);
  const srcDetails = srcs.map(s => `${(s && s.handedness) || 'none'}: hand=${Boolean(s && s.hand)}, gp=${Boolean(s && s.gamepad)}, ray=${(s && s.targetRayMode) || 'none'}`);
  window.xrSessionStartInfo = { granted: granted, requestedHand: true, sourcesCount: srcs.length, sources: srcDetails };
  console.log(`[XR-START] Granted: [${granted}] | Optional Requested: [local-floor, hand-tracking] | InputSources (${srcs.length}): ${srcDetails.join(' ; ') || 'none'}`);
  updateXRDiagOverlay();
}

function onXRInputSourcesChange(event) {
  const sources = getInputSources(event && event.session ? event.session : xrSession);
  if (typeof autoDetectInputMode === 'function') autoDetectInputMode(sources);
  if (typeof onXRInputSourcesChanged === 'function') onXRInputSourcesChanged(event);
  if (typeof updateXRDiagOverlay === 'function') updateXRDiagOverlay();
}

async function onXRSessionStarted(session, mode) {
  xrSession = session;
  inXRSession = true;
  vrRigInitialized = false;
  inViewMode = false;
  xrVisibilityHold = false;
  xrRequireButtonResume = false;
  logSessionStartDiag(session);
  if (typeof autoDetectInputMode === 'function') autoDetectInputMode(getInputSources(session));

  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) { btnCorner.innerText = 'EXIT VR'; btnCorner.style.background = '#d32f2f'; btnCorner.style.borderColor = '#ff5252'; }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) { btn.innerText = 'EXIT VR'; btn.style.background = '#d32f2f'; btn.style.borderColor = '#ff5252'; }

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IN SESSION';
    xrTag.className = 'tag tag-ok';
  }

  // Hide 2D HUD and desktop DOM sidebar
  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'none';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'none';

  // Hide desktop TransformControls in VR
  if (transformControlsL_trans) transformControlsL_trans.visible = false;
  if (transformControlsL_rot) transformControlsL_rot.visible = false;
  if (transformControlsR_trans) transformControlsR_trans.visible = false;
  if (transformControlsR_rot) transformControlsR_rot.visible = false;

  scene.background = new THREE.Color(0x0a0d14);

  session.addEventListener('end', onXRSessionEnded);
  session.addEventListener('visibilitychange', onXRVisibilityChange);
  session.addEventListener('inputsourceschange', onXRInputSourcesChange);

  if (renderer.xr) {
    try {
      await renderer.xr.setSession(session);
      if (typeof renderer.xr.setFoveation === 'function') renderer.xr.setFoveation(1.0);
      if (typeof renderer.xr.setFramebufferScaleFactor === 'function') renderer.xr.setFramebufferScaleFactor(0.8);
    } catch (e) { console.warn('renderer.xr.setSession warning:', e); }
  }

  try {
    xrRefSpace = await session.requestReferenceSpace('local-floor');
  } catch (e) {
    xrRefSpace = await session.requestReferenceSpace('local');
  }

  session.requestAnimationFrame(onXRFrame);
}

function onXRSessionEnded() {
  xrSession = null;
  inXRSession = false;
  xrRefSpace = null;
  vrRigInitialized = false;
  inViewMode = false;
  xrVisibilityHold = false;
  xrRequireButtonResume = false;

  const container = document.getElementById('vr-corner-container');
  if (container) container.style.display = 'flex';
  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) { btnCorner.innerText = 'ENTER VR'; btnCorner.style.display = 'block'; btnCorner.style.background = '#00ff66'; btnCorner.style.color = '#0a0e17'; btnCorner.style.borderColor = '#00ff66'; }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) { btn.innerText = 'Enter VR (Quest)'; btn.style.background = '#00c853'; btn.style.borderColor = '#00e676'; }

  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'flex';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'block';

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) { xrTag.innerText = 'XR: IDLE'; xrTag.className = 'tag tag-warn'; }

  // Restore vrRig and robotGroup for desktop
  if (vrRig) { vrRig.position.set(0, 0, 0); vrRig.quaternion.set(0, 0, 0, 1); }
  if (robotGroup) { robotGroup.position.set(0, 0, 0); robotGroup.rotation.set(0, 0, 0); }
  scene.background = new THREE.Color(0x0e111a);
  updateGizmoVisibility();
  sceneNeedsRender = true;
  requestAnimationFrame(animate);
}
