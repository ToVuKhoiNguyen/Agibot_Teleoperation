// ==========================================
// WebXR Immersive-VR Session Management
// ==========================================
let xrVisibilityHold = false;
let xrRequireButtonResume = false;

function checkXRSupport() {
  const container = document.getElementById('vr-corner-container');
  const btn = document.getElementById('btn-vr-corner');
  if (!btn) return;
  if (container) container.style.display = 'none';
  else btn.style.display = 'none';
  if (!window.isSecureContext || !navigator.xr) return;

  navigator.xr.isSessionSupported('immersive-vr').then(supported => {
    if (supported) {
      if (container) container.style.display = 'flex';
      btn.style.display = 'block';
    }
  }).catch(err => {
    console.warn('isSessionSupported(immersive-vr) error:', err);
  });
}

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

function onXRInputSourcesChange(event) {
  if (typeof onXRInputSourcesChanged === 'function') {
    onXRInputSourcesChanged(event);
  }
}

async function onXRSessionStarted(session, mode) {
  xrSession = session;
  inXRSession = true;
  vrRigInitialized = false;
  inViewMode = false;
  xrVisibilityHold = false;
  xrRequireButtonResume = false;

  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) {
    btnCorner.innerText = 'EXIT VR';
    btnCorner.style.background = '#d32f2f';
    btnCorner.style.color = '#ffffff';
    btnCorner.style.borderColor = '#ff5252';
  }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'EXIT VR';
    btn.style.background = '#d32f2f';
    btn.style.borderColor = '#ff5252';
  }

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
    await renderer.xr.setSession(session);
    if (typeof renderer.xr.setFoveation === 'function') renderer.xr.setFoveation(1.0);
    if (typeof renderer.xr.setFramebufferScaleFactor === 'function') renderer.xr.setFramebufferScaleFactor(0.8);
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
  if (btnCorner) {
    btnCorner.innerText = 'ENTER VR';
    btnCorner.style.display = 'block';
    btnCorner.style.background = '#00ff66';
    btnCorner.style.color = '#0a0e17';
    btnCorner.style.borderColor = '#00ff66';
  }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Enter VR (Quest)';
    btn.style.background = '#00c853';
    btn.style.borderColor = '#00e676';
  }

  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'flex';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'block';

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IDLE';
    xrTag.className = 'tag tag-warn';
  }

  // Restore vrRig and robotGroup for desktop
  if (vrRig) {
    vrRig.position.set(0, 0, 0);
    vrRig.quaternion.set(0, 0, 0, 1);
  }
  if (robotGroup) {
    robotGroup.position.set(0, 0, 0);
    robotGroup.rotation.set(0, 0, 0);
  }
  scene.background = new THREE.Color(0x0e111a);
  updateGizmoVisibility();
  sceneNeedsRender = true;
  requestAnimationFrame(animate);
}
