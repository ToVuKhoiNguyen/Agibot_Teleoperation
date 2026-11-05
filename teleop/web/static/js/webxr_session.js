// ==========================================
// WebXR Immersive-VR Session Management
// ==========================================
function checkXRSupport() {
  const btn = document.getElementById('btn-vr-corner');
  if (!btn) return;
  btn.style.display = 'none';
  if (!window.isSecureContext || !navigator.xr) return;

  navigator.xr.isSessionSupported('immersive-vr').then(supported => {
    if (supported) {
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

  // Synchronous requestSession: no await/fetch before it to maintain user gesture
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

async function onXRSessionStarted(session, mode) {
  xrSession = session;
  inXRSession = true;
  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) {
    btnCorner.innerText = 'Exit VR';
    btnCorner.style.background = '#d32f2f';
    btnCorner.style.borderColor = '#ff5252';
  }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Exit VR';
    btn.style.background = '#d32f2f';
    btn.style.borderColor = '#ff5252';
  }

  // Update session tag
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

  // Hide desktop TransformControls in VR (markers stay visible)
  if (transformControlsL_trans) transformControlsL_trans.visible = false;
  if (transformControlsL_rot) transformControlsL_rot.visible = false;
  if (transformControlsR_trans) transformControlsR_trans.visible = false;
  if (transformControlsR_rot) transformControlsR_rot.visible = false;

  // Background handling: dark room
  scene.background = new THREE.Color(0x0a0d14);

  // Position robotGroup in front of user at floor level: ~0.8m forward in -Z, feet at Y=0
  robotGroup.position.set(0, 0, -0.8);
  robotGroup.rotation.set(-Math.PI / 2, 0, 0); // Convert MuJoCo Z-up to Three.js WebXR Y-up

  // Session end listener
  session.addEventListener('end', onXRSessionEnded);

  // Setup WebXR rendering context with foveation & frame scaling
  if (renderer.xr) {
    await renderer.xr.setSession(session);
    if (typeof renderer.xr.setFoveation === 'function') {
      renderer.xr.setFoveation(1.0);
    }
    if (typeof renderer.xr.setFramebufferScaleFactor === 'function') {
      renderer.xr.setFramebufferScaleFactor(0.8);
    }
  }

  // Reference space: local-floor preferred, fallback local
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

  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) {
    btnCorner.innerText = 'VR';
    btnCorner.style.background = '#00c853';
    btnCorner.style.borderColor = '#00e676';
  }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Enter VR (Quest)';
    btn.style.background = '#00c853';
    btn.style.borderColor = '#00e676';
  }

  // Restore DOM elements
  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'flex';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'block';

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IDLE';
    xrTag.className = 'tag tag-warn';
  }

  // Restore robot group pose for desktop view
  robotGroup.position.set(0, 0, 0);
  robotGroup.rotation.set(0, 0, 0);
  scene.background = new THREE.Color(0x0e111a);
  updateGizmoVisibility();
  sceneNeedsRender = true;
  requestAnimationFrame(animate);
}
