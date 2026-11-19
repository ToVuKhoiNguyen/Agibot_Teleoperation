function initGizmos() {
  const sphereGeo = new THREE.SphereGeometry(0.005, 8, 8);
  const sphereMatL = new THREE.MeshStandardMaterial({ color: 0x00e676, emissive: 0x00a844, roughness: 0.3, transparent: true, opacity: 0.0, depthTest: false, visible: false });
  targetSphereL = new THREE.Mesh(sphereGeo, sphereMatL);
  targetSphereL.position.set(0.08868, 0.35456, 0.82388);
  targetSphereL.quaternion.set(0.27299, 0.51045, 0.69221, -0.43100);  // HOME keyframe hand orientation (server re-syncs on first status)
  targetSphereL.userData = { isMarker: true, side: 'left' };
  targetSphereL.renderOrder = 900;
  (vrRig || scene).add(targetSphereL);

  labelSpriteL = createTextSprite('L', '#00c853');
  labelSpriteL.position.set(0, 0, 0.025 * HANDLE_SCALE);
  labelSpriteL.raycast = function() {};
  targetSphereL.add(labelSpriteL);

  const sphereMatR = new THREE.MeshStandardMaterial({ color: 0x2979ff, emissive: 0x1565c0, roughness: 0.3, transparent: true, opacity: 0.0, depthTest: false, visible: false });
  targetSphereR = new THREE.Mesh(sphereGeo, sphereMatR);
  targetSphereR.position.set(0.08868, -0.35456, 0.82388);
  targetSphereR.quaternion.set(0.51045, 0.27299, -0.43100, 0.69221);
  targetSphereR.userData = { isMarker: true, side: 'right' };
  targetSphereR.renderOrder = 900;
  (vrRig || scene).add(targetSphereR);

  labelSpriteR = createTextSprite('R', '#1565c0');
  labelSpriteR.position.set(0, 0, 0.025 * HANDLE_SCALE);
  labelSpriteR.raycast = function() {};
  targetSphereR.add(labelSpriteR);

  const actualGeo = new THREE.SphereGeometry(0.008, 12, 12);
  const actualMatL = new THREE.MeshBasicMaterial({ color: 0x69f0ae, wireframe: true });
  actualSphereL = new THREE.Mesh(actualGeo, actualMatL);
  (vrRig || scene).add(actualSphereL);

  const actualMatR = new THREE.MeshBasicMaterial({ color: 0x82b1ff, wireframe: true });
  actualSphereR = new THREE.Mesh(actualGeo, actualMatR);
  (vrRig || scene).add(actualSphereR);

  // Setup Combined TransformControls with unified GIZMO_SIZE
  transformControlsL_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_trans.size = GIZMO_SIZE;
  transformControlsL_trans.space = gizmoSpace;
  transformControlsL_trans.setMode('translate');
  transformControlsL_trans.attach(targetSphereL);

  transformControlsL_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_rot.size = GIZMO_SIZE;
  transformControlsL_rot.space = gizmoSpace;
  transformControlsL_rot.setMode('rotate');
  transformControlsL_rot.attach(targetSphereL);

  setupGizmoPair(transformControlsL_trans, transformControlsL_rot, targetSphereL, 'left');
  addGizmoToScene(transformControlsL_trans);
  addGizmoToScene(transformControlsL_rot);

  // Right Marker Controls
  transformControlsR_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_trans.size = GIZMO_SIZE;
  transformControlsR_trans.space = gizmoSpace;
  transformControlsR_trans.setMode('translate');
  transformControlsR_trans.attach(targetSphereR);

  transformControlsR_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_rot.size = GIZMO_SIZE;
  transformControlsR_rot.space = gizmoSpace;
  transformControlsR_rot.setMode('rotate');
  transformControlsR_rot.attach(targetSphereR);

  setupGizmoPair(transformControlsR_trans, transformControlsR_rot, targetSphereR, 'right');
  addGizmoToScene(transformControlsR_trans);
  addGizmoToScene(transformControlsR_rot);

  // Backwards-compatible aliases
  transformControlsL = transformControlsL_trans;
  transformControlsR = transformControlsR_trans;

  // Initial visibility update according to default mode ('both')
  updateGizmoVisibility();

  // Hand click & marker selection raycaster
  renderer.domElement.addEventListener('pointerdown', onCanvasPointerDown);

  // Keyboard shortcuts: W = translate, E = rotate, R = both (default), Q = toggle local/world
  window.addEventListener('keydown', onKeyDown);

  // Freeze-on-release events: pointerup, pointercancel, lostpointercapture, blur, mouseleave
  function stopActiveGizmoDrag() {
    if (isDraggingGizmo) {
      isDraggingGizmo = false;
      const armToFlush = activeGizmoArm || 'both';
      orbitControls.enabled = true;
      if (transformControlsL_trans) transformControlsL_trans.enabled = (gizmoMode === 'both' || gizmoMode === 'translate') && (showGizmoL !== false);
      if (transformControlsL_rot) transformControlsL_rot.enabled = (gizmoMode === 'both' || gizmoMode === 'rotate') && (showGizmoL !== false);
      if (transformControlsR_trans) transformControlsR_trans.enabled = (gizmoMode === 'both' || gizmoMode === 'translate') && (showGizmoR !== false);
      if (transformControlsR_rot) transformControlsR_rot.enabled = (gizmoMode === 'both' || gizmoMode === 'rotate') && (showGizmoR !== false);
      flushTargetUpdates(armToFlush);
    }
  }
  const domEl = renderer.domElement;
  domEl.addEventListener('pointerup', stopActiveGizmoDrag);
  domEl.addEventListener('pointercancel', stopActiveGizmoDrag);
  domEl.addEventListener('lostpointercapture', stopActiveGizmoDrag);
  domEl.addEventListener('mouseleave', stopActiveGizmoDrag);
  window.addEventListener('blur', stopActiveGizmoDrag);
  window.stopActiveGizmoDrag = stopActiveGizmoDrag;

  window.addEventListener('resize', onWindowResize);
  animate();
}



function setupGizmoPair(transCtrl, rotCtrl, targetObj, arm) {
  configureGizmo(transCtrl);
  configureGizmo(rotCtrl);

  renderer.domElement.addEventListener('pointerdown', function(e) {
    if (transCtrl.axis && rotCtrl.axis) {
      if ((transCtrl.intersectDistance || 0) <= (rotCtrl.intersectDistance || 0)) {
        rotCtrl.axis = null;
      } else {
        transCtrl.axis = null;
      }
    }
  }, true);

  transCtrl.addEventListener('dragging-changed', function(e) {
    if (e.value && typeof cancelGizmoRecall === 'function') cancelGizmoRecall(arm);
    orbitControls.enabled = !e.value;
    rotCtrl.enabled = !e.value;
    isDraggingGizmo = e.value;
    if (e.value) activeGizmoArm = arm;
    if (!e.value) flushTargetUpdates(arm);
  });

  rotCtrl.addEventListener('dragging-changed', function(e) {
    if (e.value && typeof cancelGizmoRecall === 'function') cancelGizmoRecall(arm);
    orbitControls.enabled = !e.value;
    transCtrl.enabled = !e.value;
    isDraggingGizmo = e.value;
    if (e.value) activeGizmoArm = arm;
    if (!e.value) flushTargetUpdates(arm);
  });

  transCtrl.addEventListener('objectChange', function() {
    queueTargetUpdate(arm, targetObj.position, targetObj.quaternion);
  });

  rotCtrl.addEventListener('objectChange', function() {
    queueTargetUpdate(arm, targetObj.position, targetObj.quaternion);
  });
}

function configureGizmo(ctrl) {
  ctrl.renderOrder = 950;
  ctrl.traverse(function(child) {
    if (child.material) {
      child.material.depthTest = false;
      child.material.depthWrite = false;
      child.material.transparent = true;
      child.renderOrder = 950;
    }
  });
}

function addGizmoToScene(ctrl) {
  if (typeof ctrl.getHelper === 'function') scene.add(ctrl.getHelper());
  scene.add(ctrl);
}

function onKeyDown(event) {
  if (event.key === 'Escape') {
    if (typeof stopActiveGizmoDrag === 'function') stopActiveGizmoDrag();
    return;
  }
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) return;
  const key = event.key.toUpperCase();
  if (key === 'W') setGizmoMode('translate');
  else if (key === 'E') setGizmoMode('rotate');
  else if (key === 'R') {
    setGizmoMode('both');
    if (typeof recenterHandAction === 'function') recenterHandAction();
  } else if (key === 'Q') toggleGizmoSpace();
  else if (key === '1') toggleActiveGizmo('left');      // show / hide left marker (was the "Left Marker" button)
  else if (key === '2') toggleActiveGizmo('right');
  else if (key === 'F') resetCameraView();             // was the "Reset Cam" button
  else if (key === 'C') {
    if (typeof calibratePoseAction === 'function') calibratePoseAction();
  } else if (key === 'T') {
    if (typeof startHandSelfTest === 'function') startHandSelfTest();
  } else if (key === 'L') {
    if (typeof recordHandDataAction === 'function') recordHandDataAction();
  }
}

function resetCameraView() {
  camera.position.set(1.8, -1.8, 1.4);
  orbitControls.target.set(0.2, 0.0, 1.0);
  orbitControls.update();
}

function setGizmoMode(mode) {
  gizmoMode = mode; // 'both' | 'translate' | 'rotate'
  updateGizmoVisibility();
  const bBoth = document.getElementById('gizmo-mode-both');
  const bTrans = document.getElementById('gizmo-mode-trans');
  const bRot = document.getElementById('gizmo-mode-rot');
  if (bBoth) bBoth.classList.toggle('active', mode === 'both');
  if (bTrans) bTrans.classList.toggle('active', mode === 'translate');
  if (bRot) bRot.classList.toggle('active', mode === 'rotate');
}

function updateGizmoVisibility() {
  const showTrans = (gizmoMode === 'both' || gizmoMode === 'translate');
  const showRot = (gizmoMode === 'both' || gizmoMode === 'rotate');
  const visL = (showGizmoL !== false);
  const visR = (showGizmoR !== false);

  if (transformControlsL_trans) {
    transformControlsL_trans.visible = transformControlsL_trans.enabled = showTrans && visL;
  }
  if (transformControlsL_rot) {
    transformControlsL_rot.visible = transformControlsL_rot.enabled = showRot && visL;
  }
  if (transformControlsR_trans) {
    transformControlsR_trans.visible = transformControlsR_trans.enabled = showTrans && visR;
  }
  if (transformControlsR_rot) {
    transformControlsR_rot.visible = transformControlsR_rot.enabled = showRot && visR;
  }
}

function toggleGizmoSpace() {
  gizmoSpace = (gizmoSpace === 'world') ? 'local' : 'world';
  if (transformControlsL_trans) transformControlsL_trans.setSpace(gizmoSpace);
  if (transformControlsL_rot) transformControlsL_rot.setSpace(gizmoSpace);
  if (transformControlsR_trans) transformControlsR_trans.setSpace(gizmoSpace);
  if (transformControlsR_rot) transformControlsR_rot.setSpace(gizmoSpace);
  const btn = document.getElementById('gizmo-space-btn');
  if (btn) {
    btn.innerText = `Space: ${gizmoSpace.charAt(0).toUpperCase() + gizmoSpace.slice(1)} (Q)`;
    btn.classList.toggle('active', gizmoSpace === 'local');
  }
  sceneNeedsRender = true;
}

function adjustGizmoSize(delta) {
  GIZMO_SIZE = Math.max(0.15, Math.min(1.2, GIZMO_SIZE + delta));
  if (transformControlsL_trans) transformControlsL_trans.size = GIZMO_SIZE;
  if (transformControlsL_rot) transformControlsL_rot.size = GIZMO_SIZE;
  if (transformControlsR_trans) transformControlsR_trans.size = GIZMO_SIZE;
  if (transformControlsR_rot) transformControlsR_rot.size = GIZMO_SIZE;
  sceneNeedsRender = true;
}
