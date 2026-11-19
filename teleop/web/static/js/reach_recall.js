// ==========================================
// REACH RECALL & GHOST WORKSPACE VISUALIZER
// ==========================================
let ghostSphereL = null, ghostSphereR = null;
let ghostLineL = null, ghostLineR = null;
let activeRecallAnimation = null; // { arm, startPos, targetPos, startTime, duration }

const SHOULDER_L = new THREE.Vector3(-0.03779, 0.19100, 1.27210);
const SHOULDER_R = new THREE.Vector3(-0.03779, -0.19100, 1.27210);
const MAX_REACH_M = 0.500;

function initReachRecallVisuals() {
  if (ghostSphereL) return;
  const ghostGeo = new THREE.SphereGeometry(0.012, 16, 16);
  const ghostMat = new THREE.MeshBasicMaterial({
    color: 0xffd600,
    transparent: true,
    opacity: 0.6,
    wireframe: true,
    depthTest: false
  });

  ghostSphereL = new THREE.Mesh(ghostGeo, ghostMat);
  ghostSphereL.visible = false;
  ghostSphereL.renderOrder = 910;
  (vrRig || scene).add(ghostSphereL);

  ghostSphereR = new THREE.Mesh(ghostGeo, ghostMat.clone());
  ghostSphereR.visible = false;
  ghostSphereR.renderOrder = 910;
  (vrRig || scene).add(ghostSphereR);

  const lineMat = new THREE.LineDashedMaterial({
    color: 0xffd600,
    dashSize: 0.015,
    gapSize: 0.01,
    transparent: true,
    opacity: 0.8,
    depthTest: false
  });

  const lineGeoL = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  ghostLineL = new THREE.Line(lineGeoL, lineMat);
  ghostLineL.visible = false;
  ghostLineL.renderOrder = 905;
  (vrRig || scene).add(ghostLineL);

  const lineGeoR = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  ghostLineR = new THREE.Line(lineGeoR, lineMat.clone());
  ghostLineR.visible = false;
  ghostLineR.renderOrder = 905;
  (vrRig || scene).add(ghostLineR);
}

function projectPointToReach(targetPos, shoulderPos, maxReach) {
  const v = new THREE.Vector3().subVectors(targetPos, shoulderPos);
  const dist = v.length();
  if (dist > maxReach && dist > 1e-6) {
    const clamped = new THREE.Vector3().copy(shoulderPos).addScaledVector(v, maxReach / dist);
    return { clamped: clamped, isClamped: true, beyond: dist - maxReach };
  }
  return { clamped: targetPos.clone(), isClamped: false, beyond: 0.0 };
}

function updateGhostReachVisuals() {
  if (!ghostSphereL) initReachRecallVisuals();

  // Left arm check
  if (targetSphereL && isDraggingGizmo && activeGizmoArm === 'left') {
    const res = projectPointToReach(targetSphereL.position, SHOULDER_L, MAX_REACH_M);
    if (res.isClamped) {
      ghostSphereL.position.copy(res.clamped);
      ghostSphereL.visible = true;
      const pts = [targetSphereL.position.clone(), res.clamped.clone()];
      ghostLineL.geometry.setFromPoints(pts);
      ghostLineL.computeLineDistances();
      ghostLineL.visible = true;
    } else {
      ghostSphereL.visible = false;
      ghostLineL.visible = false;
    }
  } else {
    if (ghostSphereL) ghostSphereL.visible = false;
    if (ghostLineL) ghostLineL.visible = false;
  }

  // Right arm check
  if (targetSphereR && isDraggingGizmo && activeGizmoArm === 'right') {
    const res = projectPointToReach(targetSphereR.position, SHOULDER_R, MAX_REACH_M);
    if (res.isClamped) {
      ghostSphereR.position.copy(res.clamped);
      ghostSphereR.visible = true;
      const pts = [targetSphereR.position.clone(), res.clamped.clone()];
      ghostLineR.geometry.setFromPoints(pts);
      ghostLineR.computeLineDistances();
      ghostLineR.visible = true;
    } else {
      ghostSphereR.visible = false;
      ghostLineR.visible = false;
    }
  } else {
    if (ghostSphereR) ghostSphereR.visible = false;
    if (ghostLineR) ghostLineR.visible = false;
  }
}

function handleGizmoRecall(data) {
  const arm = data.arm || 'left';
  const targetObj = (arm === 'left') ? targetSphereL : targetSphereR;
  if (!targetObj || !data.pos) return;
  if (isDraggingGizmo && activeGizmoArm === arm) return;  // never pull the gizmo out from under the mouse
  if (data.rot && Array.isArray(data.rot) && data.rot.length === 4) {
    targetObj.quaternion.set(data.rot[0], data.rot[1], data.rot[2], data.rot[3]);
  }

  const targetPos = new THREE.Vector3(data.pos[0], data.pos[1], data.pos[2]);
  activeRecallAnimation = {
    arm: arm,
    targetObj: targetObj,
    startPos: targetObj.position.clone(),
    targetPos: targetPos,
    startTime: performance.now(),
    duration: data.duration_ms || 250
  };
  if (typeof sceneNeedsRender !== 'undefined') sceneNeedsRender = true;
}

function updateGizmoRecallAnimation(currentTime) {
  if (!activeRecallAnimation) return;
  const anim = activeRecallAnimation;
  const elapsed = currentTime - anim.startTime;
  const progress = Math.min(1.0, Math.max(0.0, elapsed / anim.duration));
  // Ease-out cubic: 1 - (1 - t)^3
  const easeOut = 1.0 - Math.pow(1.0 - progress, 3);

  anim.targetObj.position.lerpVectors(anim.startPos, anim.targetPos, easeOut);
  if (typeof sceneNeedsRender !== 'undefined') sceneNeedsRender = true;

  if (progress >= 1.0) {
    anim.targetObj.position.copy(anim.targetPos);
    activeRecallAnimation = null;
  }
}

function isGizmoRecalling() {
  return activeRecallAnimation !== null;
}

function cancelGizmoRecall(arm) {
  if (activeRecallAnimation && (!arm || activeRecallAnimation.arm === arm)) {
    activeRecallAnimation = null;
  }
}

function handleGizmoSet(data) {
  const arm = data.arm || 'left';
  const targetObj = (arm === 'left') ? targetSphereL : targetSphereR;
  if (!targetObj) return;
  if (data.pos && Array.isArray(data.pos)) {
    targetObj.position.set(data.pos[0], data.pos[1], data.pos[2]);
  }
  if (data.rot && Array.isArray(data.rot) && data.rot.length === 4) {
    targetObj.quaternion.set(data.rot[0], data.rot[1], data.rot[2], data.rot[3]);
  }
  if (typeof sceneNeedsRender !== 'undefined') sceneNeedsRender = true;
}

window.handleGizmoRecall = handleGizmoRecall;
window.handleGizmoSet = handleGizmoSet;
window.updateGhostReachVisuals = updateGhostReachVisuals;
window.updateGizmoRecallAnimation = updateGizmoRecallAnimation;
window.isGizmoRecalling = isGizmoRecalling;
window.cancelGizmoRecall = cancelGizmoRecall;
