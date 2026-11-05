// ==========================================
// WebXR Controller Tracking, VR Viewpoint & View Mode
// ==========================================
let lastXRFrameTime = 0;
let vrRigInitialized = false;
let headYaw = 0.0;
let rigYawOffset = 0.0;
let rigDist = 1.2;
let rigHeightOffset = 0.0;
let currentRigYaw = 0.0;
let inViewMode = false;
let bothClickStart = 0;
let bothClickTriggered = false;

// HUD 3D Sprites
let viewModeSprite = null;
let trackingLostSprite = null;

// Tracking Loss State per hand with 150ms debounce
const trackingState = {
  left: { lost: false, lossStart: 0, lastPosStr: '', lastChangeTime: 0 },
  right: { lost: false, lossStart: 0, lastPosStr: '', lastChangeTime: 0 }
};

function initVRViewpoint(viewerPose) {
  const p = viewerPose.transform.position;
  const q = viewerPose.transform.orientation;
  const siny_cosp = 2.0 * (q.w * q.y + q.x * q.z);
  const cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  headYaw = Math.atan2(siny_cosp, cosy_cosp);

  const viewParam = (new URLSearchParams(window.location.search).get('view') || 'front').toLowerCase();
  let viewOffset = 0.0;
  if (viewParam === 'side') viewOffset = Math.PI / 2.0;
  else if (viewParam === 'back') viewOffset = Math.PI;

  rigYawOffset = viewOffset;
  rigDist = 1.2;
  rigHeightOffset = 0.0;
  updateVRRigTransform(p.x, p.y, p.z);
  vrRigInitialized = true;
}

function updateVRRigTransform(hx, hy, hz) {
  if (!vrRig) return;
  const totalYaw = headYaw + rigYawOffset;
  currentRigYaw = totalYaw;
  const lookX = -Math.sin(headYaw);
  const lookZ = -Math.cos(headYaw);

  const torsoX = hx + rigDist * lookX;
  const torsoY = hy + rigHeightOffset;
  const torsoZ = hz + rigDist * lookZ;

  // Base rotation: MuJoCo Z up -> Y up, MuJoCo X fwd -> +Z (facing user)
  const baseMat = new THREE.Matrix4().set(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1);
  const baseQuat = new THREE.Quaternion().setFromRotationMatrix(baseMat);
  const rotY = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), totalYaw);
  vrRig.quaternion.copy(rotY).multiply(baseQuat);

  const localTorso = new THREE.Vector3(0, 0, 1.1).applyQuaternion(vrRig.quaternion);
  vrRig.position.set(torsoX - localTorso.x, torsoY - localTorso.y, torsoZ - localTorso.z);
}

function updateSceneHUD(msgView, msgLoss, headPos) {
  if (!scene) return;
  if (!viewModeSprite) {
    viewModeSprite = createTextSprite('VIEW MODE', '#ffb300');
    viewModeSprite.visible = false;
    scene.add(viewModeSprite);
  }
  if (!trackingLostSprite) {
    trackingLostSprite = createTextSprite('TRACKING LOST', '#d32f2f');
    trackingLostSprite.visible = false;
    scene.add(trackingLostSprite);
  }

  if (headPos) {
    const fwdX = -Math.sin(headYaw), fwdZ = -Math.cos(headYaw);
    viewModeSprite.position.set(headPos.x + 0.8 * fwdX, headPos.y + 0.25, headPos.z + 0.8 * fwdZ);
    trackingLostSprite.position.set(headPos.x + 0.8 * fwdX, headPos.y - 0.25, headPos.z + 0.8 * fwdZ);
  }
  viewModeSprite.visible = Boolean(msgView);
  trackingLostSprite.visible = Boolean(msgLoss);
}

function updateTrackingStatus(side, src, pose, nowMs) {
  const st = trackingState[side];
  let isLoss = false;
  if (!src || !pose || !pose.transform) {
    isLoss = true;
  } else {
    const p = pose.transform.position;
    const posStr = `${p.x.toFixed(3)},${p.y.toFixed(3)},${p.z.toFixed(3)}`;
    if (posStr !== st.lastPosStr) {
      st.lastPosStr = posStr;
      st.lastChangeTime = nowMs;
    } else if (st.lastChangeTime > 0 && nowMs - st.lastChangeTime > 1000) {
      isLoss = true; // frozen > 1s
    }
  }

  if (isLoss) {
    if (!st.lossStart) st.lossStart = nowMs;
    if (nowMs - st.lossStart >= 150) st.lost = true; // 150ms debounce
  } else {
    st.lossStart = 0;
    st.lost = false;
  }
  return !st.lost;
}

function onXRFrame(time, frame) {
  if (!xrSession) return;
  xrSession.requestAnimationFrame(onXRFrame);

  const nowMs = performance.now();
  const dtSec = lastXRFrameTime > 0 ? Math.min(0.05, Math.max(0.001, (nowMs - lastXRFrameTime) / 1000.0)) : 0.016;
  lastXRFrameTime = nowMs;

  if (typeof applyLatestMovingPoses === 'function') applyLatestMovingPoses(dtSec);
  renderer.render(scene, camera);
  if (!xrRefSpace) return;

  const viewerPose = frame.getViewerPose(xrRefSpace);
  let headPos = null;
  if (viewerPose) {
    headPos = viewerPose.transform.position;
    if (!vrRigInitialized) initVRViewpoint(viewerPose);
  }

  const inputSources = xrSession.inputSources || [];
  let leftSrc = null, rightSrc = null;
  for (let i = 0; i < inputSources.length; i++) {
    if (inputSources[i].handedness === 'left') leftSrc = inputSources[i];
    else if (inputSources[i].handedness === 'right') rightSrc = inputSources[i];
  }

  const parseHand = (src, side) => {
    if (!src || !src.gamepad) {
      updateTrackingStatus(side, null, null, nowMs);
      return { pos: null, quat: null, tracking: false, tracking_loss: true, grip: false, trigger: 0, thumbstick: [0, 0] };
    }
    const space = src.gripSpace || src.targetRaySpace;
    const pose = space ? frame.getPose(space, xrRefSpace) : null;
    const isTracked = updateTrackingStatus(side, src, pose, nowMs);

    let posArr = null, quatArr = null;
    if (isTracked && pose && pose.transform) {
      const p = pose.transform.position, q = pose.transform.orientation;
      posArr = [p.x, p.y, p.z];
      quatArr = [q.w, q.x, q.y, q.z];
    }

    const gp = src.gamepad;
    const btns = gp.buttons || [];
    const axes = gp.axes || [0, 0];
    const trig = btns.length > 0 ? (typeof btns[0].value === 'number' ? btns[0].value : (btns[0].pressed ? 1.0 : 0.0)) : 0.0;
    const grip = btns.length > 1 ? (typeof btns[1].value === 'number' ? btns[1].value : (btns[1].pressed ? 1.0 : 0.0)) : 0.0;
    const thumbClick = btns.length > 3 ? Boolean(btns[3].pressed || (btns[3].value > 0.5)) : false;
    const btnPri = btns.length > 4 ? Boolean(btns[4].pressed || (btns[4].value > 0.5)) : false;
    const btnSec = btns.length > 5 ? Boolean(btns[5].pressed || (btns[5].value > 0.5)) : false;

    let sx = axes[0] || 0.0, sy = axes[1] || 0.0;
    if (Math.abs(sx) < 0.05 && Math.abs(sy) < 0.05 && axes.length >= 4) {
      if (Math.abs(axes[2]) > 0.05 || Math.abs(axes[3]) > 0.05) { sx = axes[2]; sy = axes[3]; }
    }

    return {
      pos: posArr, quat: quatArr, tracking: isTracked, tracking_loss: !isTracked,
      grip: grip >= 0.25, grip_val: grip, trigger: trig, thumbstick: [sx, sy],
      thumbstick_click: thumbClick,
      buttons: { primary: btnPri, secondary: btnSec },
      btn_x: side === 'left' ? btnPri : false, btn_y: side === 'left' ? btnSec : false,
      btn_a: side === 'right' ? btnPri : false, btn_b: side === 'right' ? btnSec : false
    };
  };

  const leftPkt = parseHand(leftSrc, 'left');
  const rightPkt = parseHand(rightSrc, 'right');

  // Resume from visibility hold if user presses any button
  if (xrRequireButtonResume) {
    if (leftPkt.trigger > 0.3 || leftPkt.grip_val > 0.3 || rightPkt.trigger > 0.3 || rightPkt.grip_val > 0.3 || leftPkt.buttons.primary || rightPkt.buttons.primary) {
      xrRequireButtonResume = false;
      xrVisibilityHold = false;
    }
  }

  // Toggle View Mode: both thumbstick clicks held for 0.5s
  if (leftPkt.thumbstick_click && rightPkt.thumbstick_click) {
    if (!bothClickStart) bothClickStart = nowMs;
    else if (nowMs - bothClickStart >= 500 && !bothClickTriggered) {
      inViewMode = !inViewMode;
      bothClickTriggered = true;
    }
  } else {
    bothClickStart = 0;
    bothClickTriggered = false;
  }

  // View Mode Navigation
  if (inViewMode && headPos) {
    const rx = rightPkt.thumbstick[0], ry = rightPkt.thumbstick[1], ly = leftPkt.thumbstick[1];
    if (Math.abs(rx) > 0.15) rigYawOffset += rx * 1.5 * dtSec;
    if (Math.abs(ry) > 0.15) rigDist = Math.max(0.6, Math.min(3.5, rigDist - ry * 1.5 * dtSec));
    if (Math.abs(ly) > 0.15) rigHeightOffset += -ly * 1.0 * dtSec;
    if (rightPkt.btn_a) { rigYawOffset = 0.0; rigDist = 1.2; rigHeightOffset = 0.0; } // Recenter
    updateVRRigTransform(headPos.x, headPos.y, headPos.z);
  }

  // Update In-Scene HUD Warnings
  const anyLoss = !leftPkt.tracking || !rightPkt.tracking;
  updateSceneHUD(inViewMode, anyLoss, headPos);

  // Controller Status DOM update
  const ctrlStatus = document.getElementById('ctrl-status-text');
  if (ctrlStatus) {
    if (anyLoss) ctrlStatus.innerText = (!leftPkt.tracking && !rightPkt.tracking) ? 'L/R TRACKING LOST' : (!leftPkt.tracking ? 'L TRACKING LOST' : 'R TRACKING LOST');
    else if (inViewMode) ctrlStatus.innerText = 'VIEW MODE ACTIVE';
  }

  // Teleoperation Command Transmission
  if (!inViewMode && !xrVisibilityHold) {
    sendCmd({
      cmd: 'xr_controllers',
      t: performance.now() / 1000.0,
      rotation_enabled: xrRotationEnabled,
      rig_yaw: currentRigYaw,
      left: leftPkt,
      right: rightPkt
    });
  } else {
    // When in view mode or visibility hold: lock clutches & freeze arm targets
    sendCmd({
      cmd: 'xr_controllers',
      t: performance.now() / 1000.0,
      rotation_enabled: xrRotationEnabled,
      rig_yaw: currentRigYaw,
      left: { pos: null, quat: null, tracking: false, tracking_loss: true, grip: false, trigger: 0, thumbstick: [0, 0] },
      right: { pos: null, quat: null, tracking: false, tracking_loss: true, grip: false, trigger: 0, thumbstick: [0, 0] }
    });
  }
}
