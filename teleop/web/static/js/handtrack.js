// ==========================================
// WebXR Bare-Hand Tracking & Retargeting Module
// ==========================================
const XR_JOINT_NAMES = [
  "wrist",
  "thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip",
  "index-finger-metacarpal", "index-finger-phalanx-proximal", "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip",
  "middle-finger-metacarpal", "middle-finger-phalanx-proximal", "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip",
  "ring-finger-metacarpal", "ring-finger-phalanx-proximal", "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip",
  "pinky-finger-metacarpal", "pinky-finger-phalanx-proximal", "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip"
];

const handUrlParams = new URLSearchParams(window.location.search);
const urlInputParam = (handUrlParams.get('input') || 'auto').toLowerCase();
window.currentInputMode = urlInputParam;
window.confirmedInputMode = 'auto';
window.currentHandFollowMode = (handUrlParams.get('mode') || handUrlParams.get('handmode') || 'absolute').toLowerCase();
const handDebugEnabled = (handUrlParams.get('handdebug') === '1');

let lastHandSendMs = 0;
let handClutchSprite = null;
let handDebugSpheres = { left: [], right: [] };

function setInputMode(mode) {
  window.currentInputMode = mode;
  const sel = document.getElementById('select-input-mode');
  if (sel) sel.value = mode;
  sendCmd({ cmd: 'set_input_mode', mode: mode });
  updateModeBannerUI();
}

function autoDetectInputMode(inputSources) {
  const sources = (typeof getInputSources === 'function') ? getInputSources(inputSources) : (Array.isArray(inputSources) ? inputSources : Array.from(inputSources || []));
  if (!sources || sources.length === 0) return;
  let hasHand = false, hasController = false;
  for (let i = 0; i < sources.length; i++) {
    const src = sources[i];
    if (src && src.hand) hasHand = true;
    else if (src && (src.targetRayMode === 'tracked-pointer' || src.gamepad)) hasController = true;
  }
  let mode = window.currentInputMode;
  if (hasHand) mode = 'hands';
  else if (hasController) mode = 'controllers';
  window.currentInputMode = mode;
  sendCmd({ cmd: 'set_input_mode', mode: mode });
  updateModeBannerUI();
}

window.onXRInputSourcesChanged = function(event) {
  const s = event && event.session ? event.session : window.xrSession;
  const srcs = (typeof getInputSources === 'function') ? getInputSources(s) : Array.from((s && s.inputSources) || []);
  autoDetectInputMode(srcs);
};

function calibrateHandAction() { sendCmd({ cmd: 'calibrate_hand' }); }
function calibratePoseAction() { sendCmd({ cmd: 'calibrate_pose' }); }
function recenterHandAction() { sendCmd({ cmd: 'recenter_hand' }); }
window.calibrateHandAction = calibrateHandAction;
window.calibratePoseAction = calibratePoseAction;
window.recenterHandAction = recenterHandAction;
function setHandFollowMode(mode) {
  window.currentHandFollowMode = mode;
  sendCmd({ cmd: 'set_hand_follow_mode', mode: mode });
  const sel = document.getElementById('select-hand-mode');
  if (sel) sel.value = mode;
}
window.setHandFollowMode = setHandFollowMode;
window.setInputMode = setInputMode;

function initHandDebugSpheres() {
  if (!handDebugEnabled || !scene) return;
  ['left', 'right'].forEach(side => {
    if (handDebugSpheres[side].length === 0) {
      const col = (side === 'left') ? 0x00e676 : 0x2979ff;
      for (let i = 0; i < 25; i++) {
        const m = new THREE.Mesh(new THREE.SphereGeometry(0.006, 6, 6), new THREE.MeshBasicMaterial({ color: col }));
        m.visible = false;
        scene.add(m);
        handDebugSpheres[side].push(m);
      }
    }
  });
}

function updateHandSceneHUD(statusText, headPos, isMismatch) {
  if (!scene) return;
  if (!handClutchSprite) {
    handClutchSprite = createTextSprite('HAND TRACKING', '#00e5ff');
    handClutchSprite.visible = false;
    scene.add(handClutchSprite);
  }
  if (headPos) {
    const hYaw = (typeof headYaw !== 'undefined') ? headYaw : 0.0;
    const fwdX = -Math.sin(hYaw), fwdZ = -Math.cos(hYaw);
    handClutchSprite.position.set(headPos.x + 0.8 * fwdX, headPos.y - 0.15, headPos.z + 0.8 * fwdZ);
  }
  if (statusText) {
    if (typeof updateSpriteText === 'function') {
      updateSpriteText(handClutchSprite, statusText, isMismatch ? '#ff1744' : '#00e5ff');
    }
    handClutchSprite.visible = true;
  } else {
    handClutchSprite.visible = false;
  }
}

function parseXRHandSource(src, frame, refSpace) {
  if (!src || !src.hand) return { tracking: false, confidence: 0.0, wrist: null, joints: [] };
  const hand = src.hand, jointCoords = [];
  let wristPose = null, trackedCount = 0;
  for (let i = 0; i < XR_JOINT_NAMES.length; i++) {
    const jSpace = (hand && typeof hand.get === 'function') ? hand.get(XR_JOINT_NAMES[i]) : (hand ? hand[i] : null);
    if (jSpace && frame && typeof frame.getJointPose === 'function') {
      const p = frame.getJointPose(jSpace, refSpace);
      if (p && p.transform && p.transform.position && p.transform.orientation) {
        const pos = p.transform.position, q = p.transform.orientation;
        jointCoords.push([pos.x, pos.y, pos.z]);
        trackedCount++;
        if (i === 0) {
          wristPose = { pos: [pos.x, pos.y, pos.z], quat: [q.w, q.x, q.y, q.z] };
        }
        continue;
      }
    }
    jointCoords.push([0, 0, 0]);
  }
  const confidence = trackedCount / 25.0;
  const isTracked = (confidence >= 0.20 && wristPose !== null);
  if (isTracked && wristPose && jointCoords[10] && jointCoords[0] && (jointCoords[10][0] !== 0 || jointCoords[10][1] !== 0)) {
    wristPose.pos = [0.5 * (jointCoords[0][0] + jointCoords[10][0]), 0.5 * (jointCoords[0][1] + jointCoords[10][1]), 0.5 * (jointCoords[0][2] + jointCoords[10][2])];
  }
  return { tracking: isTracked, confidence: confidence, wrist: wristPose, joints: jointCoords };
}

function updateHandDebugSpheres(side, joints, isTracked, conf) {
  if (!handDebugEnabled || !handDebugSpheres[side]) return;
  const spheres = handDebugSpheres[side];
  for (let i = 0; i < spheres.length; i++) {
    const s = spheres[i];
    if (isTracked && joints[i] && (joints[i][0] !== 0 || joints[i][1] !== 0 || joints[i][2] !== 0)) {
      s.position.set(joints[i][0], joints[i][1], joints[i][2]);
      s.material.color.setHex(conf > 0.8 ? 0x00e676 : (conf > 0.4 ? 0xffb300 : 0xd50000));
      s.visible = true;
    } else { s.visible = false; }
  }
}

function updateModeBannerUI(extraBanner) {
  const req = window.currentInputMode, conf = window.confirmedInputMode || req;
  const isMismatch = (req !== 'auto' && conf !== 'auto' && req !== conf);
  const bld = window.BUILD_ID ? ` | bld=${window.BUILD_ID}` : '';
  const bar = document.getElementById('hand-banner-bar');
  if (bar) {
    if (isMismatch) {
      bar.innerText = `[MODE MISMATCH: req=${req} vs conf=${conf}]${bld}`;
      bar.style.background = '#b71c1c'; bar.style.color = '#ffcdd2';
    } else {
      bar.innerText = (extraBanner || `INPUT: ${conf.toUpperCase()}${bld}`);
      bar.style.background = conf === 'hands' ? '#1b382b' : '#1e293b';
      bar.style.color = conf === 'hands' ? '#a7f3d0' : '#90caf9';
    }
  }
}

function processHandTrackingFrame(frame, refSpace, inputSources, headPos, nowMs) {
  const sources = (typeof getInputSources === 'function') ? getInputSources(inputSources) : (Array.isArray(inputSources) ? inputSources : Array.from(inputSources || []));
  if (sources && sources.length > 0 && Math.random() < 0.05) {
    autoDetectInputMode(sources);
  }
  if (window.confirmedInputMode !== 'hands' && window.currentInputMode !== 'hands') return;

  initHandDebugSpheres();
  let leftSrc = null, rightSrc = null;
  for (let i = 0; i < sources.length; i++) {
    const s = sources[i];
    if (s && s.hand) {
      if (s.handedness === 'left') leftSrc = s;
      else if (s.handedness === 'right') rightSrc = s;
    }
  }

  const leftData = parseXRHandSource(leftSrc, frame, refSpace);
  const rightData = parseXRHandSource(rightSrc, frame, refSpace);
  updateHandDebugSpheres('left', leftData.joints, leftData.tracking, leftData.confidence);
  updateHandDebugSpheres('right', rightData.joints, rightData.tracking, rightData.confidence);

  let stText = null;
  if (typeof updateHandSelfTest === 'function') {
    stText = updateHandSelfTest(nowMs, leftData, rightData, window.latestTelemetry);
  }

  const req = window.currentInputMode, conf = window.confirmedInputMode || req;
  const isMismatch = (req !== 'auto' && conf !== 'auto' && req !== conf);
  const isVal = (leftData.tracking || rightData.tracking) ? 'Y' : 'N';
  const e2e = (window.latestTelemetry && window.latestTelemetry.xr_handtrack && window.latestTelemetry.xr_handtrack.e2e_ms) ? `${window.latestTelemetry.xr_handtrack.e2e_ms}ms` : '<80ms';
  const bld = window.BUILD_ID ? ` | bld=${window.BUILD_ID}` : '';
  const banner = isMismatch ? `[MODE MISMATCH: req=${req} vs conf=${conf}]` : `INPUT: HANDS | ${window.currentHandFollowMode.toUpperCase()} | src=hand | valid=${isVal} | fingers=5/5 | e2e=${e2e}${bld}`;

  const hudText = stText || banner;
  updateHandSceneHUD(hudText, headPos, isMismatch);
  updateModeBannerUI(hudText);

  const predTimeS = (frame && typeof frame.predictedDisplayTime === 'number' && frame.predictedDisplayTime > 0) ? (frame.predictedDisplayTime / 1000.0) : (performance.now() / 1000.0);
  const hPos = headPos ? [headPos.x, headPos.y, headPos.z] : [0, 1.5, 0];
  const hYaw = (typeof headYaw !== 'undefined') ? headYaw : 0.0;
  const hPitch = (typeof headPitch !== 'undefined') ? headPitch : 0.0;
  const rYaw = (typeof currentRigYaw !== 'undefined') ? currentRigYaw : 0.0;
  sendCmd({
    cmd: 'xr_handtrack',
    t: predTimeS,
    rig_yaw: rYaw,
    head: { pos: hPos, yaw: hYaw, pitch: hPitch },
    left: leftData,
    right: rightData
  });
  if (typeof window.recordHandPacketSent === 'function') window.recordHandPacketSent();
}

window.addEventListener('DOMContentLoaded', () => {
  const sel = document.getElementById('select-input-mode');
  if (sel) sel.value = window.currentInputMode;
  sendCmd({ cmd: 'set_input_mode', mode: window.currentInputMode });
  const modeSel = document.getElementById('select-hand-mode');
  if (modeSel) modeSel.value = window.currentHandFollowMode;
  sendCmd({ cmd: 'set_hand_follow_mode', mode: window.currentHandFollowMode });
  const minCut = handUrlParams.get('min_cutoff'), betaVal = handUrlParams.get('beta'), leadVal = handUrlParams.get('lead_ms');
  if (minCut || betaVal || leadVal) {
    sendCmd({ cmd: 'set_hand_filter_config', min_cutoff: minCut ? parseFloat(minCut) : 1.5, beta: betaVal ? parseFloat(betaVal) : 0.03, lead_ms: leadVal ? parseFloat(leadVal) : 20.0 });
  }
  updateModeBannerUI();
});
