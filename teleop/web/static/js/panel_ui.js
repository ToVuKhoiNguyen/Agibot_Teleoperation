
// Hand / Finger Test Helper Functions
let currentHandTab = 'left';
let currentGraspType = 'cylinder';
let activeJointSlidersCreatedFor = '';

function selectHandTab(side) {
  currentHandTab = side;
  const tabL = document.getElementById('hand_tab_l');
  if (tabL) tabL.className = side === 'left' ? 'tab-btn active-tab' : 'tab-btn';
  const tabR = document.getElementById('hand_tab_r');
  if (tabR) tabR.className = side === 'right' ? 'tab-btn active-tab' : 'tab-btn';
  activeJointSlidersCreatedFor = '';
  if (latestStatusData && latestStatusData.hand_status) {
    const sideHs = latestStatusData.hand_status[side];
    updateJointSlidersUI(sideHs);
    updateMimicJointsUI(sideHs);
  }
}

function sendHandGrasp(type) {
  currentGraspType = type;
  const slider = document.getElementById('test_closure_slider');
  const closure = slider ? parseFloat(slider.value) / 100.0 : 0.0;
  sendCmd({ cmd: 'hand_grasp', side: currentHandTab, grasp: type, closure: closure });
}

function onHandClosureSlider(val) {
  const out = document.getElementById('test_closure_val');
  if (out) out.innerText = val + '%';
  const closure = parseFloat(val) / 100.0;
  sendCmd({ cmd: 'hand_grasp', side: currentHandTab, grasp: currentGraspType, closure: closure });
}

function onFingerCurlChange() {
  const elT = document.getElementById('fcurl_thumb');
  const elI = document.getElementById('fcurl_index');
  const elM = document.getElementById('fcurl_mid');
  const elR = document.getElementById('fcurl_ring');
  const elP = document.getElementById('fcurl_pinky');
  const cThumb = elT ? parseFloat(elT.value) / 100.0 : 0.0;
  const cIndex = elI ? parseFloat(elI.value) / 100.0 : 0.0;
  const cMid = elM ? parseFloat(elM.value) / 100.0 : 0.0;
  const cRing = elR ? parseFloat(elR.value) / 100.0 : 0.0;
  const cPinky = elP ? parseFloat(elP.value) / 100.0 : 0.0;
  const outT = document.getElementById('fcurl_thumb_val'); if (outT) outT.innerText = Math.round(cThumb * 100);
  const outI = document.getElementById('fcurl_index_val'); if (outI) outI.innerText = Math.round(cIndex * 100);
  const outM = document.getElementById('fcurl_mid_val'); if (outM) outM.innerText = Math.round(cMid * 100);
  const outR = document.getElementById('fcurl_ring_val'); if (outR) outR.innerText = Math.round(cRing * 100);
  const outP = document.getElementById('fcurl_pinky_val'); if (outP) outP.innerText = Math.round(cPinky * 100);
  sendCmd({ cmd: 'hand_curls', side: currentHandTab, curls: [cThumb, cIndex, cMid, cRing, cPinky], pinch: 0.0 });
}

function onDirectJointSlider(jname, val) {
  const rad = parseFloat(val);
  const outEl = document.getElementById('jval_' + jname);
  if (outEl) outEl.innerText = rad.toFixed(2) + ' rad';
  sendCmd({ cmd: 'hand_joint', side: currentHandTab, joint: jname, angle: rad });
}

function updateJointSlidersUI(sideHs) {
  const container = document.getElementById('joint_sliders_container');
  if (!container || !sideHs || !sideHs.active_joints) return;
  const jnames = Object.keys(sideHs.active_joints);
  if (activeJointSlidersCreatedFor !== currentHandTab) {
    activeJointSlidersCreatedFor = currentHandTab;
    container.innerHTML = jnames.map(j => {
      const data = sideHs.active_joints[j];
      const shortName = j.replace(/^[LR]_/, '').replace(/_joint$/, '');
      return `<div style="display:flex; align-items:center; justify-content:space-between; margin:2px 0;">
        <span style="width:70px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap;" title="${j}">${shortName}</span>
        <input type="range" id="jslider_${j}" min="${data.min}" max="${data.max}" step="0.01" value="${data.val}" oninput="onDirectJointSlider('${j}', this.value)" style="flex:1; margin:0 4px;">
        <span id="jval_${j}" style="width:55px; text-align:right;">${data.val.toFixed(2)} rad</span>
      </div>`;
    }).join('');
  } else {
    jnames.forEach(j => {
      const data = sideHs.active_joints[j];
      const slider = document.getElementById('jslider_' + j);
      const out = document.getElementById('jval_' + j);
      if (slider && document.activeElement !== slider) slider.value = data.val;
      if (out) out.innerText = `${data.val.toFixed(2)} rad`;
    });
  }
}

function updateMimicJointsUI(sideHs) {
  const container = document.getElementById('mimic_joints_container');
  if (!container || !sideHs || !sideHs.mimic_joints) return;
  const entries = Object.entries(sideHs.mimic_joints);
  container.innerHTML = entries.map(([k, v]) => {
    const sName = k.replace(/^[LR]_/, '').replace(/_joint$/, '');
    return `<div>${sName}: <b>${v.toFixed(2)}</b> rad</div>`;
  }).join('');
}

// Sliders and button events
['step_xyz_l', 'step_xyz_r'].forEach(id => {
  const el = document.getElementById(id);
  const out = document.getElementById(id + '_val');
  if (el && out) el.addEventListener('input', () => { out.innerText = el.value + ' cm'; });
});
['step_rpy_l', 'step_rpy_r'].forEach(id => {
  const el = document.getElementById(id);
  const out = document.getElementById(id + '_val');
  if (el && out) el.addEventListener('input', () => { out.innerText = el.value + '°'; });
});

window.isDraggingSliderL = false;
window.isDraggingSliderR = false;

// Data recording: REC button (same as a short tap of A on the right controller)
function recToggle() {
  const active = Boolean(window.latestTelemetry && window.latestTelemetry.rec && window.latestTelemetry.rec.active);
  const t = document.getElementById('rec_task');
  const ins = document.getElementById('rec_instruction');
  sendCmd(active ? {cmd: 'rec_stop'} : {cmd: 'rec_start', task: t ? t.value : 'default', instruction: ins ? ins.value : ''});
}
window.recToggle = recToggle;
function recDiscard() {
  const sel = document.getElementById('rec_discard_reason');
  sendCmd({cmd: 'rec_discard', reason: sel ? sel.value : undefined});
}
window.recDiscard = recDiscard;

// Controller -> robot scale (Quest controllers): slider + presets, synced from telemetry (s.xr_scale)
window.isDraggingXrScale = false;
function setXrScale(v) { sendCmd({cmd: 'set_xr_scale', scale: v}); }
window.setXrScale = setXrScale;
const sliderXrScale = document.getElementById('slider_xr_scale');
if (sliderXrScale) {
  sliderXrScale.addEventListener('pointerdown', () => { window.isDraggingXrScale = true; });
  sliderXrScale.addEventListener('pointerup', () => { window.isDraggingXrScale = false; });
  sliderXrScale.addEventListener('input', (e) => {
    const v = e.target.value / 100.0;
    const lab = document.getElementById('slider_xr_scale_val');
    if (lab) lab.innerText = '1 : ' + v.toFixed(2);
    setXrScale(v);
  });
}

// Neck posture control: pitch, yaw, roll
window.isDraggingNeck = false;
function sendNeckPose(pitchDeg, yawDeg, rollDeg) {
  const p = (pitchDeg !== undefined) ? parseFloat(pitchDeg) : 0.0;
  const y = (yawDeg !== undefined) ? parseFloat(yawDeg) : 0.0;
  const r = (rollDeg !== undefined) ? parseFloat(rollDeg) : 0.0;
  const sPitch = document.getElementById('slider_neck_pitch');
  const sPitchV = document.getElementById('slider_neck_pitch_val');
  const sYaw = document.getElementById('slider_neck_yaw');
  const sYawV = document.getElementById('slider_neck_yaw_val');
  const sRoll = document.getElementById('slider_neck_roll');
  const sRollV = document.getElementById('slider_neck_roll_val');
  if (sPitch) sPitch.value = Math.round(p);
  if (sPitchV) sPitchV.innerText = (p > 0 ? '+' : '') + Math.round(p) + '°';
  if (sYaw) sYaw.value = Math.round(y);
  if (sYawV) sYawV.innerText = (y > 0 ? '+' : '') + Math.round(y) + '°';
  if (sRoll) sRoll.value = Math.round(r);
  if (sRollV) sRollV.innerText = (r > 0 ? '+' : '') + Math.round(r) + '°';
  sendCmd({ cmd: 'set_neck', pitch_deg: p, yaw_deg: y, roll_deg: r });
}
window.sendNeckPose = sendNeckPose;

function _dispatchNeckSliders() {
  const sPitch = document.getElementById('slider_neck_pitch');
  const sYaw = document.getElementById('slider_neck_yaw');
  const sRoll = document.getElementById('slider_neck_roll');
  const p = sPitch ? parseFloat(sPitch.value) : 0.0;
  const y = sYaw ? parseFloat(sYaw.value) : 0.0;
  const r = sRoll ? parseFloat(sRoll.value) : 0.0;
  sendCmd({ cmd: 'set_neck', pitch_deg: p, yaw_deg: y, roll_deg: r });
}

['slider_neck_pitch', 'slider_neck_yaw', 'slider_neck_roll'].forEach((id) => {
  const el = document.getElementById(id);
  if (el) {
    el.addEventListener('pointerdown', () => { window.isDraggingNeck = true; });
    el.addEventListener('pointerup', () => { window.isDraggingNeck = false; });
    el.addEventListener('input', (e) => {
      const val = parseFloat(e.target.value);
      const valText = document.getElementById(id + '_val');
      if (valText) valText.innerText = (val > 0 ? '+' : '') + Math.round(val) + '°';
      _dispatchNeckSliders();
    });
  }
});

const sliderHandL = document.getElementById('slider_hand_l');
if (sliderHandL) {
  sliderHandL.addEventListener('pointerdown', () => { window.isDraggingSliderL = true; });
  sliderHandL.addEventListener('pointerup', () => { window.isDraggingSliderL = false; });
  sliderHandL.addEventListener('input', (e) => {
    sendCmd({cmd: 'hand', arm: 'left', action: 'slider', value: e.target.value / 100.0});
  });
}
const sliderHandR = document.getElementById('slider_hand_r');
if (sliderHandR) {
  sliderHandR.addEventListener('pointerdown', () => { window.isDraggingSliderR = true; });
  sliderHandR.addEventListener('pointerup', () => { window.isDraggingSliderR = false; });
  sliderHandR.addEventListener('input', (e) => {
    sendCmd({cmd: 'hand', arm: 'right', action: 'slider', value: e.target.value / 100.0});
  });
}

function setupThumbOpenSlider(id, idVal) {
  const el = document.getElementById(id);
  const elVal = document.getElementById(idVal);
  if (!el) return;
  el.addEventListener('pointerdown', () => { window.isDraggingThumbOpen = true; });
  el.addEventListener('pointerup', () => { window.isDraggingThumbOpen = false; });
  el.addEventListener('input', (e) => {
    const val = parseInt(e.target.value, 10);
    if (elVal) elVal.innerText = val;
    const otherId = (id === 'slider_thumb_open') ? 'slider_thumb_open_r' : 'slider_thumb_open';
    const otherValId = otherId + '_val';
    const otherEl = document.getElementById(otherId);
    const otherValEl = document.getElementById(otherValId);
    if (otherEl && !window.isDraggingThumbOpen) {
      otherEl.value = val;
      if (otherValEl) otherValEl.innerText = val;
    }
    sendCmd({cmd: 'thumb_open', value: val});
  });
}
setupThumbOpenSlider('slider_thumb_open', 'slider_thumb_open_val');
setupThumbOpenSlider('slider_thumb_open_r', 'slider_thumb_open_r_val');

// Hold-to-repeat buttons
let repeatTimer = null;
let currentCmd = null;

function executeRepeatable() {
  if (!currentCmd) return;
  const cmdObj = JSON.parse(JSON.stringify(currentCmd));
  const arm = cmdObj.arm;
  if (cmdObj.axis === 'roll' || cmdObj.axis === 'pitch' || cmdObj.axis === 'yaw') {
    cmdObj.step_deg = parseFloat(document.getElementById('step_rpy_' + (arm==='left'?'l':'r')).value);
  } else {
    cmdObj.step_cm = parseFloat(document.getElementById('step_xyz_' + (arm==='left'?'l':'r')).value);
  }
  const targetObj = (arm === 'left') ? targetSphereL : targetSphereR;
  if (targetObj) {
    cmdObj.cur_pos = [targetObj.position.x, targetObj.position.y, targetObj.position.z];
    cmdObj.cur_quat = [targetObj.quaternion.x, targetObj.quaternion.y, targetObj.quaternion.z, targetObj.quaternion.w];
  }
  sendCmd(cmdObj);
}

document.querySelectorAll('.repeatable').forEach(btn => {
  const startRepeat = (e) => {
    e.preventDefault();
    currentCmd = JSON.parse(btn.getAttribute('data-cmd'));
    executeRepeatable();
    repeatTimer = setInterval(executeRepeatable, 120);
  };
  const stopRepeat = () => {
    if (repeatTimer) { clearInterval(repeatTimer); repeatTimer = null; }
    currentCmd = null;
  };
  btn.addEventListener('mousedown', startRepeat);
  btn.addEventListener('mouseup', stopRepeat);
  btn.addEventListener('mouseleave', stopRepeat);
  btn.addEventListener('touchstart', startRepeat, {passive: false});
  btn.addEventListener('touchend', stopRepeat);
});

async function dumpHandLog() {
  const fb = document.getElementById('dump-hand-log-feedback');
  if (fb) {
    fb.style.display = 'block';
    fb.style.color = '#38bdf8';
    fb.innerText = 'Dumping hand log...';
  }
  try {
    const res = await fetch('/api/dump_hand_log', { method: 'POST' });
    const data = await res.json();
    if (fb) {
      fb.style.color = '#4caf50';
      fb.innerText = `Saved ${data.path || '/tmp/hand_log.json'}`;
      setTimeout(() => { fb.style.display = 'none'; }, 3500);
    }
  } catch (err) {
    if (fb) {
      fb.style.color = '#f44336';
      fb.innerText = `Failed: ${err.message}`;
    }
  }
}

// Global Client Error Reporter
let lastErrorLogTimes = {};
window.frameCountXR = 0;

function reportClientError(err, extra = {}) {
  const msg = (err && (err.message || err.toString())) || 'Unknown Client Error';
  const stack = (err && err.stack) || '';
  const fileLine = (extra && extra.fileLine) || (stack ? (stack.split('\n')[1] || '').trim() : '');
  const now = Date.now();
  if (lastErrorLogTimes[msg] && (now - lastErrorLogTimes[msg] < 1000)) return;
  lastErrorLogTimes[msg] = now;

  window.lastXRError = msg;
  const bText = document.getElementById('fault-banner-text');
  const bBox = document.getElementById('fault-banner');
  if (bText && bBox) {
    bText.innerText = `${msg}`;
    bBox.style.display = 'flex';
    bBox.style.background = '#d32f2f';
  }
  if (typeof updateXRDiagOverlay === 'function') updateXRDiagOverlay();

  const payload = {
    message: msg,
    stack: stack,
    "file:line": fileLine,
    "build id": window.BUILD_ID || 'unknown',
    "input mode": window.confirmedInputMode || window.currentInputMode || 'none',
    "frame count": window.frameCountXR || 0,
    time: new Date().toISOString()
  };
  try {
    fetch('/api/client_log', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    }).catch(() => {});
  } catch (_) {}
}
window.reportClientError = reportClientError;

window.onerror = function(message, source, lineno, colno, error) {
  reportClientError(error || message, { fileLine: `${source}:${lineno}:${colno}` });
};
window.addEventListener('unhandledrejection', function(event) {
  reportClientError(event.reason || 'Unhandled Promise Rejection');
});

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  init3D();
  connectWS();
});
