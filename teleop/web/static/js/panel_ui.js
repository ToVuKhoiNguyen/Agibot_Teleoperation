
// Hand / Finger Test Helper Functions
let currentHandTab = 'left';
let currentGraspType = 'cylinder';
let activeJointSlidersCreatedFor = '';

function selectHandTab(side) {
  currentHandTab = side;
  document.getElementById('hand_tab_l').className = side === 'left' ? 'tab-btn active-tab' : 'tab-btn';
  document.getElementById('hand_tab_r').className = side === 'right' ? 'tab-btn active-tab' : 'tab-btn';
  activeJointSlidersCreatedFor = '';
  if (latestStatusData && latestStatusData.hand_status) {
    const sideHs = latestStatusData.hand_status[side];
    updateJointSlidersUI(sideHs);
    updateMimicJointsUI(sideHs);
  }
}

function sendHandGrasp(type) {
  currentGraspType = type;
  const closure = parseFloat(document.getElementById('test_closure_slider').value) / 100.0;
  sendCmd({ cmd: 'hand_grasp', side: currentHandTab, grasp: type, closure: closure });
}

function onHandClosureSlider(val) {
  document.getElementById('test_closure_val').innerText = val + '%';
  const closure = parseFloat(val) / 100.0;
  sendCmd({ cmd: 'hand_grasp', side: currentHandTab, grasp: currentGraspType, closure: closure });
}

function onFingerCurlChange() {
  const cThumb = parseFloat(document.getElementById('fcurl_thumb').value) / 100.0;
  const cIndex = parseFloat(document.getElementById('fcurl_index').value) / 100.0;
  const cMid = parseFloat(document.getElementById('fcurl_mid').value) / 100.0;
  const cRing = parseFloat(document.getElementById('fcurl_ring').value) / 100.0;
  const cPinky = parseFloat(document.getElementById('fcurl_pinky').value) / 100.0;
  document.getElementById('fcurl_thumb_val').innerText = Math.round(cThumb * 100);
  document.getElementById('fcurl_index_val').innerText = Math.round(cIndex * 100);
  document.getElementById('fcurl_mid_val').innerText = Math.round(cMid * 100);
  document.getElementById('fcurl_ring_val').innerText = Math.round(cRing * 100);
  document.getElementById('fcurl_pinky_val').innerText = Math.round(cPinky * 100);
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
  el.addEventListener('input', () => { out.innerText = el.value + ' cm'; });
});
['step_rpy_l', 'step_rpy_r'].forEach(id => {
  const el = document.getElementById(id);
  const out = document.getElementById(id + '_val');
  el.addEventListener('input', () => { out.innerText = el.value + '°'; });
});

document.getElementById('slider_hand_l').addEventListener('input', (e) => {
  sendCmd({cmd: 'hand', arm: 'left', action: 'slider', value: e.target.value / 100.0});
});
document.getElementById('slider_hand_r').addEventListener('input', (e) => {
  sendCmd({cmd: 'hand', arm: 'right', action: 'slider', value: e.target.value / 100.0});
});

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

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  init3D();
  connectWS();
});
