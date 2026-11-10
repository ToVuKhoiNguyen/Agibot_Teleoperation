
// ==========================================
// 2. WEBSOCKET TELEMETRY & COMMAND PIPELINE
// ==========================================
let ws = null;

function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url = `${proto}//${location.host}/ws_panel`;
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById('conn-tag').innerText = 'Connected';
    document.getElementById('conn-tag').className = 'tag tag-ok';
  };

  ws.binaryType = 'arraybuffer';
  ws.onmessage = (event) => {
    if (event.data instanceof ArrayBuffer) {
      handleBinaryPoses(event.data);
      return;
    }
    try {
      const data = JSON.parse(event.data);
      if (data.type === 'manifest') {
        buildRobotHierarchy(data);
      } else if (data.type === 'poses') {
        applyMuJoCoPoses(data.poses);
      } else if (data.type === 'status') {
        updateUIStatus(data.status);
      } else if (data.ee_l) {
        updateUIStatus(data);
      }
    } catch(e) {}
  };

  ws.onclose = () => {
    document.getElementById('conn-tag').innerText = 'Disconnected';
    document.getElementById('conn-tag').className = 'tag tag-danger';
    document.getElementById('sync-badge').innerText = 'DISCONNECTED (RECONNECTING...)';
    document.getElementById('sync-badge').className = 'tag tag-danger';
    setTimeout(connectWS, 1000);
  };
}

function sendCmd(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  } else {
    fetch('/api/command', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(obj)
    }).catch(e => console.error(e));
  }
}

function updateUIStatus(s) {
  latestStatusData = s;
  window.latestTelemetry = s;
  window.lastTelemetryState = s;
  lastLeftClosure = s.hand_closure_l;
  lastRightClosure = s.hand_closure_r;

  // Update badges
  const inputSrcTag = document.getElementById('input-src-tag');
  if (inputSrcTag) {
    const isXR = s.input_source === 'QUEST VR' || s.xr_controllers_active;
    inputSrcTag.innerText = isXR ? 'INPUT: QUEST VR' : 'INPUT: MOUSE';
    inputSrcTag.className = isXR ? 'tag tag-warn' : 'tag tag-ok';
  }
  const simTag = document.getElementById('sim-tag');
  if (simTag) {
    simTag.innerText = s.sim_running !== false ? 'SIM: OK' : 'SIM: STOPPED';
    simTag.className = s.sim_running !== false ? 'tag tag-ok' : 'tag tag-danger';
  }
  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = s.xr_active ? 'XR: ACTIVE' : 'XR: IDLE';
    xrTag.className = s.xr_active ? 'tag tag-ok' : 'tag tag-warn';
  }
  const estopTagEl = document.getElementById('estop-tag');
  if (estopTagEl) {
    estopTagEl.innerText = s.estop ? 'ESTOP: ACTIVE' : 'ESTOP: OFF';
    estopTagEl.className = s.estop ? 'tag tag-danger' : 'tag tag-ok';
  }

  // Sync actual spheres
  if (actualSphereL && s.ee_l) actualSphereL.position.set(s.ee_l[0], s.ee_l[1], s.ee_l[2]);
  if (actualSphereR && s.ee_r) actualSphereR.position.set(s.ee_r[0], s.ee_r[1], s.ee_r[2]);

  // Snap gizmo to FK(measured) once upon transition to READY or fault clear
  if ((window._lastSyncState === 'SYNC' && s.sync_state === 'READY') || (window._lastFaultLatched && !s.fault_latched)) {
    if (s.tgt_l && targetSphereL) targetSphereL.position.set(s.tgt_l[0], s.tgt_l[1], s.tgt_l[2]);
    if (s.tgt_r && targetSphereR) targetSphereR.position.set(s.tgt_r[0], s.tgt_r[1], s.tgt_r[2]);
  }
  window._lastSyncState = s.sync_state;
  window._lastFaultLatched = Boolean(s.fault_latched);

  // Sync target marker positions when not dragging
  if (!isDraggingGizmo) {
    const clutchedL = Boolean(s.xr_grip_l || s.held);
    const clutchedR = Boolean(s.xr_grip_r || s.held);

    if (targetSphereL) {
      const pL = (followActualWrist && s.ee_l) ? s.ee_l : (s.tgt_l || s.ee_l);
      if (pL) targetSphereL.position.set(pL[0], pL[1], pL[2]);
      const rL = (clutchedL && s.tgt_rot_l) ? s.tgt_rot_l : (s.rot_l || s.tgt_rot_l);
      if (rL && gizmoMode !== 'rotate') {
        const m = new THREE.Matrix4();
        m.set(rL[0][0], rL[0][1], rL[0][2], 0, rL[1][0], rL[1][1], rL[1][2], 0, rL[2][0], rL[2][1], rL[2][2], 0, 0, 0, 0, 1);
        targetSphereL.quaternion.setFromRotationMatrix(m);
      }
    }

    if (targetSphereR) {
      const pR = (followActualWrist && s.ee_r) ? s.ee_r : (s.tgt_r || s.ee_r);
      if (pR) targetSphereR.position.set(pR[0], pR[1], pR[2]);
      const rR = (clutchedR && s.tgt_rot_r) ? s.tgt_rot_r : (s.rot_r || s.tgt_rot_r);
      if (rR && gizmoMode !== 'rotate') {
        const m = new THREE.Matrix4();
        m.set(rR[0][0], rR[0][1], rR[0][2], 0, rR[1][0], rR[1][1], rR[1][2], 0, rR[2][0], rR[2][1], rR[2][2], 0, 0, 0, 0, 1);
        targetSphereR.quaternion.setFromRotationMatrix(m);
      }
    }
  }

  // Update HUD gizmo feedback pill (IK Error + Clamping + Collision)
  const hudFeedback = document.getElementById('gizmo-feedback');
  if (hudFeedback) {
    const ikErr = (activeGizmoArm === 'left') ? (s.ik_err_l || 0.0) : (s.ik_err_r || 0.0);
    let txt = `${activeGizmoArm.toUpperCase()} IK Err: ${ikErr.toFixed(1)} mm`;
    if (s.collision_blocked) {
      txt = `⚠️ Self-collision blocked (${(s.min_distance_mm || 0.0).toFixed(1)} mm)`;
      hudFeedback.style.color = '#ff1744';
      hudFeedback.style.borderColor = '#ff1744';
      hudFeedback.style.backgroundColor = 'rgba(255, 23, 68, 0.2)';
    } else if (s.hand_collision_blocked) {
      txt = `⚠️ Hand collision blocked (${(s.min_hand_distance_mm || 0.0).toFixed(1)} mm)`;
      hudFeedback.style.color = '#ff9100';
      hudFeedback.style.borderColor = '#ff9100';
      hudFeedback.style.backgroundColor = 'rgba(255, 145, 0, 0.2)';
    } else if (s.warning && s.warning.toLowerCase().includes('clamp')) {
      txt += ` ⚠️ CLAMPED`;
      hudFeedback.style.color = '#ff5252';
      hudFeedback.style.borderColor = '#ff5252';
      hudFeedback.style.backgroundColor = 'rgba(18, 22, 32, 0.85)';
    } else {
      if (s.predicted_clearance_mm !== undefined && s.predicted_clearance_mm < 900.0) {
        txt += ` | Clearance: ${s.predicted_clearance_mm.toFixed(0)} mm`;
      }
      hudFeedback.style.color = ikErr > 15.0 ? '#ffb74d' : '#81c784';
      hudFeedback.style.borderColor = '#32415d';
      hudFeedback.style.backgroundColor = 'rgba(18, 22, 32, 0.85)';
    }
    hudFeedback.innerText = txt;
    hudFeedback.style.display = 'inline-block';
  }

  // Tint blocked wrist marker red if collision blocked
  if (s.collision_blocked) {
    if (activeGizmoArm === 'left' && targetSphereL) {
      targetSphereL.material.color.setHex(0xff1744);
      targetSphereL.material.emissive.setHex(0xb71c1c);
    } else if (activeGizmoArm === 'right' && targetSphereR) {
      targetSphereR.material.color.setHex(0xff1744);
      targetSphereR.material.emissive.setHex(0xb71c1c);
    }
  } else {
    if (targetSphereL) {
      targetSphereL.material.color.setHex(0x00e676);
      targetSphereL.material.emissive.setHex(0x00a844);
    }
    if (targetSphereR) {
      targetSphereR.material.color.setHex(0x2979ff);
      targetSphereR.material.emissive.setHex(0x1565c0);
    }
  }


  // Sync sliders
  const sL = document.getElementById('slider_hand_l'), sLv = document.getElementById('slider_hand_l_val');
  if (sL) sL.value = Math.round(s.hand_closure_l * 100);
  if (sLv) sLv.innerText = Math.round(s.hand_closure_l * 100) + '%';
  const sR = document.getElementById('slider_hand_r'), sRv = document.getElementById('slider_hand_r_val');
  if (sR) sR.value = Math.round(s.hand_closure_r * 100);
  if (sRv) sRv.innerText = Math.round(s.hand_closure_r * 100) + '%';

  // Update persistent Hardware Fault Banner
  const faultBanner = document.getElementById('fault-banner');
  if (faultBanner) {
    if (s.fault_latched || window.forceFaultBanner || (s.warning && s.warning.includes('HARDWARE FAULT'))) {
      faultBanner.style.display = 'flex';
      const faultText = document.getElementById('fault-banner-text');
      if (faultText) faultText.innerText = window.forceFaultText || (s.fault_reason ? `⚠️ FAULT: ${s.fault_reason}` : (s.warning || '⚠️ HARDWARE FAULT LATCHED'));
    } else {
      faultBanner.style.display = 'none';
    }
  }

  // Update Sync / Ready / Engaged status badge (shows reasons when not moving)
  const syncBadge = document.getElementById('sync-badge');
  if (syncBadge) {
    let st = s.sync_state || (s.fault_latched ? 'FAULT' : 'READY');
    let sub = '';
    if (s.fault_latched || st === 'FAULT') { st = 'FAULT'; sub = ` (${s.fault_reason || 'Clear to resume'})`; }
    else if (st === 'SYNC') { sub = ' (WAITING FEEDBACK: no packet on UDP 5006)'; }
    else if (st === 'READY') { sub = ' (NOT ENGAGED)'; }
    else if (s.feedback_age_ms > 200) { sub = ` (STALE FEEDBACK: ${s.feedback_age_ms.toFixed(0)}ms)`; }
    else if (s.max_sim_real_gap_rad >= 0.095) { sub = ' (LEAD LIMIT)'; }
    const gap = (s.max_sim_real_gap_rad !== undefined && st === 'ENGAGED') ? ` • gap: ${s.max_sim_real_gap_rad.toFixed(3)} rad` : '';
    syncBadge.innerText = `${st}${sub}${gap}`;
    syncBadge.className = `tag tag-${st === 'ENGAGED' ? 'ok' : (st === 'READY' ? 'info' : (st === 'FAULT' ? 'danger' : 'warn'))}`;
  }

  const engageBtn = document.getElementById('btn-engage-arms');
  if (engageBtn) {
    const isEng = (s.sync_state === 'ENGAGED');
    engageBtn.innerText = isEng ? 'Disengage' : 'Engage';
    engageBtn.style.background = isEng ? '#e65100' : '#2e7d32';
    engageBtn.style.display = (s.sync_state === 'SYNC') ? 'none' : 'inline-block';
  }

  const hudProf = document.getElementById('hud-profile-stat');
  if (hudProf) {
    const prof = (s.speed_profile || 'normal').toUpperCase();
    const vel = s.speed_profile === 'slow' ? '0.2' : (s.speed_profile === 'fast' ? '0.48' : '0.3');
    hudProf.innerText = `Profile: ${prof} (${vel} rad/s)`;
  }

  if (typeof updateTelemetryUI === 'function') updateTelemetryUI(s);
};

window.confirmClearFault = function() {
  if (confirm("Operator confirmation required: Clear latched hardware fault and resume? (Note: Orin latch requires node restart)")) {
    sendCmd({cmd: "clear_fault", confirmed: true});
  }
};

window.toggleEngageArms = function() {
  const current = window.lastTelemetryState && window.lastTelemetryState.sync_state;
  if (current === 'ENGAGED') {
    sendCmd({cmd: "disengage_arm", arm: "both"});
  } else {
    sendCmd({cmd: "engage_arm", arm: "both"});
  }
};
