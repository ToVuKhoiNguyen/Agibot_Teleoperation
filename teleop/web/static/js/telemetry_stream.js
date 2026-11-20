
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
      } else if (data.type === 'task_scene_def') {
        if (typeof buildTaskScene === 'function') buildTaskScene(data);
      } else if (data.type === 'task_poses') {
        if (typeof applyTaskPoses === 'function') applyTaskPoses(data);
      } else if (data.type === 'status') {
        updateUIStatus(data.status);
      } else if (data.type === 'gizmo_recall') {
        if (typeof handleGizmoRecall === 'function') handleGizmoRecall(data);
      } else if (data.type === 'gizmo_set') {
        if (typeof handleGizmoSet === 'function') handleGizmoSet(data);
      } else if (data.ee_l) {
        updateUIStatus(data);
      }
    } catch(e) {}
  };

  ws.onclose = () => {
    const cTag = document.getElementById('conn-tag'); if (cTag) { cTag.innerText = 'Disconnected'; cTag.className = 'tag tag-danger'; }
    const sBadge = document.getElementById('sync-badge'); if (sBadge) { sBadge.innerText = 'DISCONNECTED (RECONNECTING...)'; sBadge.className = 'tag tag-danger'; }
    setTimeout(connectWS, 1000);
  };
}

function sendCmd(obj) {
  if (obj && (obj.cmd === 'preset' || obj.cmd === 'go_home' || obj.cmd === 'home' || obj.cmd === 'reset')) {
    window._allowGizmoServerSync = true;
  }
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
    simTag.style.display = s.sim_running !== false ? 'none' : '';
  }
  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = s.xr_active ? 'XR: ACTIVE' : 'XR: IDLE';
    xrTag.className = s.xr_active ? 'tag tag-ok' : 'tag tag-warn';
  }
  const estopTagEl = document.getElementById('estop-tag');
  if (estopTagEl) {
    estopTagEl.innerText = 'ESTOP: ACTIVE';
    estopTagEl.className = 'tag tag-danger';
    estopTagEl.style.display = s.estop ? '' : 'none';   // hidden while OFF, visible only when the e-stop is active
  }

  // Sync actual spheres
  if (actualSphereL && s.ee_l) actualSphereL.position.set(s.ee_l[0], s.ee_l[1], s.ee_l[2]);
  if (actualSphereR && s.ee_r) actualSphereR.position.set(s.ee_r[0], s.ee_r[1], s.ee_r[2]);

  // Snap gizmo (position AND rotation) to the server target once upon initial load, transition to READY or
  // fault clear. Rotation matters: a gizmo left at a stale orientation commands a large wrist rotation on the
  // first pixel of any drag, which twists the arm into the torso (ARMOR block).
  const snapGizmo = (obj, pos, rot, withRot) => {
    if (!obj) return;
    if (pos) obj.position.set(pos[0], pos[1], pos[2]);
    if (rot && withRot) {
      const m = new THREE.Matrix4();
      m.set(rot[0][0], rot[0][1], rot[0][2], 0, rot[1][0], rot[1][1], rot[1][2], 0, rot[2][0], rot[2][1], rot[2][2], 0, 0, 0, 0, 1);
      obj.quaternion.setFromRotationMatrix(m);
    }
  };
  if (!window._hasInitialGizmoSync || (window._lastSyncState === 'SYNC' && s.sync_state === 'READY') || (window._lastFaultLatched && !s.fault_latched)) {
    if (!isDraggingGizmo) {
      snapGizmo(targetSphereL, s.tgt_l, s.tgt_rot_l, true);
      snapGizmo(targetSphereR, s.tgt_r, s.tgt_rot_r, true);
      if (typeof sceneNeedsRender !== 'undefined') sceneNeedsRender = true;
    }
    window._hasInitialGizmoSync = true;
  }
  const justSynced = (window._lastSyncState === 'SYNC' && s.sync_state === 'READY');
  window._lastSyncState = s.sync_state;
  window._lastFaultLatched = Boolean(s.fault_latched);

  // Sync target markers ONLY on explicit server sync triggers (Home, T-pose, Sync transition)
  // While the server moves the targets itself (preset / home / wave / circle / grasp), and once more on the
  // update after it finishes, the gizmo follows the server target (a one-shot sync on click lands at the start).
  // Also while a Quest controller clutch drives the targets (xr_driving): the gizmo must show where the hand target is.
  const autoMotion = Boolean(s.auto_motion || s.xr_driving);
  const allowSync = Boolean(window._allowGizmoServerSync || justSynced || autoMotion || window._lastAutoMotion);
  window._lastAutoMotion = autoMotion;
  if (allowSync && !isDraggingGizmo) {
    snapGizmo(targetSphereL, s.tgt_l, s.tgt_rot_l, autoMotion || gizmoMode !== 'rotate');
    snapGizmo(targetSphereR, s.tgt_r, s.tgt_rot_r, autoMotion || gizmoMode !== 'rotate');
    if (typeof sceneNeedsRender !== 'undefined') sceneNeedsRender = true;
    window._allowGizmoServerSync = false;
  }

  // Update HUD gizmo feedback pill (IK Error + Clamping + Collision)
  const hudFeedback = document.getElementById('gizmo-feedback');
  if (hudFeedback) {
    const isLeft = (activeGizmoArm === 'left');
    const ikErr = isLeft ? (s.ik_err_l || 0.0) : (s.ik_err_r || 0.0);
    const rotErr = isLeft ? (s.ik_rot_err_l || 0.0) : (s.ik_rot_err_r || 0.0);
    const relaxed = isLeft ? Boolean(s.ik_relaxed_l) : Boolean(s.ik_relaxed_r);
    let txt = `${activeGizmoArm.toUpperCase()} IK: ${ikErr.toFixed(1)} mm`;
    if (relaxed) txt += ` (ori ${rotErr.toFixed(0)}°)`;
    if (s.collision_blocked) {
      txt = `Self-collision blocked (${(s.min_distance_mm || 0.0).toFixed(1)} mm)`;
      hudFeedback.style.color = hudFeedback.style.borderColor = '#ff1744';
      hudFeedback.style.backgroundColor = 'rgba(255, 23, 68, 0.2)';
    } else if (s.hand_collision_blocked) {
      txt = `Hand collision blocked (${(s.min_hand_distance_mm || 0.0).toFixed(1)} mm)`;
      hudFeedback.style.color = hudFeedback.style.borderColor = '#ff9100';
      hudFeedback.style.backgroundColor = 'rgba(255, 145, 0, 0.2)';
    } else if (s.reach_warning || s.proxy_collision_warning) {
      txt = s.reach_warning ? `${s.reach_warning}` : s.proxy_collision_warning;
      hudFeedback.style.color = hudFeedback.style.borderColor = '#ffd600';
      hudFeedback.style.backgroundColor = 'rgba(255, 214, 0, 0.15)';
    } else if (s.warning && s.warning.toLowerCase().includes('clamp')) {
      txt += ` CLAMPED`;
      hudFeedback.style.color = hudFeedback.style.borderColor = '#ff5252';
      hudFeedback.style.backgroundColor = 'rgba(18, 22, 32, 0.85)';
    } else {
      if (s.predicted_clearance_mm !== undefined && s.predicted_clearance_mm < 900.0) {
        txt += ` | Clr: ${s.predicted_clearance_mm.toFixed(0)} mm`;
      }
      hudFeedback.style.color = (relaxed || ikErr > 15.0) ? '#ffd600' : '#81c784';
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


  updateRecUI(s.rec);

  // Sync sliders (only when user is not actively dragging them)
  const sX = document.getElementById('slider_xr_scale'), sXv = document.getElementById('slider_xr_scale_val');
  if (typeof s.xr_scale === 'number' && !window.isDraggingXrScale) {
    if (sX) sX.value = Math.round(s.xr_scale * 100);
    if (sXv) sXv.innerText = '1 : ' + s.xr_scale.toFixed(2);
  }
  const sL = document.getElementById('slider_hand_l'), sLv = document.getElementById('slider_hand_l_val');
  if (sL && !window.isDraggingSliderL) sL.value = Math.round(s.hand_closure_l * 100);
  if (sLv && !window.isDraggingSliderL) sLv.innerText = Math.round(s.hand_closure_l * 100) + '%';
  const sR = document.getElementById('slider_hand_r'), sRv = document.getElementById('slider_hand_r_val');
  if (sR && !window.isDraggingSliderR) sR.value = Math.round(s.hand_closure_r * 100);
  if (sRv && !window.isDraggingSliderR) sRv.innerText = Math.round(s.hand_closure_r * 100) + '%';
  const sThL = document.getElementById('slider_thumb_open'), sThLv = document.getElementById('slider_thumb_open_val');
  const sThR = document.getElementById('slider_thumb_open_r'), sThRv = document.getElementById('slider_thumb_open_r_val');
  if (typeof s.thumb_open === 'number' && !window.isDraggingThumbOpen) {
    if (sThL) sThL.value = s.thumb_open;
    if (sThLv) sThLv.innerText = s.thumb_open;
    if (sThR) sThR.value = s.thumb_open;
    if (sThRv) sThRv.innerText = s.thumb_open;
  }
  const sNeckP = document.getElementById('slider_neck_pitch'), sNeckPV = document.getElementById('slider_neck_pitch_val');
  const sNeckY = document.getElementById('slider_neck_yaw'), sNeckYV = document.getElementById('slider_neck_yaw_val');
  const sNeckR = document.getElementById('slider_neck_roll'), sNeckRV = document.getElementById('slider_neck_roll_val');
  if (!window.isDraggingNeck) {
    if (typeof s.neck_pitch_deg === 'number') {
      if (sNeckP) sNeckP.value = Math.round(s.neck_pitch_deg);
      if (sNeckPV) sNeckPV.innerText = (s.neck_pitch_deg > 0 ? '+' : '') + Math.round(s.neck_pitch_deg) + '°';
    }
    if (typeof s.neck_yaw_deg === 'number') {
      if (sNeckY) sNeckY.value = Math.round(s.neck_yaw_deg);
      if (sNeckYV) sNeckYV.innerText = (s.neck_yaw_deg > 0 ? '+' : '') + Math.round(s.neck_yaw_deg) + '°';
    }
    if (typeof s.neck_roll_deg === 'number') {
      if (sNeckR) sNeckR.value = Math.round(s.neck_roll_deg);
      if (sNeckRV) sNeckRV.innerText = (s.neck_roll_deg > 0 ? '+' : '') + Math.round(s.neck_roll_deg) + '°';
    }
  }

  // Update persistent Hardware Fault Banner
  const faultBanner = document.getElementById('fault-banner');
  if (faultBanner) {
    if (s.fault_latched || window.forceFaultBanner || (s.warning && s.warning.includes('HARDWARE FAULT'))) {
      faultBanner.style.display = 'flex';
      const faultText = document.getElementById('fault-banner-text');
      if (faultText) faultText.innerText = window.forceFaultText || (s.fault_reason ? `FAULT: ${s.fault_reason}` : (s.warning || 'HARDWARE FAULT LATCHED'));
    } else {
      faultBanner.style.display = 'none';
    }
  }

  // Update Sync / Ready / Engaged status badge (shows reasons when not moving)
  const syncBadge = document.getElementById('sync-badge');
  if (syncBadge) {
    let st = s.sync_state || (s.fault_latched ? 'FAULT' : 'READY');
    let sub = '';
    const rxSt = s.orin_rx_state;
    if (s.fault_latched || st === 'FAULT') { st = 'FAULT'; sub = ` (${s.fault_reason || 'Clear to resume'})`; }
    else if (st === 'SYNC') { sub = ' (WAITING FEEDBACK: no packet on UDP 5006)'; }
    else if (rxSt && rxSt !== 'RX_OK' && rxSt !== 'WAITING') { sub = ` (ROBOT: ${rxSt})`; }
    else if (st === 'READY') { sub = ' (auto-engaging...)'; }
    else if (s.feedback_age_ms > 200) { sub = ` (STALE FEEDBACK: ${s.feedback_age_ms.toFixed(0)}ms)`; }
    const gap = (s.max_sim_real_gap_rad !== undefined && st === 'ENGAGED') ? ` • gap: ${s.max_sim_real_gap_rad.toFixed(3)} rad` : '';
    syncBadge.innerText = `${st}${sub}${gap}`;
    syncBadge.className = `tag tag-${st === 'ENGAGED' ? 'ok' : (st === 'READY' ? 'info' : (st === 'FAULT' ? 'danger' : 'warn'))}`;
  }

  if (typeof updateTelemetryUI === 'function') updateTelemetryUI(s);
};

window.confirmClearFault = function() {
  if (confirm("Operator confirmation required: Clear latched hardware fault and resume?")) {
    sendCmd({cmd: "clear_fault", confirmed: true});
  }
};


// ---- Data recording status (web card + controller haptics) ----
let _recPrev = {active: false, discarded: false};
function _recPulse(times, ms) {
  const pads = window._xrGamepads || {};
  for (let k = 0; k < times; k++) {
    setTimeout(() => {
      Object.values(pads).forEach((gp) => {
        try { const h = gp && gp.hapticActuators && gp.hapticActuators[0]; if (h && h.pulse) h.pulse(0.8, ms); } catch (e) {}
      });
    }, k * (ms + 120));
  }
}
function updateRecUI(rec) {
  if (!rec) return;
  const dot = document.getElementById('rec_dot'), btn = document.getElementById('btn_rec');
  const st = document.getElementById('rec_status'), task = document.getElementById('rec_task');
  if (dot) dot.style.display = rec.active ? 'inline' : 'none';
  if (btn) btn.innerHTML = rec.active ? '&#9632; Stop REC' : '&#9679; Start REC';
  if (task && document.activeElement !== task) { task.value = rec.task; task.disabled = rec.active; }
  const sel = document.getElementById('rec_discard_reason');
  if (sel && rec.discard_reasons) {
    if (sel.options.length !== rec.discard_reasons.length) {
      sel.innerHTML = rec.discard_reasons.map((r) => '<option value="' + r + '">Discard reason: ' + r.replace(/_/g, ' ') + '</option>').join('');
    }
    if (document.activeElement !== sel) sel.value = rec.discard_reason;
  }
  if (st) {
    const rows = rec.rows || {};
    const robot = 'rows: robot ' + (rows.robot || 0) + ', teleop ' + (rows.teleop || 0) + ', images ' + (rec.images && Object.keys(rec.images).length ? Object.entries(rec.images).map(([k, v]) => k + ' ' + v).join(' / ') : (rows.images || 0))
                + (rec.active && !(rows.robot > 0) ? ' - NO ROBOT DATA yet (receiver running?)' : '');
    const head = rec.active ? ('REC ' + rec.task + ' / ' + rec.ep + '  ' + rec.secs.toFixed(0) + ' s')
                            : ('Not recording. Episodes this session: ' + rec.count + (rec.ep ? ' (last: ' + rec.ep + (rec.last_discarded ? ', DISCARDED' + (rec.last_discard_reason ? ': ' + rec.last_discard_reason.replace(/_/g, ' ') : '') : '') + ')' : ''));
    const ex = rec.export && rec.export.msg ? '\n' + rec.export.msg : '';
    st.innerText = head + '\n' + robot + (rec.msg ? '\n' + rec.msg : '') + ex;
    st.style.color = rec.msg ? '#ff8a80' : (rec.active ? '#ffcdd2' : '#9aa');
  }
  if (rec.active !== _recPrev.active) _recPulse(rec.active ? 1 : 2, 150);        // 1 pulse = started, 2 = stopped
  if (rec.last_discarded && !_recPrev.discarded) _recPulse(1, 500);              // long pulse = discard flag set
  _recPrev = {active: rec.active, discarded: Boolean(rec.last_discarded)};
}
