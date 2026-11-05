function updateTelemetryUI(s) {

  // Controller Status Updates
  if (s.xr_status) {
    const xr = s.xr_status;
    const pktEl = document.getElementById('xr_pkt_cnt');
    if (pktEl) pktEl.innerText = xr.pkt_count || 0;
    const rateEl = document.getElementById('xr_rate');
    if (rateEl) rateEl.innerText = (xr.rate_hz || 0).toFixed(0) + ' Hz';
    const ageEl = document.getElementById('xr_age');
    if (ageEl) {
      const age = xr.age_ms || 0;
      ageEl.innerText = age.toFixed(0) + ' ms';
      ageEl.style.color = (age > 350) ? '#f44336' : '#4caf50';
    }

    // Left Hand
    const l = xr.left;
    if (l) {
      const clEl = document.getElementById('xr_clutch_l');
      if (clEl) {
        clEl.innerText = l.clutch_state || 'LOCKED';
        const colors = { LOCKED: '#37474f', TRANSLATE: '#1976d2', ROTATE: '#f57c00', '6D': '#388e3c' };
        clEl.style.background = colors[l.clutch_state] || '#37474f';
      }
      const grEl = document.getElementById('xr_grasp_l');
      if (grEl) grEl.innerText = (l.grasp_type || 'CYLINDER').toUpperCase();
      const barGripL = document.getElementById('xr_bar_grip_l');
      if (barGripL) barGripL.style.width = Math.round((l.grip_val || 0) * 100) + '%';
      const valGripL = document.getElementById('xr_val_grip_l');
      if (valGripL) valGripL.innerText = (typeof l.grip_val === 'number' ? l.grip_val : 0).toFixed(2);

      const barTrigL = document.getElementById('xr_bar_trig_l');
      if (barTrigL) barTrigL.style.width = Math.round((l.trigger_val || 0) * 100) + '%';
      const valTrigL = document.getElementById('xr_val_trig_l');
      if (valTrigL) valTrigL.innerText = (typeof l.trigger_val === 'number' ? l.trigger_val : 0).toFixed(2);

      const btnX = document.getElementById('xr_btn_x');
      if (btnX) btnX.classList.toggle('active', Boolean(l.btn_primary));
      const btnY = document.getElementById('xr_btn_y');
      if (btnY) btnY.classList.toggle('active', Boolean(l.btn_secondary));
      const btnMenuL = document.getElementById('xr_btn_menul');
      if (btnMenuL) btnMenuL.classList.toggle('active', Boolean(l.btn_menu));

      const precL = document.getElementById('xr_prec_l');
      if (precL) precL.style.display = l.precision ? 'inline-block' : 'none';

      const closeL = document.getElementById('xr_close_l');
      if (closeL) closeL.innerText = Math.round((l.closure || 0) * 100) + '%';
      const stickL = document.getElementById('xr_stick_l');
      if (stickL && l.thumbstick) stickL.innerText = `${l.thumbstick[0].toFixed(2)}, ${l.thumbstick[1].toFixed(2)}${l.thumb_click ? ' (CLK)' : ''}`;

      const homeBoxL = document.getElementById('xr_home_box_l');
      const homeBarL = document.getElementById('xr_home_bar_l');
      if (homeBoxL && homeBarL) {
        const p = l.home_progress || 0;
        homeBoxL.style.display = p > 0 ? 'block' : 'none';
        homeBarL.style.width = Math.round(p * 100) + '%';
      }
    }

    // Right Hand
    const r = xr.right;
    if (r) {
      const clEl = document.getElementById('xr_clutch_r');
      if (clEl) {
        clEl.innerText = r.clutch_state || 'LOCKED';
        const colors = { LOCKED: '#37474f', TRANSLATE: '#1976d2', ROTATE: '#f57c00', '6D': '#388e3c' };
        clEl.style.background = colors[r.clutch_state] || '#37474f';
      }
      const grEl = document.getElementById('xr_grasp_r');
      if (grEl) grEl.innerText = (r.grasp_type || 'CYLINDER').toUpperCase();
      const barGripR = document.getElementById('xr_bar_grip_r');
      if (barGripR) barGripR.style.width = Math.round((r.grip_val || 0) * 100) + '%';
      const valGripR = document.getElementById('xr_val_grip_r');
      if (valGripR) valGripR.innerText = (typeof r.grip_val === 'number' ? r.grip_val : 0).toFixed(2);

      const barTrigR = document.getElementById('xr_bar_trig_r');
      if (barTrigR) barTrigR.style.width = Math.round((r.trigger_val || 0) * 100) + '%';
      const valTrigR = document.getElementById('xr_val_trig_r');
      if (valTrigR) valTrigR.innerText = (typeof r.trigger_val === 'number' ? r.trigger_val : 0).toFixed(2);

      const btnA = document.getElementById('xr_btn_a');
      if (btnA) btnA.classList.toggle('active', Boolean(r.btn_primary));
      const btnB = document.getElementById('xr_btn_b');
      if (btnB) btnB.classList.toggle('active', Boolean(r.btn_secondary));
      const btnMenuR = document.getElementById('xr_btn_menur');
      if (btnMenuR) btnMenuR.classList.toggle('active', Boolean(r.btn_menu));

      const precR = document.getElementById('xr_prec_r');
      if (precR) precR.style.display = r.precision ? 'inline-block' : 'none';

      const closeR = document.getElementById('xr_close_r');
      if (closeR) closeR.innerText = Math.round((r.closure || 0) * 100) + '%';
      const stickR = document.getElementById('xr_stick_r');
      if (stickR && r.thumbstick) stickR.innerText = `${r.thumbstick[0].toFixed(2)}, ${r.thumbstick[1].toFixed(2)}${r.thumb_click ? ' (CLK)' : ''}`;

      const homeBoxR = document.getElementById('xr_home_box_r');
      const homeBarR = document.getElementById('xr_home_bar_r');
      if (homeBoxR && homeBarR) {
        const p = r.home_progress || 0;
        homeBoxR.style.display = p > 0 ? 'block' : 'none';
        homeBarR.style.width = Math.round(p * 100) + '%';
      }
    }

    // Event Log
    const logEl = document.getElementById('xr_events_log');
    if (logEl && xr.events && xr.events.length > 0) {
      logEl.innerHTML = xr.events.map(e => `<div><span style="color:#546e7a;">[${e.t}]</span> ${e.msg}</div>`).join('');
      logEl.scrollTop = logEl.scrollHeight;
    }
  }

  // Hand / Finger Test updates
  if (s.hand_status) {
    const hs = s.hand_status;
    const sideHs = hs[currentHandTab] || hs.left;
    const sweepArea = document.getElementById('sweep_status_area');
    if (sweepArea && hs.sweep_info) sweepArea.innerText = hs.sweep_info;

    updateJointSlidersUI(sideHs);
    updateMimicJointsUI(sideHs);
  }

}
