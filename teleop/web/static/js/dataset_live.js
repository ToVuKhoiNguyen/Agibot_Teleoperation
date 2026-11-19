// Live view (/dataset, button "Trực tiếp"): the episode being recorded, streamed over /ws/live (server: live_routes.py).
// Every robot / teleop row arrives in order with a sequence number; gaps are counted and shown, never hidden.
// Charts, robot 3D and cameras use the same code as the replay view (dataset_viewer.js, window.DSV).
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const V = window.DSV, S = V.S;
  const KEEP = 12000, TRIM = 6000;                       // rows kept per chart (120 s at 100 Hz); older ones are dropped from the live charts only
  let ws = null, retry = 0, ch = null, st = null, redraw = false, urls = { color: null, depth: null };

  function newStats() {
    return { robot: 0, tel: 0, newest: { robot: 0, tel: 0, img: 0 }, gaps: 0, missed: 0, imgs: 0, skipped: 0, expect: { robot: 1, tel: 1 },
             rate: { robot: 0, tel: 0, n0: { robot: 0, tel: 0 }, t0: performance.now() }, info: null, active: false, state: "đang kết nối", err: "" };
  }

  // ---------------------------------------------------------------- charts
  function buildLiveCharts() {
    const z = (n) => new Array(n).fill(0);
    S.d = { features: { "observation.state": [z(26)], "robot.cmd": [z(14)], "observation.ee_pose": [z(14), z(14)], "action.ee_pose": [z(16)] },
            events: [], cams: [], source: { available: false }, quality: null, frames: 0 };
    S.t = [0]; S.T = 0; S.trim = null; S.split = null; S.sel = null; S.view = [0, 10]; S.cur = 0;
    V.buildCharts();
    ch = { arm: [S.charts[0], S.charts[1]], fing: S.charts[2], spd: S.charts[3] };
    for (const c of S.charts) { c.t = []; c.series.forEach((s) => (s.data = [])); }
  }
  function trimChart(c) {
    if (c.t.length <= KEEP) return;
    c.t.splice(0, TRIM); c.series.forEach((s) => s.data.splice(0, TRIM));
  }
  function pushRobot(r) {
    const q = r.q || [], h = r.hand || [], cmd = r.cmd || [];
    for (let side = 0; side < 2; side++) {
      const c = ch.arm[side]; c.t.push(r.t);
      for (let j = 0; j < 7; j++) { c.series[2 * j].data.push(q[side * 7 + j] ?? NaN); c.series[2 * j + 1].data.push(cmd[side * 7 + j] ?? NaN); }
      trimChart(c);
    }
    ch.fing.t.push(r.t); for (let j = 0; j < 12; j++) ch.fing.series[j].data.push(h[j] ?? NaN); trimChart(ch.fing);
  }
  let prevEe = null, spdL = 0, spdR = 0;
  function pushTel(r) {
    const c = ch.spd, e = r.ee || [], pl = e[0], pr = e[1];
    if (prevEe && pl && pr && prevEe.l && prevEe.r) {
      const dt = Math.max(1e-3, r.t - prevEe.t);
      spdL += 0.35 * (Math.hypot(pl[0] - prevEe.l[0], pl[1] - prevEe.l[1], pl[2] - prevEe.l[2]) / dt - spdL);
      spdR += 0.35 * (Math.hypot(pr[0] - prevEe.r[0], pr[1] - prevEe.r[1], pr[2] - prevEe.r[2]) / dt - spdR);
    }
    if (pl && pr) prevEe = { t: r.t, l: pl, r: pr };
    c.t.push(r.t); c.series[0].data.push(spdL); c.series[1].data.push(spdR);
    c.series[2].data.push((r.clos || [])[0] ?? 0); c.series[3].data.push((r.clos || [])[1] ?? 0);
    trimChart(c);
  }
  function scheduleDraw() {
    if (redraw) return; redraw = true;
    requestAnimationFrame(() => {
      redraw = false; if (!S.live) return;
      const w = parseFloat($("liveWin").value) || 10, T = Math.max(S.T, 0.1);
      S.view = [Math.max(0, T - w), T]; S.cur = T; V.drawCharts(); renderCard();
    });
  }

  // ---------------------------------------------------------------- messages
  function onRows(m) {
    for (const r of m.robot) {
      if (r.s !== st.expect.robot) st.gaps++;
      st.expect.robot = r.s + 1; st.robot++; pushRobot(r); if (r.t > S.T) S.T = r.t;
    }
    for (const r of m.tel) {
      if (r.s !== st.expect.tel) st.gaps++;
      st.expect.tel = r.s + 1; st.tel++; pushTel(r); if (r.t > S.T) S.T = r.t;
    }
    st.missed += m.missed.robot + m.missed.tel; st.newest = m.newest; st.active = m.active;
    scheduleDraw();
  }
  function onPoses(buf) {
    const dv = new DataView(buf), n = dv.getUint32(1, true), G = S.poseIds.length; if (!n || !G) return;
    const off = 5 + 4 * n, base = (n - 1) * G * 7, p = new Float32Array(buf.slice(off));     // newest of the batch is drawn
    const R = V.R3; if (!R) return;
    for (let i = 0; i < G; i++) {
      const m = R.meshes[S.poseIds[i]]; if (!m) continue; const o = base + i * 7;
      m.position.set(p[o], p[o + 1], p[o + 2]); m.quaternion.set(p[o + 3], p[o + 4], p[o + 5], p[o + 6]);
    }
    R.dirty = true;
  }
  function onImage(buf) {
    const dv = new DataView(buf), hl = dv.getUint32(1, true);
    const h = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 5, hl)));
    const kind = h.kind === "depth" ? "depth" : "color", img = $(kind === "depth" ? "imgDepth" : "imgColor");
    const url = URL.createObjectURL(new Blob([buf.slice(5 + hl)], { type: "image/jpeg" }));
    const old = urls[kind]; img.onload = () => { if (old) URL.revokeObjectURL(old); }; urls[kind] = url; img.src = url;
    st.imgs++; if (kind === "color") $("capColor").textContent = `Camera màu · trực tiếp · ${V.fmt(h.t)} s`;
  }
  function onText(m) {
    if (m.type === "hello") S.poseIds = m.ids;
    else if (m.type === "rows") onRows(m);
    else if (m.type === "episode") startEpisode(m);
    else if (m.type === "status") { if (m.img_skipped != null) st.skipped = m.img_skipped; if (m.error) st.err = m.error; }
    else if (m.type === "idle") { st.state = "chưa ghi"; st.active = false; }
    else if (m.type === "ended") { st.info = m.info; st.active = false; }
    renderCard();
  }
  function startEpisode(m) {                        // a new episode started: empty charts, counters from zero
    const keep = { info: m.info };
    st = Object.assign(newStats(), keep); st.active = m.active; prevEe = null; spdL = spdR = 0;
    buildLiveCharts(); S.live = true; renderCard();
  }

  // ---------------------------------------------------------------- UI
  function renderCard() {
    const b = $("liveInfo"); if (!b || !st) return;
    const now = performance.now(), dt = (now - st.rate.t0) / 1000;
    if (dt >= 1) { for (const k of ["robot", "tel"]) { st.rate[k] = (st[k] - st.rate.n0[k]) / dt; st.rate.n0[k] = st[k]; } st.rate.t0 = now; }
    const i = st.info, line = (a, v, cls) => `<div class="row"><label style="min-width:70px">${a}</label><span class="${cls || ""}">${v}</span></div>`;
    const lag = (k) => Math.max(0, st.newest[k] - (st.expect[k] - 1));
    const lost = st.gaps + st.missed;
    b.innerHTML =
      line("Trạng thái", st.active ? '<b style="color:var(--bad)">ĐANG GHI</b>' : (i && i.preview ? '<b style="color:var(--ok)">XEM TRƯỚC</b> (chưa ghi)' : (i ? "đã dừng" : st.state)), "") +
      (i && !i.preview ? line("Task", i.task) + line("Episode", i.ep) + line("Câu lệnh", (i.instruction || "—").replace(/</g, "&lt;")) : "") +
      line("Thời gian", `${V.fmt(S.T)} s`) +
      line("Robot", `${st.robot} · ${st.rate.robot.toFixed(0)} Hz · trễ ${lag("robot")}`) +
      line("Teleop", `${st.tel} · ${st.rate.tel.toFixed(0)} Hz · trễ ${lag("tel")}`) +
      line("Ảnh", (i && i.preview && !st.newest.img) ? "chưa có ảnh từ robot" : `${st.imgs} / ${Math.max(st.imgs + st.skipped, st.newest.img)}${st.skipped ? ` · bỏ qua ${st.skipped}` : ""}`, st.skipped ? "tag warn" : "") +
      line("Mất", lost ? `${lost} (gap ${st.gaps}, tràn ${st.missed})` : "0", lost ? "tag bad" : "tag ok") +
      (st.err ? line("Lỗi", st.err, "tag bad") : "");
    $("liveHint").textContent = i && i.preview ? "Đang xem trực tiếp robot, chưa ghi gì. Bấm Start REC trên panel để thu: biểu đồ sẽ xoá và bắt đầu episode mới."
                              : st.active ? "Dữ liệu đang được ghi vào đĩa. Ảnh bỏ qua hiển thị (nếu có) vẫn đủ trong thư mục episode."
                                          : "Episode đã dừng. Tải lại danh sách dataset sau khi tự xuất xong để xem lại và chỉnh sửa.";
  }
  function connect() {
    if (ws) return;
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    ws = new WebSocket(`${proto}//${location.host}/ws/live`); ws.binaryType = "arraybuffer";
    ws.onopen = () => { retry = 0; st.err = ""; renderCard(); };
    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") return onText(JSON.parse(ev.data));
      const b = new Uint8Array(ev.data, 0, 1)[0]; if (b === 1) onPoses(ev.data); else if (b === 2) onImage(ev.data);
    };
    ws.onclose = () => { ws = null; if (S.live) { st.err = "mất kết nối, đang nối lại"; renderCard(); setTimeout(connect, Math.min(5000, 500 * ++retry)); } };
  }
  // ---------------------------------------------------------------- Recording Controls & Status Polling
  let statusTimer = null;
  let lastRecData = null;

  function sendLiveCmd(obj) {
    fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(obj)
    }).catch(e => console.error("Error sending command from dataset live:", e));
  }

  function toggleRec() {
    const active = Boolean(lastRecData && lastRecData.active);
    const taskInput = $("liveRecTask");
    const insInput = $("liveRecInstruction");
    if (active) {
      sendLiveCmd({ cmd: "rec_stop" });
    } else {
      sendLiveCmd({
        cmd: "rec_start",
        task: taskInput ? taskInput.value.trim() || "default" : "default",
        instruction: insInput ? insInput.value.trim() : ""
      });
    }
  }

  function discardRec() {
    const sel = $("liveRecDiscardReason");
    sendLiveCmd({ cmd: "rec_discard", reason: sel ? sel.value : undefined });
  }

  function updateLiveRecUI(rec) {
    if (!rec) return;
    lastRecData = rec;
    const dot = $("liveRecDot");
    const btn = $("liveBtnRec");
    const stEl = $("liveRecStatus");
    const taskInput = $("liveRecTask");
    const insInput = $("liveRecInstruction");
    const sel = $("liveRecDiscardReason");

    if (dot) dot.style.display = rec.active ? "inline" : "none";
    if (btn) {
      btn.textContent = rec.active ? "Stop REC" : "Start REC";
      btn.style.background = rec.active ? "#7b1111" : "#5a1010";
      btn.style.borderColor = rec.active ? "#e53935" : "#c62828";
    }
    if (taskInput && document.activeElement !== taskInput) {
      taskInput.value = rec.task || "default";
      taskInput.disabled = rec.active;
    }
    if (insInput && document.activeElement !== insInput && rec.instruction !== undefined && !rec.active) {
      if (!insInput.value) insInput.value = rec.instruction;
    }
    if (sel && rec.discard_reasons) {
      if (sel.options.length !== rec.discard_reasons.length) {
        sel.innerHTML = rec.discard_reasons.map(r => `<option value="${r}">Discard reason: ${r.replace(/_/g, " ")}</option>`).join("");
      }
      if (document.activeElement !== sel && rec.discard_reason) {
        sel.value = rec.discard_reason;
      }
    }
    if (stEl) {
      const rows = rec.rows || {};
      const robotInfo = `rows: robot ${rows.robot || 0}, teleop ${rows.teleop || 0}, images ${rec.images && Object.keys(rec.images).length ? Object.entries(rec.images).map(([k, v]) => k + " " + v).join(" / ") : (rows.images || 0)}`;
      const head = rec.active
        ? `REC ${rec.task} / ${rec.ep}  ${rec.secs ? rec.secs.toFixed(0) : "0"} s`
        : `Chưa ghi. Episode phiên này: ${rec.count || 0}${rec.ep ? ` (vừa ghi: ${rec.ep}${rec.last_discarded ? `, DISCARDED${rec.last_discard_reason ? ": " + rec.last_discard_reason.replace(/_/g, " ") : ""}` : ""})` : ""}`;
      const ex = rec.export && rec.export.msg ? "\n" + rec.export.msg : "";
      stEl.innerText = head + "\n" + robotInfo + (rec.msg ? "\n" + rec.msg : "") + ex;
      stEl.style.color = rec.msg ? "#ff8a80" : (rec.active ? "#ffcdd2" : "var(--muted)");
    }
  }

  function fetchLiveStatus() {
    if (!S.live) return;
    fetch("/api/status")
      .then(r => r.json())
      .then(data => {
        if (data && data.rec) {
          updateLiveRecUI(data.rec);
        }
      })
      .catch(() => {});
  }

  function startStatusPolling() {
    stopStatusPolling();
    fetchLiveStatus();
    statusTimer = setInterval(fetchLiveStatus, 1000);
  }

  function stopStatusPolling() {
    if (statusTimer) {
      clearInterval(statusTimer);
      statusTimer = null;
    }
  }

  function enter() {
    S.live = true; document.body.classList.add("live");
    const b = $("btnAnalyze");
    if (b) { b.classList.remove("on"); b.textContent = "Phân tích"; b.classList.add("primary"); }
    const sel = $("dsSel"), ref = $("btnRefresh"), inf = $("dsInfo");
    if (sel) sel.style.display = "none";
    if (ref) ref.style.display = "none";
    if (inf) inf.style.display = "none";
    st = newStats();
    buildLiveCharts(); renderCard(); connect();
    startStatusPolling();
  }
  function leave() {
    S.live = false; document.body.classList.remove("live");
    const b = $("btnAnalyze");
    if (b) { b.classList.add("on"); b.textContent = "Trực tiếp"; b.classList.remove("primary"); }
    const sel = $("dsSel"), ref = $("btnRefresh"), inf = $("dsInfo");
    if (sel) sel.style.display = "";
    if (ref) ref.style.display = "";
    if (inf) inf.style.display = "";
    stopStatusPolling();
    if (ws) { const w = ws; ws = null; w.close(); }
    ["color", "depth"].forEach((k) => { if (urls[k]) URL.revokeObjectURL(urls[k]); urls[k] = null; });
    S.charts = []; $("charts").innerHTML = ""; S.d = null; S.t = []; S.T = 0; $("capColor").textContent = "Camera màu";
    $("imgColor").removeAttribute("src"); $("imgDepth").removeAttribute("src"); $("tlabel").textContent = "0.00 / 0.00 s";
    V.reload();
  }
  window.addEventListener("load", () => {
    const b = $("btnAnalyze");
    if (b) b.onclick = () => (S.live ? leave() : enter());
    $("liveWin").onchange = scheduleDraw;
    const btnRec = $("liveBtnRec");
    if (btnRec) btnRec.onclick = toggleRec;
    const btnDiscard = $("liveBtnDiscard");
    if (btnDiscard) btnDiscard.onclick = discardRec;
    const selReason = $("liveRecDiscardReason");
    if (selReason) selReason.onchange = (e) => sendLiveCmd({ cmd: "set_discard_reason", reason: e.target.value });
    // Mặc định luôn vào chế độ Trực tiếp khi tải trang, trừ khi có ?replay=1 hoặc ?live=0
    const params = new URLSearchParams(location.search);
    if (params.get("live") === "0" || params.get("replay") === "1") {
      leave();
    } else {
      setTimeout(enter, 200);
    }
  });
})();
