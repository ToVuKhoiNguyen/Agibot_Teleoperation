// Dataset viewer (/dataset): one exported LeRobot episode on a shared timeline - camera, depth, 3D robot replay,
// joint / finger charts, events, quality, and edits written back to the source episode (server: dataset_routes.py).
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const EV_COLORS = { track: "#f08c3c", idle: "#6b7787", grasp: "#3ccf7a", release: "#36b3c2", safety: "#ef5b5b", reach: "#a77bf0", ik: "#e6d34a" };
  const EV_NAMES = { track: "lệch lệnh", idle: "đứng yên", grasp: "nắm", release: "mở", safety: "an toàn", reach: "tầm với", ik: "IK" };
  const PAL = ["#4ea3ff", "#ff8a3d", "#3ccf7a", "#ef5b5b", "#b07cff", "#e6d34a", "#36c2c2"];
  const S = { ds: null, dsList: [], eps: [], ep: null, d: null, t: [], T: 0, cur: 0, playing: false, speed: 1,
              view: [0, 1], trim: null, poses: null, poseIds: [], frames: 0, charts: [] };

  // ------------------------------------------------------------------ helpers
  async function jget(url) { const r = await fetch(url); if (!r.ok) throw new Error(await r.text()); return r.json(); }
  async function jpost(url, body) {
    const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const j = await r.json().catch(() => ({})); if (!r.ok && j.ok !== false) throw new Error(r.statusText); return j;
  }
  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  function status(msg) { $("status").textContent = msg || ""; }
  function overlay(on, msg, log) { $("overlay").style.display = on ? "flex" : "none"; if (msg) $("ovMsg").textContent = msg; $("ovLog").textContent = log || ""; }
  function fmt(t) { return (t || 0).toFixed(2); }
  function frameAt(t, a = S.t) {          // last frame at or before t (a: time array, S.t unless a chart has its own)
    let lo = 0, hi = a.length - 1;
    if (!a.length) return 0; if (t <= a[0]) return 0; if (t >= a[hi]) return hi;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (a[m] <= t) lo = m; else hi = m; }
    return lo;
  }
  async function waitJob(id, msg) {
    overlay(true, msg);
    for (;;) {
      if (S.live) { overlay(false); return {}; }
      const j = await jget("/api/ds/job?id=" + encodeURIComponent(id));
      overlay(true, msg, (j.log || []).slice(-12).join("\n"));
      if (j.state === "done") { overlay(false); return j; }
      if (j.state === "error" || j.state === "unknown") { overlay(false); throw new Error((j.log || []).slice(-6).join("\n") || "job failed"); }
      await sleep(700);
    }
  }

  // ------------------------------------------------------------------ datasets / episodes
  async function loadDatasets(select) {
    const r = await jget("/api/ds/list");
    S.dsList = r.datasets;
    const sel = $("dsSel"); sel.innerHTML = "";
    if (!r.datasets.length) { sel.innerHTML = "<option>(chưa có dataset trong lerobot_datasets/)</option>"; $("eplist").textContent = "Xuất dataset bằng tools/export_lerobot.py --out lerobot_datasets/<tên>."; return; }
    r.datasets.sort((a, b) => b.mtime - a.mtime).forEach((d) => {
      const o = document.createElement("option"); o.value = d.name;
      o.textContent = d.complete === false ? `${d.name}  (chưa xuất xong)` : `${d.name}  (${d.episodes} ep, ${d.frames} khung, ${d.fps} fps)`; sel.appendChild(o);
    });
    if (!r.lerobot_env) status("thiếu venv_lerobot: không giải mã được video");
    sel.value = select && r.datasets.some((d) => d.name === select) ? select : r.datasets[0].name;
    await openDataset(sel.value);
  }

  async function openDataset(name) {
    S.ds = name; S.ep = null;
    const info = S.dsList.find((d) => d.name === name) || {};
    $("dsInfo").textContent = `camera: ${(info.cameras || []).join(", ") || "—"}${info.has_sources ? "" : " · không có nguồn (xuất trước 17/11): chỉ xem"}`;
    let r = await jget("/api/ds/episodes?ds=" + encodeURIComponent(name));
    if (r.state === "incomplete") { S.eps = []; renderEpisodes(); $("eplist").textContent = r.msg + " Chờ xuất xong rồi tải lại trang."; return; }
    if (r.state === "building") { await waitJob(r.job, "Đang đọc danh sách episode…"); r = await jget("/api/ds/episodes?ds=" + encodeURIComponent(name)); }
    S.eps = r.episodes; S.fps = r.fps;
    renderEpisodes();
    if (S.eps.length && !S.live) openEpisode(S.eps[0].index);
  }

  function renderEpisodes() {
    const box = $("eplist"); box.innerHTML = "";
    S.eps.forEach((e) => {
      const div = document.createElement("div"); div.className = "ep" + (e.index === S.ep ? " sel" : "");
      const tags = [];
      if (e.discarded) tags.push('<span class="tag bad">bỏ</span>');
      if (e.edited_trim) tags.push('<span class="tag warn">cắt</span>');
      if (e.verdict) tags.push(`<span class="tag ${e.verdict === "OK" ? "ok" : e.verdict === "BAD" ? "bad" : "warn"}">${e.verdict}</span>`);
      if (e.cached) tags.push('<span class="tag" title="đã giải mã">cache</span>');
      if (e.source_missing) tags.push('<span class="tag warn" title="episode gốc không còn">gốc?</span>');
      div.innerHTML = `<div class="l1"><b>#${e.index}</b><span class="muted">${(e.length / (S.fps || 30)).toFixed(1)} s</span><span>${tags.join(" ")}</span></div>
                       <div class="l2" title="${(e.instruction || (e.tasks || []).join(", ") || "").replace(/"/g, "&quot;")}">${e.instruction || (e.tasks || []).join(", ") || "—"}</div>`;
      div.onclick = () => openEpisode(e.index);
      box.appendChild(div);
    });
  }

  async function openEpisode(ep) {
    pause();
    S.ep = ep; renderEpisodes();
    if (S.live) return;                       // live view owns charts / 3D / cameras: a replay must not overwrite them
    try {
      let r = await jpost("/api/ds/prepare", { ds: S.ds, ep });
      if (S.live) return;
      if (r.state === "building") await waitJob(r.job, `Đang giải mã episode #${ep} (chạy nền, ưu tiên thấp)…`);
      if (S.live) { overlay(false); return; }
      overlay(true, "Đang tải dữ liệu…");
      const d = await jget(`/api/ds/episode?ds=${encodeURIComponent(S.ds)}&ep=${ep}`);
      if (S.live) { overlay(false); return; }
      S.d = d; S.t = d.t.map((x) => x - d.t[0]); S.T = S.t[S.t.length - 1] || 0; S.frames = d.frames;
      S.cuts = []; S.cutSel = -1; S.split = null; S.sel = null; S.view = [0, S.T]; S.trim = d.source && d.source.trim ? d.source.trim.slice() : null;
      const e = S.eps.find((x) => x.index === ep); if (e && d.quality) { e.verdict = d.quality.verdict; e.cached = true; renderEpisodes(); }
      buildCharts(); renderEvents(); renderQuality(); renderEdit();
      overlay(true, "Đang tính tư thế robot 3D…");
      await loadPoses();
      overlay(false);
      if (S.live) return;
      preloadImages();
      seek(0);
    } catch (err) { overlay(false); status("Lỗi: " + err.message); console.error(err); }
  }

  // ------------------------------------------------------------------ images
  function camUrl(c, k) { return `/api/ds/frame/${encodeURIComponent(S.ds)}/${S.ep}/${c.dir}/${String(k).padStart(6, "0")}.jpg`; }
  let preloadToken = 0;
  async function preloadImages() {
    const tok = ++preloadToken;
    for (let k = 0; k < S.frames; k += 1) {
      if (tok !== preloadToken) return;
      for (const c of S.d.cams) { const im = new Image(); im.src = camUrl(c, k); }
      if (k % 20 === 19) await sleep(30);
    }
  }
  function showImages(k) {
    if (S.live) return;
    const col = S.d.cams.find((c) => !c.depth), dep = S.d.cams.find((c) => c.depth);
    if (col) $("imgColor").src = camUrl(col, k); else $("imgColor").removeAttribute("src");
    if (dep) $("imgDepth").src = camUrl(dep, k); else $("imgDepth").removeAttribute("src");
  }

  // ------------------------------------------------------------------ 3D robot
  let R3 = null;
  function init3D() {
    const host = $("v3d");
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    host.appendChild(renderer.domElement);
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0x10161e);
    const cam = new THREE.PerspectiveCamera(45, 16 / 9, 0.05, 20); cam.up.set(0, 0, 1); cam.position.set(1.55, -1.1, 1.3);
    const ctl = new THREE.OrbitControls(cam, renderer.domElement); ctl.target.set(0.1, 0, 0.82); ctl.update();
    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dl = new THREE.DirectionalLight(0xffffff, 0.8); dl.position.set(2, -2, 3); scene.add(dl);
    const grid = new THREE.GridHelper(3, 30, 0x2f6fae, 0x223048); grid.rotation.x = Math.PI / 2; grid.position.z = -0.057; scene.add(grid);
    const group = new THREE.Group(); scene.add(group);
    R3 = { renderer, scene, cam, ctl, group, meshes: {}, loader: new THREE.STLLoader(), geoCache: {}, dirty: true };
    ctl.addEventListener("change", () => (R3.dirty = true));
    const resize = () => { const w = host.clientWidth, h = host.clientHeight; renderer.setSize(w, h, false); cam.aspect = w / Math.max(1, h); cam.updateProjectionMatrix(); R3.dirty = true; };
    new ResizeObserver(resize).observe(host); resize();
    (function loop() { requestAnimationFrame(loop); ctl.update(); if (R3.dirty) { renderer.render(scene, cam); R3.dirty = false; } })();
  }
  async function loadManifest() {
    const man = await jget("/api/ds/manifest");
    (man.geoms || []).forEach((g) => {
      const gid = parseInt(g.geom_id, 10);
      const c = g.rgba || [0.7, 0.7, 0.7, 1];
      const mat = new THREE.MeshLambertMaterial({ color: new THREE.Color(c[0], c[1], c[2]), transparent: c[3] < 1, opacity: c[3] });
      const m = new THREE.Mesh(new THREE.BufferGeometry(), mat); m.frustumCulled = false;
      if (g.pos) m.position.set(g.pos[0], g.pos[1], g.pos[2]);
      if (g.quat) m.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
      R3.meshes[gid] = m; R3.group.add(m);
      const name = g.mesh;
      if (R3.geoCache[name]) { m.geometry = R3.geoCache[name]; return; }
      R3.loader.load(`/web_meshes/${name}`, (geo) => { geo.computeVertexNormals(); R3.geoCache[name] = geo;
        Object.values(R3.meshes).forEach((mm) => { if (mm.userData.mesh === name) mm.geometry = geo; }); R3.dirty = true; });
      m.userData.mesh = name;
    });
  }
  async function loadPoses() {
    const r = await fetch(`/api/ds/poses?ds=${encodeURIComponent(S.ds)}&ep=${S.ep}`);
    if (!r.ok) { S.poses = null; return; }
    const buf = await r.arrayBuffer(); const dv = new DataView(buf);
    const G = dv.getUint32(0, true), N = dv.getUint32(4, true);
    S.poseIds = Array.from(new Int32Array(buf, 8, G));
    S.poses = new Float32Array(buf, 8 + 4 * G, N * G * 7);
    S.poseN = N;
  }
  function show3D(k) {
    if (!S.poses || !R3 || S.live) return;
    const G = S.poseIds.length; k = Math.min(k, S.poseN - 1); const base = k * G * 7;
    for (let i = 0; i < G; i++) {
      const m = R3.meshes[S.poseIds[i]]; if (!m) continue; const o = base + i * 7, p = S.poses;
      m.position.set(p[o], p[o + 1], p[o + 2]); m.quaternion.set(p[o + 3], p[o + 4], p[o + 5], p[o + 6]);
    }
    R3.dirty = true;
  }

  // ------------------------------------------------------------------ charts
  function col(arr, j) { return arr ? arr.map((r) => r[j]) : null; }
  function buildCharts() {
    const box = $("charts"); box.innerHTML = ""; S.charts = [];
    const F = S.d.features, st = F["observation.state"], rc = F["robot.cmd"], ac = rc || F["action"];
    const cmdName = rc ? "lệnh Orin" : "action";
    const mk = (title, series, side) => {
      const wrap = document.createElement("div"); wrap.className = "chart card";
      wrap.innerHTML = `<div class="chead"><span class="ctitle">${title}</span><div class="legend"></div></div><canvas></canvas>`;
      box.appendChild(wrap);
      const ch = { canvas: wrap.querySelector("canvas"), series, title, side };
      const lg = wrap.querySelector(".legend");
      series.forEach((s) => { if (s.legend === false) return; const sp = document.createElement("span"); sp.textContent = s.name; sp.style.borderColor = s.color;
        sp.onclick = () => { s.hidden = !s.hidden; (s.pair || []).forEach((q) => (q.hidden = s.hidden)); sp.classList.toggle("off", s.hidden); drawCharts(); }; lg.appendChild(sp); });
      attachChartMouse(ch); S.charts.push(ch);
    };
    for (const [side, off] of [["Tay trái", 0], ["Tay phải", 7]]) {
      const series = [];
      for (let j = 0; j < 7; j++) {
        const meas = { name: `J${j + 1}`, data: col(st, off + j), color: PAL[j] };
        const cmd = { name: `J${j + 1} lệnh`, data: col(ac, off + j), color: PAL[j], dash: true, legend: false };
        meas.pair = [cmd]; series.push(meas, cmd);
      }
      mk(`${side} (rad) – liền: đo, đứt: ${cmdName}`, series, off ? "R" : "L");
    }
    if (st && st[0].length >= 26) {
      const fn = ["cái x", "cái g", "trỏ", "giữa", "áp út", "út"], series = [];
      for (let j = 0; j < 12; j++) series.push({ name: `${j < 6 ? "T" : "P"} ${fn[j % 6]}`, data: col(st, 14 + j), color: PAL[j % 6], dash: j >= 6 });
      mk("Ngón tay (raw) – liền: trái, đứt: phải", series);
    }
    const ee = F["observation.ee_pose"];
    if (ee) {
      const sp = (o) => ee.map((r, k) => { if (!k) return 0; const p = ee[k - 1], dt = Math.max(1e-3, S.t[k] - S.t[k - 1]);
        return Math.hypot(r[o] - p[o], r[o + 1] - p[o + 1], r[o + 2] - p[o + 2]) / dt; });
      const series = [{ name: "tay trái", data: smooth(sp(0), 5), color: PAL[0] }, { name: "tay phải", data: smooth(sp(7), 5), color: PAL[1] }];
      const ae = F["action.ee_pose"];
      if (ae && ae[0].length >= 16) { series.push({ name: "nắm trái (0–1)", data: col(ae, 14), color: PAL[2], dash: true }); series.push({ name: "nắm phải (0–1)", data: col(ae, 15), color: PAL[3], dash: true }); }
      mk("Tốc độ bàn tay (m/s), mức nắm", series);
    }
    drawCharts();
  }
  function smooth(a, n) { const out = a.slice(); for (let i = 0; i < a.length; i++) { let s = 0, c = 0; for (let k = -n; k <= n; k++) { const v = a[i + k]; if (v !== undefined) { s += v; c++; } } out[i] = s / c; } return out; }
  // Hover tooltip: why a coloured band / lane mark exists (event label + reason from the server).
  let tipEl = null;
  function evAt(t, tol) {              // every kind incl. idle (grey), grasp (green), release (cyan)
    return S.d ? S.d.events.filter((e) => t >= e.t0 - tol && t <= Math.max(e.t1, e.t0) + tol) : [];
  }
  function showTip(ev, list, tAt) {
    if (!tipEl) {
      tipEl = document.createElement("div");
      tipEl.style.cssText = "position:fixed;z-index:50;max-width:340px;padding:7px 10px;background:#1b2330;color:#dbe4ef;border:1px solid #3a4a60;border-radius:6px;font:12px/1.4 system-ui;pointer-events:none;box-shadow:0 4px 14px rgba(0,0,0,.5)";
      document.body.appendChild(tipEl);
    }
    if (!list.length) { tipEl.style.display = "none"; return; }
    const esc = (x) => String(x).replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c]));
    tipEl.innerHTML = list.map((e) => `<div style="margin-bottom:4px"><b style="color:${EV_COLORS[e.kind] || "#999"}">${esc(EV_NAMES[e.kind] || e.kind)}</b> `
      + `<span style="color:#8796a8">${e.t0.toFixed(1)}–${e.t1.toFixed(1)} s</span><br>${esc(e.label)}${e.why ? `<br><span style="color:#9fb0c4">${esc(e.why)}</span>` : ""}</div>`).join("");
    tipEl.style.display = "block";
    const w = tipEl.offsetWidth, h = tipEl.offsetHeight;
    tipEl.style.left = Math.min(innerWidth - w - 8, ev.clientX + 14) + "px";
    tipEl.style.top = Math.min(innerHeight - h - 8, ev.clientY + 16) + "px";
  }
  function hideTip() { if (tipEl) tipEl.style.display = "none"; }
  function attachChartMouse(ch) {
    const cv = ch.canvas;
    const tOf = (ev) => { const r = cv.getBoundingClientRect(); const x = (ev.clientX - r.left) / r.width; return S.view[0] + x * (S.view[1] - S.view[0]); };
    cv.addEventListener("click", (ev) => seek(tOf(ev)));
    cv.addEventListener("mousemove", (ev) => {
      const t = tOf(ev), L = 44, r = cv.getBoundingClientRect();
      if (ev.clientX - r.left < L) { hideTip(); return; }
      const tol = 4 * (S.view[1] - S.view[0]) / Math.max(1, r.width);          // ~4 px, so point events (grasp / release) can be hit
      const list = evAt(t, tol).filter((e) => !(e.kind === "track" && (!ch.side || e.label.indexOf(" " + ch.side + " J") < 0)));
      showTip(ev, list, t);
    });
    cv.addEventListener("mouseleave", hideTip);
    cv.addEventListener("dblclick", () => { S.view = [0, S.T]; drawCharts(); drawTimeline(); });
    cv.addEventListener("wheel", (ev) => { ev.preventDefault(); const c = tOf(ev), f = ev.deltaY > 0 ? 1.25 : 0.8;
      let a = c - (c - S.view[0]) * f, b = c + (S.view[1] - c) * f; a = Math.max(0, a); b = Math.min(S.T, b);
      if (b - a > 0.2) { S.view = [a, b]; drawCharts(); drawTimeline(); } }, { passive: false });
  }
  function drawCharts() {
    for (const ch of S.charts) {
      const cv = ch.canvas, dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
      cv.width = W * dpr; cv.height = H * dpr; const g = cv.getContext("2d"); g.scale(dpr, dpr);
      g.clearRect(0, 0, W, H);
      const TT = ch.t || S.t;                                   // live view: each chart has its own growing time array
      const [v0, v1] = S.view, i0 = frameAt(v0, TT), i1 = Math.min(TT.length - 1, frameAt(v1, TT) + 1);
      let lo = Infinity, hi = -Infinity;
      for (const s of ch.series) { if (s.hidden || !s.data) continue; for (let i = i0; i <= i1; i++) { const v = s.data[i]; if (v < lo) lo = v; if (v > hi) hi = v; } }
      if (!isFinite(lo)) { lo = 0; hi = 1; } if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
      const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
      const L = 44, Rm = 6, T = 6, B = 16;
      const X = (t) => L + (t - v0) / (v1 - v0) * (W - L - Rm), Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (H - T - B);
      // trim shading + event bands
      if (S.trim) { g.fillStyle = "rgba(0,0,0,.35)"; g.fillRect(L, T, Math.max(0, X(S.trim[0]) - L), H - T - B); g.fillRect(X(S.trim[1]), T, Math.max(0, W - Rm - X(S.trim[1])), H - T - B); }
      for (const e of S.d.events) {
        if (e.t1 > e.t0) {                                      // bands: idle (grey), IK (yellow), safety, reach, track (only on its own joint chart)
          if (e.kind === "track" && (!ch.side || e.label.indexOf(" " + ch.side + " J") < 0)) continue;
          g.fillStyle = EV_COLORS[e.kind] + (e.kind === "idle" ? "30" : "2e"); g.fillRect(X(e.t0), T, Math.max(1, X(e.t1) - X(e.t0)), H - T - B);
        } else if (e.t0 >= v0 && e.t0 <= v1) {                  // point events (grasp / release): vertical line + marker on the top edge
          const x = X(e.t0); g.fillStyle = EV_COLORS[e.kind] || "#999"; g.fillRect(x - 0.75, T, 1.5, H - T - B); g.fillRect(x - 3, T, 6, 4);
        }
      }
      (S.cuts || []).forEach((c, i) => {                        // reviewed idle cuts: red overlay, selected one brighter
        g.fillStyle = i === S.cutSel ? "rgba(255,77,77,.38)" : "rgba(255,77,77,.20)"; g.fillRect(X(c.a), T, Math.max(1, X(c.b) - X(c.a)), H - T - B);
        g.fillStyle = "#ff4d4d"; g.fillRect(X(c.a) - 0.5, T, 1, H - T - B); g.fillRect(X(c.b) - 0.5, T, 1, H - T - B);
      });
      g.strokeStyle = "#2b3746"; g.fillStyle = "#8796a8"; g.font = "10px system-ui"; g.lineWidth = 1;
      for (let k = 0; k <= 4; k++) { const v = lo + (hi - lo) * k / 4, y = Y(v); g.beginPath(); g.moveTo(L, y); g.lineTo(W - Rm, y); g.stroke(); g.fillText(v.toFixed(hi - lo < 2 ? 2 : hi - lo < 20 ? 1 : 0), 2, y + 3); }
      const step = niceStep((v1 - v0) / 8);
      for (let t = Math.ceil(v0 / step) * step; t <= v1; t += step) { const x = X(t); g.beginPath(); g.moveTo(x, T); g.lineTo(x, H - B); g.stroke(); g.fillText(t.toFixed(step < 1 ? 1 : 0) + "s", x + 2, H - 4); }
      for (const s of ch.series) {
        if (s.hidden || !s.data) continue;
        g.strokeStyle = s.color; g.lineWidth = s.dash ? 1 : 1.4; g.setLineDash(s.dash ? [4, 3] : []); g.beginPath();
        const stride = Math.max(1, Math.floor((i1 - i0) / (W * 1.5)));
        for (let i = i0; i <= i1; i += stride) { const x = X(TT[i]), y = Y(s.data[i]); if (i === i0) g.moveTo(x, y); else g.lineTo(x, y); }
        g.stroke();
      }
      g.setLineDash([]); g.strokeStyle = "#ff4d4d"; g.lineWidth = 1.5; const xc = X(S.cur); g.beginPath(); g.moveTo(xc, T); g.lineTo(xc, H - B); g.stroke();
    }
  }
  function niceStep(x) { const p = Math.pow(10, Math.floor(Math.log10(Math.max(x, 1e-6)))); const n = x / p; return (n < 1.5 ? 1 : n < 3.5 ? 2 : n < 7.5 ? 5 : 10) * p; }

  // ------------------------------------------------------------------ timeline
  function drawTimeline() {
    const cv = $("tlbar"), dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
    cv.width = W * dpr; cv.height = H * dpr; const g = cv.getContext("2d"); g.scale(dpr, dpr);
    g.clearRect(0, 0, W, H); if (!S.T) return;
    const X = (t) => (t / S.T) * W;
    g.fillStyle = "rgba(58,160,255,.12)"; g.fillRect(X(S.view[0]), 0, Math.max(2, X(S.view[1]) - X(S.view[0])), H);
    const tr = S.trim || [0, S.T], HW = 9;
    if (S.trim) { g.fillStyle = "rgba(0,0,0,.55)"; g.fillRect(0, 0, X(tr[0]), H); g.fillRect(X(tr[1]), 0, W - X(tr[1]), H); }
    g.fillStyle = "#f0b43c"; g.fillRect(X(tr[0]), 0, X(tr[1]) - X(tr[0]), 2); g.fillRect(X(tr[0]), H - 2, X(tr[1]) - X(tr[0]), 2);
    for (const [x, side] of [[X(tr[0]) - HW, 0], [X(tr[1]), 1]]) {
      g.fillStyle = "#f0b43c"; g.beginPath(); g.roundRect(Math.max(0, Math.min(W - HW, x)), 0, HW, H, side ? [0, 5, 5, 0] : [5, 0, 0, 5]); g.fill();
      g.fillStyle = "#3a2d08"; const gx = Math.max(0, Math.min(W - HW, x)) + HW / 2; g.fillRect(gx - 0.5, H / 2 - 7, 1.5, 14);
    }
    const lanes = ["safety", "track", "reach", "ik", "idle", "grasp", "release"];
    for (const e of S.d.events) {
      const lane = lanes.indexOf(e.kind), y = 3 + (lane % 4) * 7; g.fillStyle = EV_COLORS[e.kind] || "#999";
      if (e.t1 > e.t0) g.fillRect(X(e.t0), y, Math.max(2, X(e.t1) - X(e.t0)), 5); else g.fillRect(X(e.t0) - 1.5, 0, 3, H);
    }
    (S.cuts || []).forEach((c, i) => {                          // idle cuts waiting for review: red block on top of the lanes
      g.fillStyle = i === S.cutSel ? "rgba(255,77,77,.55)" : "rgba(255,77,77,.32)"; g.fillRect(X(c.a), 0, Math.max(2, X(c.b) - X(c.a)), H);
      g.fillStyle = "#ff4d4d"; g.fillRect(X(c.a), 0, 1.5, H); g.fillRect(X(c.b) - 1.5, 0, 1.5, H);
    });
    if (S.split != null) {
      const a = S.sel === "L" ? tr[0] : S.sel === "R" ? S.split : null, b = S.sel === "L" ? S.split : tr[1];
      if (a != null) { g.fillStyle = "rgba(255,77,77,.35)"; g.fillRect(X(a), 0, X(b) - X(a), H); }
      g.fillStyle = "#fff"; g.fillRect(X(S.split) - 1, 0, 2, H);
    }
    g.fillStyle = "#ff4d4d"; g.fillRect(X(S.cur) - 1, 0, 2, H);
  }
  function seek(t) {
    if (S.live) return;           // live view: the playhead is "now", driven by dataset_live.js
    S.cur = Math.max(0, Math.min(S.T, t)); const k = frameAt(S.cur);
    if (S.d) { showImages(k); show3D(k); }
    if (S.cur < S.view[0] || S.cur > S.view[1]) { const w = S.view[1] - S.view[0]; S.view = [Math.max(0, Math.min(S.T - w, S.cur - w / 2)), 0]; S.view[1] = S.view[0] + w; }
    $("tlabel").textContent = `${fmt(S.cur)} / ${fmt(S.T)} s · #${k}`;
    $("capColor").textContent = `Camera màu · ${fmt(S.cur)} s`;
    drawTimeline(); drawCharts(); markEvent();
  }
  let lastWall = 0;
  function tick(now) {
    if (!S.playing) return;
    const dt = lastWall ? (now - lastWall) / 1000 : 0; lastWall = now;
    let t = S.cur + dt * S.speed;
    const end = S.trim ? S.trim[1] : S.T;
    if (t >= end) { t = end; pause(); }
    seek(t); requestAnimationFrame(tick);
  }
  function play() { if (!S.d || S.live) return; if (S.cur >= (S.trim ? S.trim[1] : S.T) - 1e-3) seek(S.trim ? S.trim[0] : 0); S.playing = true; lastWall = 0; $("btnPlay").textContent = "Dừng"; requestAnimationFrame(tick); }
  function pause() { S.playing = false; $("btnPlay").textContent = "Chạy"; }
  function stepFrames(n) { pause(); const k = Math.max(0, Math.min(S.t.length - 1, frameAt(S.cur) + n)); seek(S.t[k]); }

  // ------------------------------------------------------------------ side panels
  function renderEvents() {
    const box = $("events"); box.innerHTML = "";
    if (!S.d.events.length) { box.textContent = "Không có sự kiện."; return; }
    const kinds = [...new Set(S.d.events.map((e) => e.kind))];
    const legend = document.createElement("div"); legend.className = "row";
    legend.innerHTML = kinds.map((k) => `<span class="tag"><span class="dot" style="display:inline-block;background:${EV_COLORS[k]};margin:0 4px 0 0"></span>${EV_NAMES[k] || k}</span>`).join(" ");
    box.appendChild(legend);
    S.d.events.forEach((e, i) => {
      const div = document.createElement("div"); div.className = "ev"; div.dataset.i = i;
      div.innerHTML = `<span class="dot" style="background:${EV_COLORS[e.kind] || "#999"}"></span><span style="min-width:92px" class="muted">${fmt(e.t0)}${e.t1 > e.t0 ? "–" + fmt(e.t1) : ""} s</span><span>${e.label}</span>`;
      div.onclick = () => { pause(); seek(e.t0); };
      box.appendChild(div);
    });
  }
  function markEvent() {
    const rows = $("events").querySelectorAll(".ev");
    rows.forEach((r) => { const e = S.d.events[+r.dataset.i]; r.style.background = (S.cur >= e.t0 - 0.05 && S.cur <= Math.max(e.t1, e.t0 + 0.3)) ? "#243041" : ""; });
  }
  function renderQuality() {
    const q = S.d.quality, s = S.d.source, box = $("quality");
    let h = `<div class="row"><label>Episode</label><span>#${S.d.episode} · ${S.frames} khung · ${S.d.fps} fps · ${fmt(S.T)} s</span></div>
             <div class="row"><label>Câu lệnh</label><span>${S.d.task || "—"}</span></div>`;
    if (s.available) {
      h += `<div class="row"><label>Nguồn</label><span class="muted">${s.path}</span></div>`;
      if (q) {
        h += `<div class="row"><label>Kiểm tra</label><span class="tag ${q.verdict === "OK" ? "ok" : q.verdict === "BAD" ? "bad" : "warn"}">${q.verdict}</span></div>`;
        q.bad.forEach((x) => (h += `<div style="color:var(--bad)">! ${x}</div>`)); q.warn.forEach((x) => (h += `<div style="color:var(--warn)">- ${x}</div>`));
      }
    } else if (s.missing) h += `<div class="row"><label>Nguồn</label><span style="color:var(--warn)">không còn trên máy: ${s.missing}</span></div><div class="muted">Sự kiện an toàn / kiểm tra chỉ có khi còn episode gốc.</div>`;
    else h += `<div class="muted">Dataset này xuất trước khi ghi nguồn (17/11): chỉ có sự kiện tính từ dataset. Xuất lại từ recordings/ để có sự kiện an toàn, kiểm tra và sửa.</div>`;
    box.innerHTML = h;
  }
  function renderEdit() {
    const s = S.d.source, box = $("edit");
    if (!s.available) { box.innerHTML = '<div class="muted">Không sửa được: không có episode gốc trên máy.</div>'; return; }
    box.innerHTML = `
      <div class="row"><label>Bỏ episode</label><input type="checkbox" id="edDisc" ${s.discarded ? "checked" : ""}><span class="muted">${s.discard_reason || ""}</span></div>
      <div class="row"><label>Task</label><input type="text" id="edTask" value="${(s.task || "").replace(/"/g, "&quot;")}"></div>
      <div class="row"><label>Câu lệnh</label><input type="text" id="edInstr" value="${(s.instruction || "").replace(/"/g, "&quot;")}"></div>
      <div class="row"><label>Cắt</label><button id="edIn" title="Điểm đầu = thời điểm hiện tại">Đầu</button><button id="edOut" title="Điểm cuối = thời điểm hiện tại">Cuối</button><button id="edClr">Bỏ</button></div>
      <div class="row"><label></label><span id="edTrim" class="muted"></span></div>
      <div class="row"><label>Tự cắt</label><button id="edIdle" title="Đề xuất cắt đoạn đứng yên ở đầu / cuối (chưa lưu, bạn duyệt rồi bấm Lưu)">Đề xuất cắt đứng yên</button><button id="edScan" title="Quét mọi episode của bộ, chọn episode để áp dụng">Quét cả bộ</button></div>
      <div id="cutList"></div>
      <div id="idleBox"></div>
      <div class="row"><button class="primary" id="edSave">Lưu vào episode gốc</button><span id="edMsg" class="muted"></span></div>
      <hr style="border-color:var(--line)">
      <div class="row"><label>Xuất lại</label><input type="text" id="rxName" value="${S.ds}_v2"><button id="rxGo">Xuất bộ mới</button></div>
      <div class="muted" style="font-size:11px">Tạo dataset mới từ các episode gốc của bộ này (áp dụng bỏ / câu lệnh / cắt). Bộ đang xem không bị sửa.</div>
      <div id="log"></div>`;
    const showTrim = () => { $("edTrim").textContent = S.trim ? `giữ ${fmt(S.trim[0])} – ${fmt(S.trim[1])} s (${fmt(S.trim[1] - S.trim[0])} s)` : "không cắt"; drawTimeline(); drawCharts(); };
    $("edIn").onclick = () => { S.trim = [S.cur, S.trim ? Math.max(S.trim[1], S.cur + 0.1) : S.T]; showTrim(); };
    $("edOut").onclick = () => { S.trim = [S.trim ? Math.min(S.trim[0], S.cur - 0.1) : 0, S.cur]; if (S.trim[0] < 0) S.trim[0] = 0; showTrim(); };
    $("edClr").onclick = () => { S.trim = null; showTrim(); };
    const cutRedraw = () => { renderCutList(); drawTimeline(); drawCharts(); };
    function renderCutList() {
      const box = $("cutList"); if (!box) return;
      if (!S.cuts.length) { box.innerHTML = `<div class="muted">Chưa có đoạn cắt nào đang chờ duyệt.${S.d.source.n_cuts ? ` Episode gốc đã lưu ${S.d.source.n_cuts} đoạn cắt (có hiệu lực khi xuất bộ mới).` : ""}</div>`; return; }
      const tot = S.cuts.reduce((x, c) => x + (c.b - c.a), 0);
      box.innerHTML = `<div class="muted">${S.cuts.length} đoạn đứng yên đề xuất cắt, tổng ${fmt(tot)} s. Bấm "Xem" để xem, "Xoá" để giữ lại đoạn đó.</div>` + S.cuts.map((c, i) =>
        `<div class="row" style="${i === S.cutSel ? "background:#243041" : ""}"><span style="flex:1">#${i + 1} (${c.where}) ${fmt(c.a)} – ${fmt(c.b)} s, ${fmt(c.b - c.a)} s${c.saved ? " · đã lưu" : ""}</span>`
        + `<button class="cutView" data-i="${i}">Xem</button><button class="cutDel" data-i="${i}">Xoá</button></div>`).join("")
        + '<div class="row"><button class="primary" id="cutSave">Cắt các đoạn này</button><button id="cutReset" title="Bỏ mọi đoạn cắt đã lưu trong episode gốc">Bỏ cắt đã lưu</button></div>';
      box.querySelectorAll(".cutView").forEach((bt) => (bt.onclick = () => {
        const i = +bt.dataset.i, c = S.cuts[i]; S.cutSel = i; pause();
        S.view = [Math.max(0, c.a - 2), Math.min(S.T, c.b + 2)]; seek(c.a); cutRedraw();
      }));
      box.querySelectorAll(".cutDel").forEach((bt) => (bt.onclick = () => { S.cuts.splice(+bt.dataset.i, 1); S.cutSel = -1; cutRedraw(); }));
      $("cutSave").onclick = async () => {
        const r = await jpost("/api/ds/edit", { ds: S.ds, ep: S.ep, cuts: S.cuts.map((c) => [c.a, c.b]) });
        $("edMsg").textContent = r.ok ? `đã lưu ${S.cuts.length} đoạn cắt vào ${r.source}/meta.json. Dùng "Xuất bộ mới" để áp dụng vào dataset.` : "lỗi: " + r.error;
        if (r.ok) { S.cuts.forEach((c) => (c.saved = true)); S.d.source.n_cuts = r.n_cuts; const e = S.eps.find((x) => x.index === S.ep); if (e) { e.edited_trim = true; renderEpisodes(); } cutRedraw(); }
      };
      $("cutReset").onclick = async () => {
        const r = await jpost("/api/ds/edit", { ds: S.ds, ep: S.ep, cuts_clear: true });
        $("edMsg").textContent = r.ok ? "đã bỏ mọi đoạn cắt đã lưu của episode này." : "lỗi: " + r.error;
        if (r.ok) { S.d.source.n_cuts = 0; S.cuts.forEach((c) => (c.saved = false)); cutRedraw(); }
      };
    }
    $("edIdle").onclick = () => {
      S.cuts = (S.d.idle_cuts || []).map(([a, b, where]) => ({ a, b, where })); S.cutSel = -1; cutRedraw();
      $("edMsg").textContent = S.cuts.length ? `Đề xuất ${S.cuts.length} đoạn đứng yên (giữ đệm 0,5 s ở rìa phần có chuyển động). Duyệt rồi bấm "Cắt các đoạn này".` : "Không có đoạn đứng yên đáng cắt.";
    };
    $("edScan").onclick = async () => {
      const box = $("idleBox"); box.textContent = "Đang quét...";
      const r = await fetch("/api/ds/idle_scan?ds=" + encodeURIComponent(S.ds)).then((x) => x.json()).catch(() => null);
      if (!r || !r.ok) { box.textContent = "Lỗi quét."; return; }
      const rows = r.episodes.map((e) => {
        if (!e.cached) return `<div class="muted">#${e.ep}: chưa chuẩn bị (mở episode một lần để tạo cache)</div>`;
        if (e.error) return `<div class="muted">#${e.ep}: lỗi ${e.error}</div>`;
        if (!e.editable) return `<div class="muted">#${e.ep}: không có episode gốc</div>`;
        if (!e.cuts.length) return `<div class="muted">#${e.ep}: không có đoạn đứng yên đáng cắt</div>`;
        const tot = e.cuts.reduce((x, c) => x + c[1] - c[0], 0);
        return `<label style="display:block"><input type="checkbox" class="idleCk" data-ep="${e.ep}" data-cuts='${JSON.stringify(e.cuts.map((c) => [c[0], c[1]]))}' checked> #${e.ep}: ${e.cuts.length} đoạn, bỏ ${fmt(tot)} s của ${fmt(e.T)} s</label>`;
      });
      box.innerHTML = rows.join("") + '<div class="muted">Chỉ chọn episode muốn cắt; muốn duyệt từng đoạn thì mở episode và dùng "Đề xuất cắt đứng yên".</div><div class="row"><button id="idleApply">Áp dụng đã chọn</button><span id="idleMsg" class="muted"></span></div>';
      $("idleApply").onclick = async () => {
        let ok = 0, bad = 0;
        for (const ck of box.querySelectorAll(".idleCk:checked")) {
          const rr = await jpost("/api/ds/edit", { ds: S.ds, ep: +ck.dataset.ep, cuts: JSON.parse(ck.dataset.cuts) });
          if (rr.ok) { ok++; const e = S.eps.find((x) => x.index === +ck.dataset.ep); if (e) e.edited_trim = true; } else bad++;
        }
        renderEpisodes();
        $("idleMsg").textContent = `đã lưu ${ok} episode${bad ? `, lỗi ${bad}` : ""}. Dùng "Xuất bộ mới" để áp dụng vào dataset.`;
      };
    };
    $("edSave").onclick = async () => {
      const r = await jpost("/api/ds/edit", { ds: S.ds, ep: S.ep, discarded: $("edDisc").checked, task: $("edTask").value, instruction: $("edInstr").value, trim: S.trim });
      $("edMsg").textContent = r.ok ? "đã lưu vào " + r.source + "/meta.json" : "lỗi: " + r.error;
      const e = S.eps.find((x) => x.index === S.ep); if (r.ok && e) { e.discarded = $("edDisc").checked; e.instruction = $("edInstr").value; e.edited_trim = !!S.trim; renderEpisodes(); }
    };
    $("rxGo").onclick = async () => {
      const name = $("rxName").value.trim();
      const r = await jpost("/api/ds/reexport", { ds: S.ds, new_name: name });
      if (!r.ok) { $("log").textContent = "lỗi: " + r.error; return; }
      $("log").textContent = "đang xuất…";
      for (;;) { const j = await jget("/api/ds/job?id=" + encodeURIComponent(r.job)); $("log").textContent = (j.log || []).slice(-10).join("\n");
        if (j.state === "done") { $("log").textContent += `\nXong: lerobot_datasets/${name}`; await loadDatasets(S.ds); break; }
        if (j.state !== "running") { $("log").textContent += "\nLỗi"; break; } await sleep(1000); }
    };
    S.onTrim = showTrim; showTrim(); renderCutList();
  }

  // ------------------------------------------------------------------ wiring
  function wire() {
    document.querySelectorAll("#tabs button").forEach((b) => (b.onclick = () => {
      document.querySelectorAll("#tabs button").forEach((x) => x.classList.toggle("on", x === b));
      document.querySelectorAll(".tabpane").forEach((p) => p.classList.toggle("on", p.id === "pane-" + b.dataset.t));
    }));
    new ResizeObserver(() => { drawCharts(); drawTimeline(); }).observe($("charts"));
    const gd = $("guide"), gShow = (on) => { gd.classList.toggle("on", on); if (!on) { try { localStorage.setItem("ds_guide_seen", "1"); } catch (e) {} } };
    $("btnGuide").onclick = () => gShow(true); $("guideClose").onclick = () => gShow(false);
    gd.addEventListener("mousedown", (ev) => { if (ev.target === gd) gShow(false); });
    window.addEventListener("keydown", (ev) => { if (ev.code === "Escape" && gd.classList.contains("on")) gShow(false); });
    try { if (!localStorage.getItem("ds_guide_seen")) gd.classList.add("on"); } catch (e) {}
    $("dsSel").onchange = () => openDataset($("dsSel").value);
    $("btnRefresh").onclick = () => loadDatasets(S.ds);
    $("btnPlay").onclick = () => (S.playing ? pause() : play());
    $("btnPrev").onclick = () => stepFrames(-1); $("btnNext").onclick = () => stepFrames(1);
    $("speed").onchange = () => (S.speed = parseFloat($("speed").value));
    const tl = $("tlbar"); let drag = false;
    const tOfX = (ev) => { const r = tl.getBoundingClientRect(); return Math.max(0, Math.min(S.T, (ev.clientX - r.left) / r.width * S.T)); };
    const tlSeek = (ev) => seek(tOfX(ev));
    // CapCut-style trim handles: yellow grips at the start / end of the kept range, drag to cut head / tail
    let handle = null;
    const canEdit = () => S.d && S.d.source && S.d.source.available;
    const hitHandle = (ev) => { if (!S.T || !canEdit()) return null; const r = tl.getBoundingClientRect(), tr = S.trim || [0, S.T];
      const x0 = r.left + tr[0] / S.T * r.width, x1 = r.left + tr[1] / S.T * r.width;
      if (ev.clientX >= x0 - 11 && ev.clientX <= x0 + 3) return "in"; if (ev.clientX >= x1 - 3 && ev.clientX <= x1 + 11) return "out"; return null; };
    const moveHandle = (ev) => { const t = tOfX(ev), tr = S.trim ? S.trim.slice() : [0, S.T];
      if (handle === "in") tr[0] = Math.min(t, tr[1] - 0.1); else tr[1] = Math.max(t, tr[0] + 0.1);
      S.trim = tr; if (S.onTrim) S.onTrim(); seek(handle === "in" ? tr[0] : tr[1]); };
    const refresh = () => { if (S.onTrim) S.onTrim(); drawTimeline(); drawCharts(); updSplitUi(); };
    const updSplitUi = () => { const d = $("btnDel"); d.disabled = !(S.split != null && S.sel); d.title = S.split == null ? "Bấm Cắt trước, rồi bấm chọn đoạn muốn xoá" : S.sel ? "Xoá đoạn đang chọn (Del)" : "Bấm vào đoạn bên trái hoặc phải vạch cắt để chọn"; };
    const doCut = () => { if (!canEdit()) return; const tr = S.trim || [0, S.T];
      if (S.cur <= tr[0] + 0.05 || S.cur >= tr[1] - 0.05) { status("Đặt đầu phát vào giữa đoạn đang giữ rồi mới cắt"); return; }
      S.split = S.cur; S.sel = null; refresh(); };
    const doDelete = () => { if (S.split == null || !S.sel) return; const tr = S.trim ? S.trim.slice() : [0, S.T];
      if (S.sel === "L") tr[0] = S.split; else tr[1] = S.split;
      S.trim = tr; S.split = null; S.sel = null; seek(Math.max(tr[0], Math.min(tr[1], S.cur))); refresh(); };
    const clearSplit = () => { S.split = null; S.sel = null; refresh(); };
    $("btnCut").onclick = doCut; $("btnDel").onclick = doDelete;
    tl.addEventListener("mousedown", (ev) => { pause(); handle = hitHandle(ev); if (handle) { moveHandle(ev); return; }
      if (S.split != null) { const t = tOfX(ev); S.sel = t < S.split ? "L" : "R"; updSplitUi(); }
      drag = true; tlSeek(ev); });
    tl.addEventListener("mousemove", (ev) => { if (!drag && !handle) showTip(ev, evAt(tOfX(ev), 0.25 * S.T / Math.max(1, tl.clientWidth) * 12), 0); });
    tl.addEventListener("mouseleave", hideTip);
    tl.addEventListener("mousemove", (ev) => { if (!drag && !handle) tl.style.cursor = hitHandle(ev) ? "ew-resize" : "pointer"; });
    window.addEventListener("mousemove", (ev) => { if (handle) moveHandle(ev); else if (drag) tlSeek(ev); });
    updSplitUi();
    window.addEventListener("mouseup", () => { drag = false;
      if (handle) { handle = null; S.split = null; S.sel = null; if (S.trim && S.trim[0] <= 0.01 && S.trim[1] >= S.T - 0.01) S.trim = null; if (S.onTrim) S.onTrim(); } });
    window.addEventListener("keydown", (ev) => {
      if (ev.target.tagName === "INPUT" || ev.target.tagName === "SELECT") return;
      if (ev.code === "Space") { ev.preventDefault(); S.playing ? pause() : play(); }
      else if (ev.code === "KeyS" && canEdit()) { ev.preventDefault(); doCut(); }
      else if ((ev.code === "Delete" || ev.code === "Backspace") && S.split != null) { ev.preventDefault(); doDelete(); }
      else if (ev.code === "Escape" && S.split != null) clearSplit();
      else if ((ev.code === "KeyI" || ev.code === "KeyO") && canEdit()) { const tr = S.trim ? S.trim.slice() : [0, S.T];
        if (ev.code === "KeyI") tr[0] = Math.min(S.cur, tr[1] - 0.1); else tr[1] = Math.max(S.cur, tr[0] + 0.1);
        S.trim = tr; if (S.onTrim) S.onTrim(); drawTimeline(); drawCharts(); }
      else if (ev.code === "ArrowRight") { ev.preventDefault(); ev.shiftKey ? (pause(), seek(S.cur + 1)) : stepFrames(1); }
      else if (ev.code === "ArrowLeft") { ev.preventDefault(); ev.shiftKey ? (pause(), seek(S.cur - 1)) : stepFrames(-1); }
    });
    window.addEventListener("resize", () => { drawCharts(); drawTimeline(); });
  }

  // hooks for dataset_live.js (live view of the episode being recorded)
  window.DSV = { S, buildCharts, drawCharts, get R3() { return R3; }, status, fmt, reload: () => loadDatasets(S.ds) };

  window.addEventListener("load", async () => {
    wire(); init3D();
    try { await loadManifest(); await loadDatasets(new URLSearchParams(location.search).get("ds")); }
    catch (err) { status("Lỗi: " + err.message); console.error(err); }
  });
})();
