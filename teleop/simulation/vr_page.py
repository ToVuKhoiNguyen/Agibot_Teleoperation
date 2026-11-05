"""
teleop/simulation/vr_page.py
Lightweight WebXR page for Meta Quest (/vr).
Features:
- Fullscreen 3D canvas with same Three.js robot digital twin and WebSocket streaming
- No DOM sidebar or toolbar
- Centered prominent 'ENTER VR' button with passthrough (immersive-ar) fallback to immersive-vr
- Automatic session request on page load if allowed by browser
- Automatic session request on first controller trigger press ('selectstart' / pointerdown)
- Controller 6D pose streaming over /ws_panel
"""

HTML_VR = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AGIBOT A2 Ultra - Quest WebXR</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { width: 100%; height: 100%; overflow: hidden; background: #000; color: #fff; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace; }
  #canvas3d { width: 100%; height: 100%; display: block; }
  
  #vr-overlay {
    position: absolute;
    top: 0; left: 0; width: 100%; height: 100%;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    background: rgba(0, 0, 0, 0.7);
    z-index: 999;
  }
  #btn-enter-vr {
    padding: 24px 48px;
    font-size: 2rem;
    font-weight: 800;
    color: #fff;
    background: linear-gradient(135deg, #00c853 0%, #009624 100%);
    border: 3px solid #69f0ae;
    border-radius: 16px;
    cursor: pointer;
    box-shadow: 0 0 30px rgba(0, 200, 83, 0.6);
    transition: transform 0.15s, background 0.15s;
  }
  #btn-enter-vr:active {
    transform: scale(0.96);
  }
  #vr-status {
    margin-top: 18px;
    font-size: 1.1rem;
    color: #b0bec5;
    text-shadow: 0 1px 3px rgba(0,0,0,0.8);
  }
</style>
<script src="/static/js/three.min.js?v=20261105"></script>
<script src="/static/js/STLLoader.js?v=20261105"></script>
</head>
<body>
<div id="vr-overlay">
  <button id="btn-enter-vr" onclick="startXRSession('button')">ENTER VR</button>
  <div id="vr-status">Quest Controller Teleoperation • Tap button or pull trigger</div>
  <div id="vr-diag-overlay" style="margin-top:16px; padding:12px 16px; background:rgba(10, 16, 26, 0.92); border:1px solid #1e3a5f; border-radius:8px; font-family:monospace; font-size:12px; line-height:1.6; text-align:left; min-width:320px; max-width:480px; box-shadow:0 4px 16px rgba(0,0,0,0.6);">
    <div style="font-weight:bold; color:#64b5f6; margin-bottom:4px; border-bottom:1px solid #263238; padding-bottom:3px;">🔍 WebXR Diagnostics</div>
    <div>Secure Context: <span id="diag-secure" style="color:#b0bec5;">checking...</span></div>
    <div>WebXR API: <span id="diag-xr" style="color:#b0bec5;">checking...</span></div>
    <div>immersive-vr: <span id="diag-vr" style="color:#b0bec5;">checking...</span></div>
    <div>immersive-ar: <span id="diag-ar" style="color:#b0bec5;">checking...</span></div>
    <div>Session: <span id="diag-session" style="color:#ffd54f;">idle</span></div>
  </div>
</div>

<div id="vr-controller-hud" style="position:fixed; bottom:15px; left:15px; background:rgba(10, 16, 26, 0.88); border:1px solid #1e3a5f; border-radius:8px; padding:10px; font-family:monospace; font-size:11px; color:#cfd8dc; z-index:900; pointer-events:none; min-width:280px; box-shadow:0 4px 12px rgba(0,0,0,0.5);">
  <div style="display:flex; justify-content:space-between; margin-bottom:6px; border-bottom:1px solid #263238; padding-bottom:4px;">
    <span>🎮 VR Teleop</span>
    <span id="vr_hud_rate" style="color:#64b5f6;">0 Hz</span>
    <span id="vr_hud_age" style="color:#81c784;">0 ms</span>
  </div>
  <div style="display:grid; grid-template-columns: 1fr 1fr; gap:6px;">
    <div>
      <div style="color:#90caf9; font-weight:bold; display:flex; justify-content:space-between;">
        <span>LEFT</span><span id="vr_hud_clutch_l" style="background:#37474f; padding:0 3px; border-radius:2px; font-size:9px;">LOCKED</span>
      </div>
      <div>Grasp: <b id="vr_hud_grasp_l" style="color:#ffb74d;">CYLINDER</b></div>
      <div>Closure: <b id="vr_hud_close_l">0%</b></div>
      <div>Grip/Trig: <span id="vr_hud_gt_l">0% / 0%</span></div>
      <div id="vr_hud_prec_l" style="display:none; color:#00e5ff; font-weight:bold;">[PRECISION]</div>
    </div>
    <div>
      <div style="color:#a5d6a7; font-weight:bold; display:flex; justify-content:space-between;">
        <span>RIGHT</span><span id="vr_hud_clutch_r" style="background:#37474f; padding:0 3px; border-radius:2px; font-size:9px;">LOCKED</span>
      </div>
      <div>Grasp: <b id="vr_hud_grasp_r" style="color:#ffb74d;">CYLINDER</b></div>
      <div>Closure: <b id="vr_hud_close_r">0%</b></div>
      <div>Grip/Trig: <span id="vr_hud_gt_r">0% / 0%</span></div>
      <div id="vr_hud_prec_r" style="display:none; color:#00e5ff; font-weight:bold;">[PRECISION]</div>
    </div>
  </div>
  <div id="vr_hud_events" style="margin-top:6px; font-size:9px; color:#78909c; border-top:1px solid #263238; padding-top:3px; max-height:36px; overflow:hidden;">
    -- Ready --
  </div>
</div>
<canvas id="canvas3d"></canvas>

<script>
let scene, camera, renderer, xrSession = null, xrRefSpace = null;
let robotGroup;
let geomObjectMap = {};
let bodyObjectMap = {};
let targetSphereL, targetSphereR;
let actualSphereL, actualSphereR;
let stlLoader = new THREE.STLLoader();
let ws = null;
let latestStatus = null;
let xrRotationEnabled = true;

function init3D() {
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 50.0);
  camera.position.set(0, 1.4, 0.8);

  renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('canvas3d'), antialias: true, alpha: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2.0));
  renderer.xr.enabled = true;

  const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 1.2);
  hemiLight.position.set(0, 20, 0);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
  dirLight.position.set(3, 5, 4);
  scene.add(dirLight);

  robotGroup = new THREE.Group();
  scene.add(robotGroup);

  // Markers
  const mGeo = new THREE.SphereGeometry(0.015, 16, 16);
  targetSphereL = new THREE.Mesh(mGeo, new THREE.MeshStandardMaterial({ color: 0x00e676, emissive: 0x00a844 }));
  robotGroup.add(targetSphereL);

  targetSphereR = new THREE.Mesh(mGeo, new THREE.MeshStandardMaterial({ color: 0x2979ff, emissive: 0x1565c0 }));
  robotGroup.add(targetSphereR);

  window.addEventListener('resize', onWindowResize);
  renderer.setAnimationLoop(renderLoop);

  // User gesture listeners
  const enterBtn = document.getElementById('btn-enter-vr');
  if (enterBtn) {
    enterBtn.addEventListener('click', (e) => {
      e.stopPropagation();
      startXRSession('button');
    });
  }
  window.addEventListener('click', (e) => {
    if (!xrSession && e.target !== enterBtn) {
      startXRSession('window_click');
    }
  }, { once: true });

  checkXR();
}

function updateDiag(field, value, isError = false) {
  console.log(`[WebXR] ${field}: ${value}`);
  const el = document.getElementById(`diag-${field}`);
  if (el) {
    el.textContent = value;
    el.style.color = isError ? '#f87171' : '#4ade80';
  }
}

let supportedMode = 'immersive-vr';
let vrSupported = false;
let arSupported = false;

async function checkXR() {
  const isSecure = window.isSecureContext;
  updateDiag('secure', isSecure ? 'true' : 'false (INSECURE)', !isSecure);

  if (!navigator.xr) {
    updateDiag('xr', 'Not available (use Quest Browser)', true);
    updateDiag('vr', 'N/A', true);
    updateDiag('ar', 'N/A', true);
    updateDiag('session', 'WebXR unavailable', true);
    return;
  }
  updateDiag('xr', 'Available');

  try {
    vrSupported = await navigator.xr.isSessionSupported('immersive-vr').catch(e => {
      console.warn('[WebXR] isSessionSupported(immersive-vr) error:', e);
      return false;
    });
    updateDiag('vr', vrSupported ? 'Supported' : 'Unsupported', !vrSupported);

    arSupported = await navigator.xr.isSessionSupported('immersive-ar').catch(e => {
      console.warn('[WebXR] isSessionSupported(immersive-ar) error:', e);
      return false;
    });
    updateDiag('ar', arSupported ? 'Supported' : 'Unsupported', !arSupported);

    if (arSupported) {
      supportedMode = 'immersive-ar';
    } else if (vrSupported) {
      supportedMode = 'immersive-vr';
    } else {
      updateDiag('session', 'No immersive mode supported', true);
      const statusEl = document.getElementById('vr-status');
      if (statusEl) statusEl.innerText = 'Immersive XR not supported on this device/browser';
    }
  } catch (err) {
    console.error('[WebXR] checkXR error:', err);
    updateDiag('session', `Check Error: ${err.message}`, true);
  }
}

function startXRSession(triggerSource = 'click') {
  if (xrSession) return;
  console.log(`[WebXR] startXRSession triggered via: ${triggerSource}`);

  if (!navigator.xr) {
    updateDiag('session', 'navigator.xr not supported', true);
    return;
  }

  const mode = supportedMode;
  const sessionInit = {
    optionalFeatures: ['local-floor', 'hand-tracking']
  };

  updateDiag('session', `Requesting ${mode}...`);
  console.log(`[WebXR] Calling navigator.xr.requestSession('${mode}') synchronously...`);

  let sessionPromise;
  try {
    sessionPromise = navigator.xr.requestSession(mode, sessionInit);
  } catch (syncErr) {
    console.error('[WebXR] requestSession sync exception:', syncErr);
    updateDiag('session', `Exception: ${syncErr.name}: ${syncErr.message}`, true);
    return;
  }

  sessionPromise.then(async (session) => {
    console.log('[WebXR] requestSession resolved:', session);
    updateDiag('session', `Resolved (${mode} active)`);
    xrSession = session;
    renderer.xr.setSession(session);

    const overlay = document.getElementById('vr-overlay');
    if (overlay) overlay.style.display = 'none';

    // Place robot ~0.8m in front of user at floor level
    robotGroup.position.set(0, 0, -0.8);
    robotGroup.rotation.set(0, 0, 0);

    try {
      xrRefSpace = await session.requestReferenceSpace('local-floor');
    } catch (e) {
      console.warn('[WebXR] local-floor reference space failed, falling back to local:', e);
      xrRefSpace = await session.requestReferenceSpace('local');
    }

    session.addEventListener('end', onXREnd);
  }).catch((err) => {
    console.error('[WebXR] requestSession rejected:', err.name, err.message, err);
    const errText = `Rejected: ${err.name} - ${err.message}`;
    updateDiag('session', errText, true);

    const statusEl = document.getElementById('vr-status');
    if (statusEl) statusEl.innerText = `XR failed: ${err.name || err.message}`;

    if (mode === 'immersive-ar' && vrSupported) {
      console.log('[WebXR] Falling back to immersive-vr for subsequent attempt');
      supportedMode = 'immersive-vr';
    }
  });
}

function onXREnd() {
  xrSession = null;
  xrRefSpace = null;
  document.getElementById('vr-overlay').style.display = 'flex';
  robotGroup.position.set(0, 0, 0);
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function renderLoop(time, frame) {
  renderer.render(scene, camera);

  if (xrSession && frame && xrRefSpace) {
    handleXRInputs(frame);
  }
}

function handleXRInputs(frame) {
  if (!xrSession) return;
  const inputSources = xrSession.inputSources || [];
  let leftPacket = null;
  let rightPacket = null;

  for (let i = 0; i < inputSources.length; i++) {
    const src = inputSources[i];
    if (!src || !src.gamepad) continue;
    const handedness = src.handedness;
    if (handedness !== 'left' && handedness !== 'right') continue;

    const space = src.gripSpace || src.targetRaySpace;
    let posArr = null;
    let quatArr = null;
    if (space) {
      const pose = frame.getPose(space, xrRefSpace);
      if (pose && pose.transform) {
        const p = pose.transform.position;
        const q = pose.transform.orientation;
        posArr = [p.x, p.y, p.z];
        quatArr = [q.w, q.x, q.y, q.z];
      }
    }

    const gp = src.gamepad;
    const btns = gp.buttons || [];
    const axes = gp.axes || [0.0, 0.0];

    let triggerVal = btns.length > 0 ? (typeof btns[0].value === 'number' ? btns[0].value : (btns[0].pressed ? 1.0 : 0.0)) : 0.0;
    let gripVal = btns.length > 1 ? (typeof btns[1].value === 'number' ? btns[1].value : (btns[1].pressed ? 1.0 : 0.0)) : 0.0;
    let gripPressed = gripVal >= 0.25 || Boolean(btns[1]?.pressed);

    let thumbClick = btns.length > 3 ? Boolean(btns[3].pressed) : false;
    let btnPrimary = btns.length > 4 ? Boolean(btns[4].pressed) : false;
    let btnSecondary = btns.length > 5 ? Boolean(btns[5].pressed) : false;
    let btnMenu = btns.length > 6 ? Boolean(btns[6]?.pressed || (typeof btns[6]?.value === 'number' && btns[6].value > 0.5)) : false;

    let stickX = axes.length > 0 ? axes[0] : 0.0;
    let stickY = axes.length > 1 ? axes[1] : 0.0;

    const handData = {
      pos: posArr,
      quat: quatArr,
      grip: gripPressed,
      grip_val: gripVal,
      trigger: triggerVal,
      thumbstick: [stickX, stickY],
      thumbstick_click: thumbClick,
      btn_menu: btnMenu,
      buttons: { primary: btnPrimary, secondary: btnSecondary, menu: btnMenu }
    };

    if (handedness === 'left') {
      handData.btn_x = btnPrimary;
      handData.btn_y = btnSecondary;
      leftPacket = handData;
    } else {
      handData.btn_a = btnPrimary;
      handData.btn_b = btnSecondary;
      rightPacket = handData;
    }
  }

  if (leftPacket || rightPacket) {
    sendCmd({
      cmd: 'xr_controllers',
      t: performance.now() / 1000.0,
      rotation_enabled: xrRotationEnabled,
      left: leftPacket,
      right: rightPacket
    });
  }
}

function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url = `${proto}//${location.host}/ws_panel`;
  ws = new WebSocket(url);
  ws.binaryType = 'arraybuffer';

  ws.onmessage = (event) => {
    if (typeof event.data !== 'string') return;
    try {
      const data = JSON.parse(event.data);
      if (data.type === 'manifest') {
        buildRobotHierarchy(data);
      } else if (data.type === 'poses') {
        updatePoses(data.poses);
      } else if (data.type === 'status') {
        latestStatus = data.status;
        if (targetSphereL && latestStatus.tgt_l) targetSphereL.position.set(latestStatus.tgt_l[0], latestStatus.tgt_l[1], latestStatus.tgt_l[2]);
        if (targetSphereR && latestStatus.tgt_r) targetSphereR.position.set(latestStatus.tgt_r[0], latestStatus.tgt_r[1], latestStatus.tgt_r[2]);

        // Update VR controller status HUD
        if (latestStatus.xr_status) {
          const xr = latestStatus.xr_status;
          const rEl = document.getElementById('vr_hud_rate');
          if (rEl) rEl.innerText = (xr.rate_hz || 0).toFixed(0) + ' Hz';
          const aEl = document.getElementById('vr_hud_age');
          if (aEl) {
            const age = xr.age_ms || 0;
            aEl.innerText = age.toFixed(0) + ' ms';
            aEl.style.color = age > 350 ? '#f44336' : '#81c784';
          }
          if (xr.left) {
            const cl = document.getElementById('vr_hud_clutch_l');
            if (cl) {
              cl.innerText = xr.left.clutch_state || 'LOCKED';
              const cols = { LOCKED: '#37474f', TRANSLATE: '#1976d2', ROTATE: '#f57c00', '6D': '#388e3c' };
              cl.style.background = cols[xr.left.clutch_state] || '#37474f';
            }
            const gr = document.getElementById('vr_hud_grasp_l');
            if (gr) gr.innerText = (xr.left.grasp_type || 'CYLINDER').toUpperCase();
            const clz = document.getElementById('vr_hud_close_l');
            if (clz) clz.innerText = Math.round((xr.left.closure || 0) * 100) + '%';
            const gt = document.getElementById('vr_hud_gt_l');
            if (gt) gt.innerText = `${Math.round((xr.left.grip_val || 0)*100)}% / ${Math.round((xr.left.trigger_val || 0)*100)}%`;
            const pr = document.getElementById('vr_hud_prec_l');
            if (pr) pr.style.display = xr.left.precision ? 'block' : 'none';
          }
          if (xr.right) {
            const cl = document.getElementById('vr_hud_clutch_r');
            if (cl) {
              cl.innerText = xr.right.clutch_state || 'LOCKED';
              const cols = { LOCKED: '#37474f', TRANSLATE: '#1976d2', ROTATE: '#f57c00', '6D': '#388e3c' };
              cl.style.background = cols[xr.right.clutch_state] || '#37474f';
            }
            const gr = document.getElementById('vr_hud_grasp_r');
            if (gr) gr.innerText = (xr.right.grasp_type || 'CYLINDER').toUpperCase();
            const clz = document.getElementById('vr_hud_close_r');
            if (clz) clz.innerText = Math.round((xr.right.closure || 0) * 100) + '%';
            const gt = document.getElementById('vr_hud_gt_r');
            if (gt) gt.innerText = `${Math.round((xr.right.grip_val || 0)*100)}% / ${Math.round((xr.right.trigger_val || 0)*100)}%`;
            const pr = document.getElementById('vr_hud_prec_r');
            if (pr) pr.style.display = xr.right.precision ? 'block' : 'none';
          }
          if (xr.events && xr.events.length > 0) {
            const evEl = document.getElementById('vr_hud_events');
            if (evEl) evEl.innerText = xr.events.slice(-2).map(e => `[${e.t}] ${e.msg}`).join('\\n');
          }
        }
      }
    } catch (e) {
      console.error(e);
    }
  };

  ws.onclose = () => setTimeout(connectWS, 1000);
}

function sendCmd(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function buildRobotHierarchy(manifest) {
  const geoms = manifest.geoms || [];
  geoms.forEach(g => {
    const bname = g.body;
    if (!bodyObjectMap[bname]) {
      const bGroup = new THREE.Group();
      bGroup.name = bname;
      bodyObjectMap[bname] = bGroup;
      robotGroup.add(bGroup);
    }
    const meshPath = `/web_meshes/${g.mesh}`;
    stlLoader.load(meshPath, (geom) => {
      geom.computeVertexNormals();
      const rgba = g.rgba || [0.8, 0.8, 0.8, 1.0];
      const mat = new THREE.MeshStandardMaterial({
        color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
        roughness: 0.45,
        metalness: 0.2
      });
      const mesh = new THREE.Mesh(geom, mat);
      if (g.pos) mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
      if (g.quat) mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
      bodyObjectMap[bname].add(mesh);
      geomObjectMap[g.geom_id] = mesh;
    });
  });
}

function updatePoses(poses) {
  for (const [bname, pose] of Object.entries(poses)) {
    const bGroup = bodyObjectMap[bname];
    if (bGroup && pose.length === 7) {
      bGroup.position.set(pose[0], pose[1], pose[2]);
      bGroup.quaternion.set(pose[4], pose[5], pose[6], pose[3]); // x, y, z, w
    }
  }
}

window.addEventListener('DOMContentLoaded', () => {
  init3D();
  connectWS();
});
</script>
</body>
</html>
"""

