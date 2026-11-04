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
<script src="/static/js/three.min.js"></script>
<script src="/static/js/STLLoader.js"></script>
</head>
<body>
<div id="vr-overlay">
  <button id="btn-enter-vr" onclick="startXRSession()">ENTER VR</button>
  <div id="vr-status">Quest Controller Teleoperation • Tap button or pull trigger</div>
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

  // Auto-request or user gesture listeners
  window.addEventListener('click', onInitialUserInteraction, { once: true });
  window.addEventListener('pointerdown', onInitialUserInteraction, { once: true });
  window.addEventListener('keydown', onInitialUserInteraction, { once: true });

  checkXR();
}

function onInitialUserInteraction() {
  if (!xrSession) {
    startXRSession();
  }
}

async function checkXR() {
  const statusEl = document.getElementById('vr-status');
  if (!navigator.xr) {
    if (statusEl) statusEl.innerText = 'WebXR not supported in this browser (use Quest Browser)';
    return;
  }
  try {
    const isAr = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
    const isVr = await navigator.xr.isSessionSupported('immersive-vr').catch(() => false);
    if (!isAr && !isVr) {
      if (statusEl) statusEl.innerText = 'Immersive XR not available on this device';
      return;
    }
    // Attempt auto-start if allowed by user activation policies
    startXRSession();
  } catch (e) {
    console.warn(e);
  }
}

async function startXRSession() {
  if (xrSession || !navigator.xr) return;
  const btn = document.getElementById('btn-enter-vr');
  const statusEl = document.getElementById('vr-status');

  try {
    let mode = 'immersive-ar';
    let isAr = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
    if (!isAr) mode = 'immersive-vr';

    const session = await navigator.xr.requestSession(mode, {
      optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
    });

    xrSession = session;
    renderer.xr.setSession(session);

    document.getElementById('vr-overlay').style.display = 'none';

    // Place robot ~0.8m in front of user at floor level
    robotGroup.position.set(0, 0, -0.8);
    robotGroup.rotation.set(0, 0, 0);

    try {
      xrRefSpace = await session.requestReferenceSpace('local-floor');
    } catch (e) {
      xrRefSpace = await session.requestReferenceSpace('local');
    }

    session.addEventListener('end', onXREnd);
  } catch (err) {
    console.error("XR start failed:", err);
    if (statusEl) statusEl.innerText = "Tap ENTER VR to allow permission (" + (err.name || err.message) + ")";
  }
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
      buttons: { primary: btnPrimary, secondary: btnSecondary }
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

  ws.onmessage = (event) => {
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

