"""
teleop/simulation/web_panel.py
Enhanced HTTPS / WebSocket 3D Web Panel server for AGIBOT A2 Ultra.
Provides:
  - 3D WebGL viewport (Three.js STLLoader + OrbitControls + TransformControls)
  - Split side-by-side view (3D viewport left/center, control cards right)
  - Direct control via draggable 3D wrist gizmos (translate + rotate toggle)
  - Interactive hand mesh clicking to toggle open/close
  - Full telemetry and button jogging / presets / walk demo / safety e-stop
  - Real-time 20-30 Hz MuJoCo link poses streaming over WebSocket
  - Static file serving for decimated web meshes and Three.js scripts
  - HTTP redirect / -> /panel
"""

import os
import sys
import ssl
import json
import time
import queue
import asyncio
import threading
import numpy as np
from aiohttp import web

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATIC_JS_DIR = os.path.join(PROJECT_ROOT, "assets", "static", "js")
WEB_MESHES_DIR = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "web_meshes")

HTML_PANEL = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AGIBOT A2 Ultra - 3D Interactive Teleop Panel</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace; }
  html, body { width: 100%; height: 100%; overflow: hidden; background: #0c0e14; color: #e1e4ea; }
  
  #app-container {
    display: flex;
    width: 100vw;
    height: 100vh;
  }
  
  /* 3D VIEWPORT */
  #viewport-container {
    position: relative;
    flex: 1;
    height: 100%;
    background: radial-gradient(circle at 50% 50%, #171b26 0%, #090b10 100%);
    overflow: hidden;
  }
  #canvas3d {
    width: 100%;
    height: 100%;
    display: block;
  }
  
  .viewport-overlay {
    position: absolute;
    top: 14px;
    left: 14px;
    z-index: 10;
    pointer-events: none;
  }
  .viewport-title {
    font-size: 1.15rem;
    font-weight: 800;
    color: #00d2ff;
    text-shadow: 0 2px 4px rgba(0,0,0,0.8);
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .viewport-sub {
    font-size: 0.75rem;
    color: #8fa0be;
    margin-top: 4px;
    text-shadow: 0 1px 2px rgba(0,0,0,0.8);
  }
  
  .viewport-controls-hud {
    position: absolute;
    bottom: 14px;
    left: 14px;
    z-index: 10;
    background: rgba(18, 22, 32, 0.85);
    backdrop-filter: blur(8px);
    border: 1px solid #2a354a;
    border-radius: 8px;
    padding: 8px 12px;
    display: flex;
    gap: 10px;
    align-items: center;
  }
  .viewport-controls-hud button {
    background: #252e42;
    color: #e1e4ea;
    border: 1px solid #3c4b69;
    padding: 5px 10px;
    font-size: 0.75rem;
    border-radius: 4px;
    cursor: pointer;
  }
  .viewport-controls-hud button.active {
    background: #00d2ff;
    color: #000;
    font-weight: bold;
    border-color: #00d2ff;
  }

  /* SIDEBAR CONTROL PANEL */
  #sidebar {
    width: 440px;
    height: 100%;
    background: #12151e;
    border-left: 1px solid #252d3d;
    overflow-y: auto;
    padding: 14px;
    display: flex;
    flex-direction: column;
    gap: 12px;
    box-shadow: -4px 0 20px rgba(0,0,0,0.5);
  }
  
  .panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding-bottom: 8px;
    border-bottom: 1px solid #252d3d;
  }
  .tag { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 0.72rem; font-weight: bold; }
  .tag-ok { background: #1b5e20; color: #a5d6a7; }
  .tag-warn { background: #e65100; color: #ffcc80; }
  .tag-danger { background: #b71c1c; color: #ffcdd2; }

  .card {
    background: #181d29;
    border: 1px solid #283245;
    border-radius: 6px;
    padding: 10px 12px;
  }
  .card h2 {
    font-size: 0.86rem;
    color: #ffbc00;
    margin-bottom: 8px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid #283245;
    padding-bottom: 4px;
  }
  
  .btn-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 5px; margin-bottom: 6px; }
  button {
    background: #252e42; color: #fff; border: 1px solid #3c4a67; border-radius: 4px;
    padding: 6px 4px; font-size: 0.78rem; font-weight: bold; cursor: pointer; user-select: none;
    transition: background 0.1s, border-color 0.1s;
  }
  button:hover { background: #374666; border-color: #00d2ff; }
  button:active { background: #00d2ff; color: #000; }
  
  .btn-estop { background: #c62828; color: #fff; font-size: 1rem; padding: 10px; border: 2px solid #ff5252; width: 100%; font-weight: 900; margin-bottom: 5px; }
  .btn-estop:hover { background: #d32f2f; }
  .btn-clear { background: #2e7d32; color: #fff; font-size: 0.8rem; padding: 5px; width: 100%; border: none; }
  .btn-clear:hover { background: #388e3c; }

  .slider-row { display: flex; align-items: center; justify-content: space-between; margin: 4px 0; font-size: 0.75rem; }
  .slider-row input[type=range] { flex: 1; margin: 0 8px; }
  
  pre#status {
    background: #0a0c12; border: 1px solid #202737; border-radius: 4px; padding: 8px;
    font-size: 0.72rem; color: #73e68c; line-height: 1.35; overflow-x: auto; max-height: 220px;
  }
  
  .hud-notice {
    font-size: 0.72rem;
    color: #64b5f6;
    margin-top: 4px;
    line-height: 1.25;
  }
</style>
<!-- Offline-served Three.js library stack -->
<script src="/static/js/three.min.js"></script>
<script src="/static/js/OrbitControls.js"></script>
<script src="/static/js/TransformControls.js"></script>
<script src="/static/js/STLLoader.js"></script>
</head>
<body>

<div id="app-container">
  <!-- 3D VIEWPORT -->
  <div id="viewport-container">
    <div class="viewport-overlay">
      <div class="viewport-title">
        <span>🤖 AGIBOT A2 Ultra - 3D Digital Twin</span>
        <span id="sync-badge" class="tag tag-warn">SYNCING</span>
      </div>
      <div class="viewport-sub">MuJoCo Source of Truth • 25 Hz Stream • Drag Markers or Click Hands</div>
    </div>
    
    <div class="viewport-controls-hud">
      <span style="font-size:0.75rem; color:#8fa0be; font-weight:bold;">Gizmo:</span>
      <button id="gizmo-mode-trans" class="active" onclick="setGizmoMode('translate')">Translate</button>
      <button id="gizmo-mode-rot" onclick="setGizmoMode('rotate')">Rotate</button>
      <button id="btn-toggle-gizmo-l" class="active" onclick="toggleActiveGizmo('left')">Left Wrist</button>
      <button id="btn-toggle-gizmo-r" onclick="toggleActiveGizmo('right')">Right Wrist</button>
      <button onclick="resetCameraView()">Reset Cam</button>
    </div>
    
    <canvas id="canvas3d"></canvas>
  </div>

  <!-- RIGHT CONTROL SIDEBAR -->
  <div id="sidebar">
    <div class="panel-header">
      <span style="font-weight: bold; font-size: 0.95rem; color: #00d2ff;">Control Interface</span>
      <div style="display:flex; gap:4px; align-items:center;">
        <span id="sim-tag" class="tag tag-ok">SIM: OK</span>
        <span id="xr-tag" class="tag tag-warn">XR: IDLE</span>
        <span id="estop-tag" class="tag tag-ok">ESTOP: OFF</span>
        <span id="conn-tag" class="tag tag-warn">Connecting...</span>
      </div>
    </div>

    <!-- SAFETY & SYSTEM -->
    <div class="card" style="border-color:#5c2b2b;">
      <h2>⚠️ Safety & Mode</h2>
      <button class="btn-estop" id="btn_estop" onclick="sendCmd({cmd:'estop'})">EMERGENCY STOP</button>
      <button class="btn-clear" id="btn_clear_estop" onclick="sendCmd({cmd:'clear_estop'})">Clear E-STOP / Resume</button>
      <div style="display:flex; gap:6px; margin-top:6px;">
        <button style="flex:1;" onclick="sendCmd({cmd:'hold_toggle'})">Toggle Hold</button>
        <button style="flex:1;" onclick="sendCmd({cmd:'reset'})">Reset Home</button>
      </div>
    </div>

    <!-- PRESETS -->
    <div class="card">
      <h2>Preset Motions & Demos</h2>
      <div class="btn-grid">
        <button onclick="sendCmd({cmd:'preset', pose:'home'})">Home</button>
        <button onclick="sendCmd({cmd:'preset', pose:'forward'})">Arms Fwd</button>
        <button onclick="sendCmd({cmd:'preset', pose:'up'})">Arms Up</button>
        <button onclick="sendCmd({cmd:'preset', pose:'tpose'})">T-Pose</button>
        <button onclick="sendCmd({cmd:'preset', pose:'wave'})">Wave (3s)</button>
        <button onclick="sendCmd({cmd:'preset', pose:'circle'})">Arms Circle</button>
        <button onclick="sendCmd({cmd:'preset', pose:'grasp'})">Grasp Demo</button>
        <button onclick="sendCmd({cmd:'walk_demo'})" style="background:#284332;border-color:#4caf50;">Walk Demo</button>
        <button onclick="sendCmd({cmd:'preset', pose:'stop'})" style="background:#522828;">Stop Seq</button>
      </div>
    </div>

    <!-- LEFT ARM & HAND -->
    <div class="card">
      <h2>Left Arm & Hand</h2>
      <div class="slider-row">
        <span>XYZ Step:</span>
        <input type="range" id="step_xyz_l" min="1" max="10" value="2">
        <span id="step_xyz_l_val">2 cm</span>
      </div>
      <div class="btn-grid">
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"x","dir":1}'>+X (Fwd)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"y","dir":1}'>+Y (Left)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"z","dir":1}'>+Z (Up)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"x","dir":-1}'>-X (Back)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"y","dir":-1}'>-Y (Right)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"z","dir":-1}'>-Z (Down)</button>
      </div>
      <div class="slider-row">
        <span>RPY Step:</span>
        <input type="range" id="step_rpy_l" min="1" max="15" value="5">
        <span id="step_rpy_l_val">5°</span>
      </div>
      <div class="btn-grid">
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"roll","dir":1}'>+Roll</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"pitch","dir":1}'>+Pitch</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"yaw","dir":1}'>+Yaw</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"roll","dir":-1}'>-Roll</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"pitch","dir":-1}'>-Pitch</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"left","axis":"yaw","dir":-1}'>-Yaw</button>
      </div>
      <div class="btn-grid" style="margin-top:6px;">
        <button onclick="sendCmd({cmd:'hand','arm':'left','action':'open'})">Open</button>
        <button onclick="sendCmd({cmd:'hand','arm':'left','action':'close'})">Close</button>
        <button onclick="sendCmd({cmd:'hand','arm':'left','action':'pinch'})">Pinch</button>
      </div>
      <div class="slider-row">
        <span>Hand Close:</span>
        <input type="range" id="slider_hand_l" min="0" max="100" value="0">
        <span id="slider_hand_l_val">0%</span>
      </div>
    </div>

    <!-- RIGHT ARM & HAND -->
    <div class="card">
      <h2>Right Arm & Hand</h2>
      <div class="slider-row">
        <span>XYZ Step:</span>
        <input type="range" id="step_xyz_r" min="1" max="10" value="2">
        <span id="step_xyz_r_val">2 cm</span>
      </div>
      <div class="btn-grid">
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"x","dir":1}'>+X (Fwd)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"y","dir":1}'>+Y (Left)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"z","dir":1}'>+Z (Up)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"x","dir":-1}'>-X (Back)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"y","dir":-1}'>-Y (Right)</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"z","dir":-1}'>-Z (Down)</button>
      </div>
      <div class="slider-row">
        <span>RPY Step:</span>
        <input type="range" id="step_rpy_r" min="1" max="15" value="5">
        <span id="step_rpy_r_val">5°</span>
      </div>
      <div class="btn-grid">
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"roll","dir":1}'>+Roll</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"pitch","dir":1}'>+Pitch</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"yaw","dir":1}'>+Yaw</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"roll","dir":-1}'>-Roll</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"pitch","dir":-1}'>-Pitch</button>
        <button class="repeatable" data-cmd='{"cmd":"arm_jog","arm":"right","axis":"yaw","dir":-1}'>-Yaw</button>
      </div>
      <div class="btn-grid" style="margin-top:6px;">
        <button onclick="sendCmd({cmd:'hand','arm':'right','action':'open'})">Open</button>
        <button onclick="sendCmd({cmd:'hand','arm':'right','action':'close'})">Close</button>
        <button onclick="sendCmd({cmd:'hand','arm':'right','action':'pinch'})">Pinch</button>
      </div>
      <div class="slider-row">
        <span>Hand Close:</span>
        <input type="range" id="slider_hand_r" min="0" max="100" value="0">
        <span id="slider_hand_r_val">0%</span>
      </div>
    </div>

    <!-- TELEMETRY STATUS -->
    <div class="card">
      <h2>Live Telemetry</h2>
      <pre id="status">Connecting to AGIBOT A2 Ultra...</pre>
    </div>
  </div>
</div>

<script>
// ==========================================
// 1. THREE.JS 3D DIGITAL TWIN & GIZMOS
// ==========================================
let scene, camera, renderer, orbitControls, transformControls;
let robotGroup;
let geomObjectMap = {}; // geom_id -> THREE.Mesh
let bodyObjectMap = {}; // bname -> THREE.Group
let meshGeomCache = {}; // filename -> THREE.BufferGeometry
let stlLoader = new THREE.STLLoader();

// Wrist markers & targets
let targetSphereL, targetSphereR;
let actualSphereL, actualSphereR;
let activeGizmoArm = 'left';
let gizmoMode = 'translate';
let raycaster = new THREE.Raycaster();
let mouse = new THREE.Vector2();

let latestStreamPoses = null;
let lastLeftClosure = 0.0;
let lastRightClosure = 0.0;
let isDraggingGizmo = false;

function init3D() {
  const container = document.getElementById('viewport-container');
  const canvas = document.getElementById('canvas3d');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e111a);

  // Camera in robot view (looking from front-quarter)
  camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.05, 50);
  camera.position.set(1.8, -1.8, 1.4); // MuJoCo world coords
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
  scene.add(ambientLight);

  const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.7);
  dirLight1.position.set(2, -2, 3);
  scene.add(dirLight1);

  const dirLight2 = new THREE.DirectionalLight(0x90b0e0, 0.4);
  dirLight2.position.set(-2, 2, 2);
  scene.add(dirLight2);

  // Ground Grid (Z=0)
  const gridHelper = new THREE.GridHelper(4.0, 40, 0x00d2ff, 0x223048);
  gridHelper.rotation.x = Math.PI / 2; // MuJoCo Z is up
  gridHelper.position.set(0, 0, 0);
  scene.add(gridHelper);

  // Workspace Bounds Box (faint box: X[0.0, 0.65], Y[-0.75, 0.75], Z[0.6, 1.6])
  const wsBoxGeo = new THREE.BoxGeometry(0.65, 1.50, 1.00);
  const wsBoxMat = new THREE.MeshBasicMaterial({ color: 0x00d2ff, wireframe: true, transparent: true, opacity: 0.15 });
  const wsBox = new THREE.Mesh(wsBoxGeo, wsBoxMat);
  wsBox.position.set(0.325, 0.0, 1.10);
  scene.add(wsBox);

  // Robot Root Group
  robotGroup = new THREE.Group();
  scene.add(robotGroup);

  // Orbit Controls
  orbitControls = new THREE.OrbitControls(camera, renderer.domElement);
  orbitControls.target.set(0.2, 0.0, 1.0);
  orbitControls.enableDamping = true;
  orbitControls.dampingFactor = 0.05;
  orbitControls.update();

  // Transform Controls (Draggable 3D Wrist Gizmo)
  transformControls = new THREE.TransformControls(camera, renderer.domElement);
  transformControls.size = 0.65;
  transformControls.space = 'world';
  transformControls.addEventListener('dragging-changed', function (event) {
    orbitControls.enabled = !event.value;
    isDraggingGizmo = event.value;
  });
  transformControls.addEventListener('objectChange', function () {
    if (activeGizmoArm === 'left' && targetSphereL) {
      const p = targetSphereL.position;
      const q = targetSphereL.quaternion;
      sendCmd({
        cmd: 'set_wrist_target',
        arm: 'left',
        pos: [p.x, p.y, p.z],
        rot_quat: [q.w, q.x, q.y, q.z]
      });
    } else if (activeGizmoArm === 'right' && targetSphereR) {
      const p = targetSphereR.position;
      const q = targetSphereR.quaternion;
      sendCmd({
        cmd: 'set_wrist_target',
        arm: 'right',
        pos: [p.x, p.y, p.z],
        rot_quat: [q.w, q.x, q.y, q.z]
      });
    }
  });
  scene.add(transformControls);

  // Target Markers (Spheres)
  const sphereGeo = new THREE.SphereGeometry(0.022, 16, 16);
  const sphereMatL = new THREE.MeshStandardMaterial({ color: 0x00e676, emissive: 0x00a844, roughness: 0.3 });
  targetSphereL = new THREE.Mesh(sphereGeo, sphereMatL);
  targetSphereL.position.set(0.235, 0.504, 1.25);
  scene.add(targetSphereL);

  const sphereMatR = new THREE.MeshStandardMaterial({ color: 0x2979ff, emissive: 0x1565c0, roughness: 0.3 });
  targetSphereR = new THREE.Mesh(sphereGeo, sphereMatR);
  targetSphereR.position.set(0.235, -0.504, 1.25);
  scene.add(targetSphereR);

  // Actual EE feedback spheres (small translucent)
  const actualGeo = new THREE.SphereGeometry(0.015, 12, 12);
  const actualMatL = new THREE.MeshBasicMaterial({ color: 0x69f0ae, wireframe: true });
  actualSphereL = new THREE.Mesh(actualGeo, actualMatL);
  scene.add(actualSphereL);

  const actualMatR = new THREE.MeshBasicMaterial({ color: 0x82b1ff, wireframe: true });
  actualSphereR = new THREE.Mesh(actualGeo, actualMatR);
  scene.add(actualSphereR);

  // Attach initial gizmo to Left Target Marker
  transformControls.attach(targetSphereL);

  // Hand click raycaster
  renderer.domElement.addEventListener('pointerdown', onCanvasPointerDown);

  window.addEventListener('resize', onWindowResize);
  animate();
}

function resetCameraView() {
  camera.position.set(1.8, -1.8, 1.4);
  orbitControls.target.set(0.2, 0.0, 1.0);
  orbitControls.update();
}

function setGizmoMode(mode) {
  gizmoMode = mode;
  transformControls.setMode(mode);
  document.getElementById('gizmo-mode-trans').classList.toggle('active', mode === 'translate');
  document.getElementById('gizmo-mode-rot').classList.toggle('active', mode === 'rotate');
}

function toggleActiveGizmo(arm) {
  activeGizmoArm = arm;
  if (arm === 'left') {
    transformControls.attach(targetSphereL);
    document.getElementById('btn-toggle-gizmo-l').classList.add('active');
    document.getElementById('btn-toggle-gizmo-r').classList.remove('active');
  } else {
    transformControls.attach(targetSphereR);
    document.getElementById('btn-toggle-gizmo-r').classList.add('active');
    document.getElementById('btn-toggle-gizmo-l').classList.remove('active');
  }
}

function onCanvasPointerDown(event) {
  if (isDraggingGizmo) return;
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);

  // Check clicks on robot hand meshes
  const intersects = raycaster.intersectObjects(robotGroup.children, true);
  if (intersects.length > 0) {
    let topObj = intersects[0].object;
    while (topObj && !topObj.name && topObj.parent) topObj = topObj.parent;
    const hitName = topObj ? topObj.name : "";
    if (hitName.includes("left_hand") || hitName.includes("L_")) {
      const nextAction = (lastLeftClosure > 0.5) ? 'open' : 'close';
      sendCmd({ cmd: 'hand', arm: 'left', action: nextAction });
    } else if (hitName.includes("right_hand") || hitName.includes("R_")) {
      const nextAction = (lastRightClosure > 0.5) ? 'open' : 'close';
      sendCmd({ cmd: 'hand', arm: 'right', action: nextAction });
    }
  }
}

function onWindowResize() {
  const container = document.getElementById('viewport-container');
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// Build robot link hierarchy from static manifest sent by server
function buildRobotHierarchy(manifest) {
  const geoms = manifest.geoms || [];
  if (geoms.length > 0) {
    geoms.forEach(g => {
      const gid = g.geom_id;
      if (geomObjectMap[gid]) return;
      const meshName = g.mesh; // e.g. "body.stl"
      const rgba = g.rgba;

      const geomMeshObj = new THREE.Mesh();
      geomMeshObj.name = g.body; // Retain body name for click raycasting
      geomMeshObj.userData = { geom_id: gid, body: g.body };

      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
        roughness: 0.45,
        metalness: 0.2
      });
      geomMeshObj.material = material;
      geomObjectMap[gid] = geomMeshObj;
      robotGroup.add(geomMeshObj);

      loadMeshGeometry(meshName, geomMeshObj);
    });
  } else {
    // Fallback if legacy bodies manifest
    const bodies = manifest.bodies || {};
    for (const bname in bodies) {
      if (bodyObjectMap[bname]) continue;
      const bodyGrp = new THREE.Group();
      bodyGrp.name = bname;
      bodyObjectMap[bname] = bodyGrp;
      robotGroup.add(bodyGrp);

      const bGeoms = bodies[bname];
      bGeoms.forEach(g => {
        const meshName = g.mesh;
        const geomMeshObj = new THREE.Mesh();
        geomMeshObj.name = bname;
        const material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]),
          roughness: 0.45,
          metalness: 0.2
        });
        geomMeshObj.material = material;
        bodyGrp.add(geomMeshObj);
        loadMeshGeometry(meshName, geomMeshObj);
      });
    }
  }
  document.getElementById('sync-badge').innerText = "LIVE (MUJOCO)";
  document.getElementById('sync-badge').className = "tag tag-ok";
}

let meshLoadingPending = {}; // meshName -> [targetMeshes]

function loadMeshGeometry(meshName, targetMesh) {
  if (meshGeomCache[meshName]) {
    targetMesh.geometry = meshGeomCache[meshName];
    return;
  }
  if (meshLoadingPending[meshName]) {
    meshLoadingPending[meshName].push(targetMesh);
    return;
  }
  meshLoadingPending[meshName] = [targetMesh];
  stlLoader.load(`/web_meshes/${meshName}`, function (geometry) {
    geometry.computeVertexNormals();
    meshGeomCache[meshName] = geometry;
    const targets = meshLoadingPending[meshName] || [];
    targets.forEach(t => { t.geometry = geometry; });
    delete meshLoadingPending[meshName];
  }, undefined, function (err) {
    console.warn(`Mesh load failed: ${meshName}`, err);
    delete meshLoadingPending[meshName];
  });
}

// 25 Hz Stream update from MuJoCo (Per-GEOM world pose)
function applyMuJoCoPoses(poses) {
  latestStreamPoses = poses;
  for (const key in poses) {
    const meshObj = geomObjectMap[key];
    if (meshObj) {
      const p = poses[key]; // [x, y, z, qx, qy, qz, qw] Three.js quaternion order
      meshObj.position.set(p[0], p[1], p[2]);
      meshObj.quaternion.set(p[3], p[4], p[5], p[6]);
    } else {
      const grp = bodyObjectMap[key];
      if (grp) {
        const p = poses[key];
        grp.position.set(p[0], p[1], p[2]);
        grp.quaternion.set(p[4], p[5], p[6], p[3]); // [w,x,y,z] -> [x,y,z,w]
      }
    }
  }
}

function animate() {
  requestAnimationFrame(animate);
  orbitControls.update();
  renderer.render(scene, camera);
}


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

  ws.onmessage = (event) => {
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
  lastLeftClosure = s.hand_closure_l;
  lastRightClosure = s.hand_closure_r;

  // Update badges
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

  // Sync target marker positions when not dragging
  if (!isDraggingGizmo) {
    if (targetSphereL && s.tgt_l) targetSphereL.position.set(s.tgt_l[0], s.tgt_l[1], s.tgt_l[2]);
    if (targetSphereR && s.tgt_r) targetSphereR.position.set(s.tgt_r[0], s.tgt_r[1], s.tgt_r[2]);
  }

  // Update status sidebar
  let heldTag = s.held ? "[HELD]" : "[TRACKING]";
  let estopTag = s.estop ? "[E-STOP ACTIVE]" : "[NORMAL]";
  let out =
`=========================================
STATUS: ${heldTag}  ${estopTag}
Cmd Rate: ${s.cmd_rate_hz.toFixed(1)} Hz | Seq: ${s.walking ? "Walking" : "Idle"}
-----------------------------------------
LEFT ARM (Robot Frame: +X Fwd, +Y Left, +Z Up):
  Actual EE: [${s.ee_l.map(v => (v>=0?'+':'')+v.toFixed(3)).join(', ')}] m
  Target   : [${s.tgt_l.map(v => (v>=0?'+':'')+v.toFixed(3)).join(', ')}] m
  IK Error : ${s.ik_err_l.toFixed(1)} mm
  Hand Cls : ${(s.hand_closure_l * 100).toFixed(1)} %

RIGHT ARM:
  Actual EE: [${s.ee_r.map(v => (v>=0?'+':'')+v.toFixed(3)).join(', ')}] m
  Target   : [${s.tgt_r.map(v => (v>=0?'+':'')+v.toFixed(3)).join(', ')}] m
  IK Error : ${s.ik_err_r.toFixed(1)} mm
  Hand Cls : ${(s.hand_closure_r * 100).toFixed(1)} %
-----------------------------------------
Last Cmd: ${s.last_cmd_name} @ ${s.last_cmd_time.toFixed(2)}s
${s.warning ? "WARN: " + s.warning : ""}
=========================================`;
  document.getElementById('status').innerText = out;

  // Sync sliders
  document.getElementById('slider_hand_l').value = Math.round(s.hand_closure_l * 100);
  document.getElementById('slider_hand_l_val').innerText = Math.round(s.hand_closure_l * 100) + '%';
  document.getElementById('slider_hand_r').value = Math.round(s.hand_closure_r * 100);
  document.getElementById('slider_hand_r_val').innerText = Math.round(s.hand_closure_r * 100) + '%';
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

// Initialize on page load
window.addEventListener('DOMContentLoaded', () => {
  init3D();
  connectWS();
});
</script>
</body>
</html>
"""


class WebPanelServer:
    """
    Lightweight HTTPS server providing the 3D WebGL / WebXR control panel,
    high-speed WebSocket poses stream, and REST endpoints for AGIBOT A2 Ultra.
    """

    def __init__(self, port: int = 8012, cert_file: str = None, key_file: str = None, scene_xml: str = None):
        self.port = port
        self.cert_file = cert_file
        self.key_file = key_file
        self.scene_xml = scene_xml
        self.cmd_queue = queue.Queue()
        self.ws_clients = set()
        self.latest_status = {}
        self.status_lock = threading.Lock()

        # Visual bodies manifest & geometry mapping
        self.visual_manifest = self._build_visual_manifest()

        self.app = web.Application()
        # Route 1: Redirect / to /panel
        self.app.router.add_get("/", self.handle_root_redirect)
        self.app.router.add_get("/panel", self.handle_panel)
        self.app.router.add_get("/health", self.handle_health)
        self.app.router.add_get("/ws_panel", self.handle_ws)
        self.app.router.add_post("/api/command", self.handle_post_cmd)
        self.app.router.add_get("/api/status", self.handle_get_status)

        # Serve static assets (Three.js and decimated meshes)
        if os.path.exists(STATIC_JS_DIR):
            self.app.router.add_static("/static/js", STATIC_JS_DIR)
        if os.path.exists(WEB_MESHES_DIR):
            self.app.router.add_static("/web_meshes", WEB_MESHES_DIR)

        self.loop = None
        self.thread = None

    def _build_visual_manifest(self):
        """Constructs static geoms visual manifest from scene MJCF."""
        import mujoco
        if self.scene_xml and os.path.exists(self.scene_xml):
            model = mujoco.MjModel.from_xml_path(self.scene_xml)
        else:
            default_xml = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
            model = mujoco.MjModel.from_xml_path(default_xml)

        geoms_list = []
        bodies = {}
        for i in range(model.ngeom):
            if model.geom_group[i] == 1 and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                bid = model.geom_bodyid[i]
                bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
                mid = model.geom_dataid[i]
                mname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
                geom_spec = {
                    "geom_id": str(i),
                    "body": bname,
                    "mesh": f"{mname}.stl",
                    "pos": [round(float(x), 6) for x in model.geom_pos[i]],
                    "quat": [round(float(x), 6) for x in model.geom_quat[i]], # w, x, y, z
                    "rgba": [round(float(x), 4) for x in model.geom_rgba[i]]
                }
                geoms_list.append(geom_spec)
                if bname not in bodies:
                    bodies[bname] = []
                bodies[bname].append(geom_spec)
        return {"type": "manifest", "geoms": geoms_list, "bodies": bodies}

    async def handle_root_redirect(self, request):
        return web.HTTPFound("/panel")

    async def handle_health(self, request):
        with self.status_lock:
            st = self.latest_status.copy()
        viewer_status = "ACTIVE" if st.get("viewer_running", False) else ("ENABLED" if st.get("viewer_enabled", False) else "OFF")
        nan_detected = bool(st.get("nan_detected", False))
        health_resp = {
            "status": "ERROR" if nan_detected else "OK",
            "nan_detected": nan_detected,
            "sim": "RUNNING" if st.get("sim_running", True) else "STOPPED",
            "viewer": viewer_status,
            "server": "LISTENING",
            "panel": "READY",
            "xr_receiver": "CONNECTED" if st.get("xr_active", False) else "IDLE",
            "safety": "ESTOP_TRIGGERED" if st.get("estop", False) else "NORMAL",
            "details": st
        }
        return web.json_response(health_resp)

    async def handle_panel(self, request):
        return web.Response(text=HTML_PANEL, content_type="text/html")

    async def handle_ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.ws_clients.add(ws)

        # Immediately send static robot manifest to initialize 3D scene
        try:
            await ws.send_str(json.dumps(self.visual_manifest))
        except Exception:
            pass

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if hasattr(self, "xr_receiver") and self.xr_receiver is not None:
                            self.xr_receiver.handle_message(data)
                        self.cmd_queue.put(data)
                    except Exception:
                        pass
        finally:
            self.ws_clients.discard(ws)
        return ws

    async def handle_post_cmd(self, request):
        try:
            data = await request.json()
            self.cmd_queue.put(data)
            return web.json_response({"status": "ok"})
        except Exception as e:
            return web.json_response({"status": "error", "message": str(e)}, status=400)

    async def handle_get_status(self, request):
        with self.status_lock:
            return web.json_response(self.latest_status)

    def set_status(self, status_dict: dict):
        """Sets status telemetry and broadcasts to connected clients."""
        with self.status_lock:
            self.latest_status = status_dict.copy()
        if self.loop is not None and len(self.ws_clients) > 0:
            msg = json.dumps({"type": "status", "status": status_dict})
            for ws in list(self.ws_clients):
                if not ws.closed:
                    asyncio.run_coroutine_threadsafe(ws.send_str(msg), self.loop)

    def broadcast_poses(self, poses_dict: dict):
        """Streams 25-30 Hz MuJoCo link poses to 3D browser viewport."""
        if self.loop is not None and len(self.ws_clients) > 0:
            msg = json.dumps({"type": "poses", "poses": poses_dict})
            for ws in list(self.ws_clients):
                if not ws.closed:
                    asyncio.run_coroutine_threadsafe(ws.send_str(msg), self.loop)

    def get_command(self):
        try:
            return self.cmd_queue.get_nowait()
        except queue.Empty:
            return None

    def start(self):
        def _runner():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(self.cert_file, self.key_file)

            runner = web.AppRunner(self.app)
            self.loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, "0.0.0.0", self.port, ssl_context=ssl_context)
            self.loop.run_until_complete(site.start())
            self.loop.run_forever()

        self.thread = threading.Thread(target=_runner, daemon=True)
        self.thread.start()

    def stop(self):
        """Clean shutdown of aiohttp server and event loop."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
