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
from .vr_page import HTML_VR

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
    background: rgba(18, 22, 32, 0.90);
    backdrop-filter: blur(8px);
    border: 1px solid #2a354a;
    border-radius: 8px;
    padding: 8px 12px;
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    align-items: center;
    max-width: calc(100% - 28px);
  }
  .hud-badge {
    font-size: 0.72rem;
    padding: 3px 8px;
    border-radius: 4px;
    background: #1e2638;
    color: #90caf9;
    border: 1px solid #32415d;
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
    
    <div class="viewport-controls-hud" id="viewport-hud">
      <span style="font-size:0.75rem; color:#8fa0be; font-weight:bold;">Gizmo:</span>
      <button id="gizmo-mode-both" class="active" onclick="setGizmoMode('both')" title="Shortcut: R">Both (R)</button>
      <button id="gizmo-mode-trans" onclick="setGizmoMode('translate')" title="Shortcut: W">Translate (W)</button>
      <button id="gizmo-mode-rot" onclick="setGizmoMode('rotate')" title="Shortcut: E">Rotate (E)</button>
      <button id="gizmo-space-btn" onclick="toggleGizmoSpace()" title="Shortcut: Q">Space: World (Q)</button>
      <span style="color:#3c4b69;">|</span>
      <button id="btn-toggle-gizmo-l" class="active" onclick="toggleActiveGizmo('left')">Left Marker</button>
      <button id="btn-toggle-gizmo-r" class="active" onclick="toggleActiveGizmo('right')">Right Marker</button>
      <button onclick="resetMarkerToWrist('left')" title="Snap left marker to actual wrist">Reset L</button>
      <button onclick="resetMarkerToWrist('right')" title="Snap right marker to actual wrist">Reset R</button>
      <label style="font-size:0.75rem; color:#8fa0be; display:flex; align-items:center; gap:4px; cursor:pointer;">
        <input type="checkbox" id="chk-follow-wrist" onchange="toggleFollowWrist(this.checked)"> Follow actual
      </label>
      <label style="font-size:0.75rem; color:#8fa0be; display:flex; align-items:center; gap:4px; cursor:pointer;">
        <input type="checkbox" id="chk-xr-rot" checked onchange="toggleXRRotation(this.checked)"> XR 6D Rot
      </label>
      <button onclick="resetCameraView()">Reset Cam</button>
      <button id="btn-enter-vr" style="background:#00c853; color:#fff; font-weight:bold; border-color:#00e676;" onclick="toggleXRSession()">Enter VR (Quest)</button>
      <div id="gizmo-feedback" class="hud-badge" style="display:none; color:#ffb74d;"></div>
    </div>
    
    <canvas id="canvas3d"></canvas>
  </div>

  <!-- RIGHT CONTROL SIDEBAR -->
  <div id="sidebar">
    <div class="panel-header">
      <span style="font-weight: bold; font-size: 0.95rem; color: #00d2ff;">Control Interface</span>
      <div style="display:flex; gap:4px; align-items:center;">
        <span id="input-src-tag" class="tag tag-ok">INPUT: MOUSE</span>
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
let scene, camera, renderer, orbitControls;
let transformControlsL_trans, transformControlsL_rot;
let transformControlsR_trans, transformControlsR_rot;
let transformControlsL, transformControlsR; // Backwards-compatible aliases
let showGizmoL = true;
let showGizmoR = true;
let robotGroup;
let geomObjectMap = {}; // geom_id -> THREE.Mesh
let bodyObjectMap = {}; // bname -> THREE.Group
let meshGeomCache = {}; // filename -> THREE.BufferGeometry
let stlLoader = new THREE.STLLoader();

// Wrist markers & targets
let targetSphereL, targetSphereR;
let labelSpriteL, labelSpriteR;
let actualSphereL, actualSphereR;
let activeGizmoArm = 'left';
let gizmoMode = 'both'; // 'both' | 'translate' | 'rotate' (DEFAULT = both)
let gizmoSpace = 'world';
let followActualWrist = false;
let xrRotationEnabled = true;
let raycaster = new THREE.Raycaster();
let mouse = new THREE.Vector2();

let latestStreamPoses = null;
let lastLeftClosure = 0.0;
let lastRightClosure = 0.0;
let isDraggingGizmo = false;
let latestStatusData = null;

// WebXR Session State
let xrSession = null;
let xrRefSpace = null;
let xrWorldGroup = null;
let xrControllers = []; // [controller0, controller1]
let xrGripSpaces = [];  // [grip0, grip1]
let inXRSession = false;

// Latest-wins streaming with requestAnimationFrame (60 Hz)
let pendingTargetL = null;
let pendingTargetR = null;
let rafScheduled = false;

function queueTargetUpdate(side, pos, quat) {
  const data = {
    cmd: 'set_wrist_target',
    side: side,
    arm: side,
    source: 'gizmo',
    pos: [pos.x, pos.y, pos.z],
    rot_quat: [quat.w, quat.x, quat.y, quat.z]
  };
  if (side === 'left') {
    pendingTargetL = data;
  } else {
    pendingTargetR = data;
  }
  if (!rafScheduled) {
    rafScheduled = true;
    requestAnimationFrame(flushTargetUpdates);
  }
}

function flushTargetUpdates() {
  rafScheduled = false;
  if (pendingTargetL) {
    sendCmd(pendingTargetL);
    pendingTargetL = null;
  }
  if (pendingTargetR) {
    sendCmd(pendingTargetR);
    pendingTargetR = null;
  }
}

function createTextSprite(text, bgColor) {
  const canvas = document.createElement('canvas');
  canvas.width = 128;
  canvas.height = 128;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = bgColor;
  ctx.beginPath();
  ctx.arc(64, 64, 56, 0, 2 * Math.PI);
  ctx.fill();
  ctx.lineWidth = 8;
  ctx.strokeStyle = '#ffffff';
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 64px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 66);

  const texture = new THREE.CanvasTexture(canvas);
  const spriteMat = new THREE.SpriteMaterial({ map: texture, depthTest: false, depthWrite: false });
  const sprite = new THREE.Sprite(spriteMat);
  sprite.scale.set(0.06, 0.06, 0.06);
  sprite.renderOrder = 999;
  return sprite;
}

function init3D() {
  const container = document.getElementById('viewport-container');
  const canvas = document.getElementById('canvas3d');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e111a);

  // Camera in robot view (looking from front-quarter)
  camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.05, 50);
  camera.position.set(1.8, -1.8, 1.4); // MuJoCo world coords
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: false, powerPreference: "high-performance" });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = false;
  if (renderer.xr) {
    renderer.xr.enabled = true;
  }

  // Check WebXR support
  checkXRSupport();

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

  // Target Markers (Spheres)
  const sphereGeo = new THREE.SphereGeometry(0.012, 16, 16);
  const sphereMatL = new THREE.MeshStandardMaterial({ color: 0x00e676, emissive: 0x00a844, roughness: 0.3 });
  targetSphereL = new THREE.Mesh(sphereGeo, sphereMatL);
  targetSphereL.position.set(0.235, 0.504, 1.25);
  targetSphereL.quaternion.set(-0.24176, 0.81643, 0.51972, -0.06989);
  targetSphereL.userData = { isMarker: true, side: 'left' };
  targetSphereL.renderOrder = 900;
  scene.add(targetSphereL);

  labelSpriteL = createTextSprite('L', '#00c853');
  labelSpriteL.position.set(0, 0, 0.025);
  targetSphereL.add(labelSpriteL);

  const sphereMatR = new THREE.MeshStandardMaterial({ color: 0x2979ff, emissive: 0x1565c0, roughness: 0.3 });
  targetSphereR = new THREE.Mesh(sphereGeo, sphereMatR);
  targetSphereR.position.set(0.235, -0.504, 1.25);
  targetSphereR.quaternion.set(0.81771, -0.16138, 0.05290, 0.55001);
  targetSphereR.userData = { isMarker: true, side: 'right' };
  targetSphereR.renderOrder = 900;
  scene.add(targetSphereR);

  labelSpriteR = createTextSprite('R', '#1565c0');
  labelSpriteR.position.set(0, 0, 0.025);
  targetSphereR.add(labelSpriteR);

  // Actual EE feedback spheres (small translucent)
  const actualGeo = new THREE.SphereGeometry(0.008, 12, 12);
  const actualMatL = new THREE.MeshBasicMaterial({ color: 0x69f0ae, wireframe: true });
  actualSphereL = new THREE.Mesh(actualGeo, actualMatL);
  scene.add(actualSphereL);

  const actualMatR = new THREE.MeshBasicMaterial({ color: 0x82b1ff, wireframe: true });
  actualSphereR = new THREE.Mesh(actualGeo, actualMatR);
  scene.add(actualSphereR);

  // Setup Combined TransformControls (Two per marker: one Translate, one Rotate)
  // Left Marker Controls
  transformControlsL_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_trans.size = 0.30;
  transformControlsL_trans.space = gizmoSpace;
  transformControlsL_trans.setMode('translate');
  transformControlsL_trans.attach(targetSphereL);

  transformControlsL_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_rot.size = 0.42; // Slightly larger so rotation rings surround translate arrows
  transformControlsL_rot.space = gizmoSpace;
  transformControlsL_rot.setMode('rotate');
  transformControlsL_rot.attach(targetSphereL);

  setupGizmoPair(transformControlsL_trans, transformControlsL_rot, targetSphereL, 'left');
  addGizmoToScene(transformControlsL_trans);
  addGizmoToScene(transformControlsL_rot);

  // Right Marker Controls
  transformControlsR_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_trans.size = 0.30;
  transformControlsR_trans.space = gizmoSpace;
  transformControlsR_trans.setMode('translate');
  transformControlsR_trans.attach(targetSphereR);

  transformControlsR_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_rot.size = 0.42;
  transformControlsR_rot.space = gizmoSpace;
  transformControlsR_rot.setMode('rotate');
  transformControlsR_rot.attach(targetSphereR);

  setupGizmoPair(transformControlsR_trans, transformControlsR_rot, targetSphereR, 'right');
  addGizmoToScene(transformControlsR_trans);
  addGizmoToScene(transformControlsR_rot);

  // Backwards-compatible aliases
  transformControlsL = transformControlsL_trans;
  transformControlsR = transformControlsR_trans;

  // Initial visibility update according to default mode ('both')
  updateGizmoVisibility();

  // Hand click & marker selection raycaster
  renderer.domElement.addEventListener('pointerdown', onCanvasPointerDown);

  // Keyboard shortcuts: W = translate, E = rotate, R = both (default), Q = toggle local/world
  window.addEventListener('keydown', onKeyDown);

  window.addEventListener('resize', onWindowResize);
  animate();
}

function setupGizmoPair(transCtrl, rotCtrl, targetObj, arm) {
  configureGizmo(transCtrl);
  configureGizmo(rotCtrl);

  transCtrl.addEventListener('dragging-changed', function(e) {
    orbitControls.enabled = !e.value;
    rotCtrl.enabled = !e.value;
    isDraggingGizmo = e.value;
    if (e.value) activeGizmoArm = arm;
    if (!e.value) flushTargetUpdates();
  });

  rotCtrl.addEventListener('dragging-changed', function(e) {
    orbitControls.enabled = !e.value;
    transCtrl.enabled = !e.value;
    isDraggingGizmo = e.value;
    if (e.value) activeGizmoArm = arm;
    if (!e.value) flushTargetUpdates();
  });

  transCtrl.addEventListener('objectChange', function() {
    queueTargetUpdate(arm, targetObj.position, targetObj.quaternion);
  });

  rotCtrl.addEventListener('objectChange', function() {
    queueTargetUpdate(arm, targetObj.position, targetObj.quaternion);
  });
}

function configureGizmo(ctrl) {
  ctrl.renderOrder = 950;
  ctrl.traverse(function(child) {
    if (child.material) {
      child.material.depthTest = false;
      child.material.depthWrite = false;
      child.material.transparent = true;
      child.renderOrder = 950;
    }
  });
}

function addGizmoToScene(ctrl) {
  if (typeof ctrl.getHelper === 'function') {
    scene.add(ctrl.getHelper());
  }
  scene.add(ctrl);
}

function onKeyDown(event) {
  if (['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName)) return;
  const key = event.key.toUpperCase();
  if (key === 'W') {
    setGizmoMode('translate');
  } else if (key === 'E') {
    setGizmoMode('rotate');
  } else if (key === 'R') {
    setGizmoMode('both');
  } else if (key === 'Q') {
    toggleGizmoSpace();
  }
}

function resetCameraView() {
  camera.position.set(1.8, -1.8, 1.4);
  orbitControls.target.set(0.2, 0.0, 1.0);
  orbitControls.update();
}

function setGizmoMode(mode) {
  gizmoMode = mode; // 'both' | 'translate' | 'rotate'
  updateGizmoVisibility();
  const bBoth = document.getElementById('gizmo-mode-both');
  const bTrans = document.getElementById('gizmo-mode-trans');
  const bRot = document.getElementById('gizmo-mode-rot');
  if (bBoth) bBoth.classList.toggle('active', mode === 'both');
  if (bTrans) bTrans.classList.toggle('active', mode === 'translate');
  if (bRot) bRot.classList.toggle('active', mode === 'rotate');
}

function updateGizmoVisibility() {
  const showTrans = (gizmoMode === 'both' || gizmoMode === 'translate');
  const showRot = (gizmoMode === 'both' || gizmoMode === 'rotate');

  const visL = (showGizmoL !== false);
  const visR = (showGizmoR !== false);

  if (transformControlsL_trans) {
    transformControlsL_trans.visible = showTrans && visL;
    transformControlsL_trans.enabled = showTrans && visL;
  }
  if (transformControlsL_rot) {
    transformControlsL_rot.visible = showRot && visL;
    transformControlsL_rot.enabled = showRot && visL;
  }
  if (transformControlsR_trans) {
    transformControlsR_trans.visible = showTrans && visR;
    transformControlsR_trans.enabled = showTrans && visR;
  }
  if (transformControlsR_rot) {
    transformControlsR_rot.visible = showRot && visR;
    transformControlsR_rot.enabled = showRot && visR;
  }
}

function toggleGizmoSpace() {
  gizmoSpace = (gizmoSpace === 'world') ? 'local' : 'world';
  if (transformControlsL_trans) transformControlsL_trans.setSpace(gizmoSpace);
  if (transformControlsL_rot) transformControlsL_rot.setSpace(gizmoSpace);
  if (transformControlsR_trans) transformControlsR_trans.setSpace(gizmoSpace);
  if (transformControlsR_rot) transformControlsR_rot.setSpace(gizmoSpace);
  const btn = document.getElementById('gizmo-space-btn');
  if (btn) {
    btn.innerText = `Space: ${gizmoSpace.charAt(0).toUpperCase() + gizmoSpace.slice(1)} (Q)`;
    btn.classList.toggle('active', gizmoSpace === 'local');
  }
}

function toggleFollowWrist(checked) {
  followActualWrist = Boolean(checked);
}

function resetMarkerToWrist(side) {
  if (!latestStatusData) return;
  if (side === 'left' && targetSphereL && latestStatusData.ee_l) {
    targetSphereL.position.set(latestStatusData.ee_l[0], latestStatusData.ee_l[1], latestStatusData.ee_l[2]);
    sendCmd({
      cmd: 'set_wrist_target',
      side: 'left',
      arm: 'left',
      pos: latestStatusData.ee_l
    });
  } else if (side === 'right' && targetSphereR && latestStatusData.ee_r) {
    targetSphereR.position.set(latestStatusData.ee_r[0], latestStatusData.ee_r[1], latestStatusData.ee_r[2]);
    sendCmd({
      cmd: 'set_wrist_target',
      side: 'right',
      arm: 'right',
      pos: latestStatusData.ee_r
    });
  }
}

function updateMarkerHighlights() {
  if (targetSphereL && targetSphereL.material) {
    if (activeGizmoArm === 'left') {
      targetSphereL.material.emissive.setHex(0x00ff73);
      targetSphereL.scale.set(1.25, 1.25, 1.25);
    } else {
      targetSphereL.material.emissive.setHex(0x005522);
      targetSphereL.scale.set(1.0, 1.0, 1.0);
    }
  }
  if (targetSphereR && targetSphereR.material) {
    if (activeGizmoArm === 'right') {
      targetSphereR.material.emissive.setHex(0x4499ff);
      targetSphereR.scale.set(1.25, 1.25, 1.25);
    } else {
      targetSphereR.material.emissive.setHex(0x103060);
      targetSphereR.scale.set(1.0, 1.0, 1.0);
    }
  }
}

function toggleActiveGizmo(arm) {
  if (arm === 'left') {
    showGizmoL = !showGizmoL;
    const btn = document.getElementById('btn-toggle-gizmo-l');
    if (btn) btn.classList.toggle('active', showGizmoL);
    if (targetSphereL) targetSphereL.visible = showGizmoL;
    if (showGizmoL) activeGizmoArm = 'left';
  } else if (arm === 'right') {
    showGizmoR = !showGizmoR;
    const btn = document.getElementById('btn-toggle-gizmo-r');
    if (btn) btn.classList.toggle('active', showGizmoR);
    if (targetSphereR) targetSphereR.visible = showGizmoR;
    if (showGizmoR) activeGizmoArm = 'right';
  }
  updateGizmoVisibility();
  updateMarkerHighlights();
}

function onCanvasPointerDown(event) {
  if (isDraggingGizmo) return;
  const rect = renderer.domElement.getBoundingClientRect();
  mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);

  // 1. Prioritize clicking on wrist target markers
  const markerHits = raycaster.intersectObjects([targetSphereL, targetSphereR].filter(Boolean), false);
  if (markerHits.length > 0) {
    const hitObj = markerHits[0].object;
    if (hitObj === targetSphereL) {
      toggleActiveGizmo('left');
      return;
    } else if (hitObj === targetSphereR) {
      toggleActiveGizmo('right');
      return;
    }
  }

  // 2. Check clicks on robot hand meshes
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

let lastRenderTime = 0;
const targetRenderInterval = 1000 / 60; // clamp rendering to <= 60 fps

function animate(currentTime) {
  if (inXRSession) return; // In XR session, onXRFrame handles rendering
  requestAnimationFrame(animate);
  if (currentTime - lastRenderTime < targetRenderInterval) {
    return;
  }
  lastRenderTime = currentTime;
  orbitControls.update();
  renderer.render(scene, camera);
}

// ==========================================
// WebXR Immersive-AR / VR Session Management
// ==========================================
function checkXRSupport() {
  const btn = document.getElementById('btn-enter-vr');
  if (!btn) return;
  if (!window.isSecureContext) {
    btn.disabled = true;
    btn.style.opacity = '0.5';
    btn.innerText = 'VR: Insecure Context';
    btn.title = 'WebXR requires HTTPS or localhost';
    return;
  }
  if (!navigator.xr) {
    btn.disabled = true;
    btn.style.opacity = '0.5';
    btn.innerText = 'VR: Unsupported';
    btn.title = 'WebXR not supported in this browser';
    return;
  }
  navigator.xr.isSessionSupported('immersive-ar').then(supported => {
    if (supported) {
      btn.innerText = 'Enter AR (Quest)';
      btn.disabled = false;
    } else {
      navigator.xr.isSessionSupported('immersive-vr').then(vrSupported => {
        if (vrSupported) {
          btn.innerText = 'Enter VR (Quest)';
          btn.disabled = false;
        } else {
          btn.disabled = true;
          btn.style.opacity = '0.5';
          btn.innerText = 'VR: Not Supported';
        }
      }).catch(() => {
        btn.disabled = true;
        btn.innerText = 'VR: Error';
      });
    }
  }).catch(() => {
    btn.disabled = true;
    btn.innerText = 'VR: Error';
  });
}

function toggleXRRotation(checked) {
  xrRotationEnabled = Boolean(checked);
}

async function toggleXRSession() {
  if (xrSession) {
    await xrSession.end();
    return;
  }
  if (!navigator.xr) return;

  const btn = document.getElementById('btn-enter-vr');
  try {
    let sessionInit = {
      optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
    };
    let mode = 'immersive-ar';
    let isSupported = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
    if (!isSupported) {
      mode = 'immersive-vr';
    }

    const session = await navigator.xr.requestSession(mode, sessionInit);
    onXRSessionStarted(session, mode);
  } catch (err) {
    console.error('Failed to start WebXR session:', err);
    if (btn) btn.innerText = 'VR Failed: ' + (err.message || err.name);
  }
}

async function onXRSessionStarted(session, mode) {
  xrSession = session;
  inXRSession = true;
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Exit VR';
    btn.style.background = '#d32f2f';
    btn.style.borderColor = '#ff5252';
  }

  // Update session tag
  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IN SESSION';
    xrTag.className = 'tag tag-ok';
  }

  // Hide 2D HUD and desktop DOM sidebar
  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'none';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'none';

  // Hide desktop TransformControls in VR (markers stay visible)
  if (transformControlsL_trans) transformControlsL_trans.visible = false;
  if (transformControlsL_rot) transformControlsL_rot.visible = false;
  if (transformControlsR_trans) transformControlsR_trans.visible = false;
  if (transformControlsR_rot) transformControlsR_rot.visible = false;

  // Background handling: transparent for AR passthrough, dark room for VR
  if (mode === 'immersive-ar') {
    scene.background = null;
  } else {
    scene.background = new THREE.Color(0x0a0d14);
  }

  // Position robotGroup in front of user at floor level: ~0.8m forward in -Z, feet at Y=0
  robotGroup.position.set(0, 0, -0.8);
  robotGroup.rotation.set(-Math.PI / 2, 0, 0); // Convert MuJoCo Z-up to Three.js WebXR Y-up

  // Session end listener
  session.addEventListener('end', onXRSessionEnded);

  // Setup WebXR rendering context
  if (renderer.xr) {
    await renderer.xr.setSession(session);
  }

  // Reference space: local-floor preferred, fallback local
  try {
    xrRefSpace = await session.requestReferenceSpace('local-floor');
  } catch (e) {
    xrRefSpace = await session.requestReferenceSpace('local');
  }

  session.requestAnimationFrame(onXRFrame);
}

function onXRSessionEnded() {
  xrSession = null;
  inXRSession = false;
  xrRefSpace = null;

  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Enter VR (Quest)';
    btn.style.background = '#00c853';
    btn.style.borderColor = '#00e676';
  }

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IDLE';
    xrTag.className = 'tag tag-warn';
  }

  // Restore DOM HUD and sidebar
  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'flex';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'flex';

  // Restore robot group position and scene background
  robotGroup.position.set(0, 0, 0);
  robotGroup.rotation.set(0, 0, 0);
  scene.background = new THREE.Color(0x0e111a);

  // Restore desktop gizmo visibility
  updateGizmoVisibility();

  // Resume desktop animation loop
  animate();
}

function onXRFrame(time, frame) {
  if (!xrSession) return;
  xrSession.requestAnimationFrame(onXRFrame);

  // Render scene through WebXR camera
  renderer.render(scene, camera);

  if (!xrRefSpace) return;

  // Extract controller poses and buttons
  const inputSources = xrSession.inputSources || [];
  let leftPacket = null;
  let rightPacket = null;

  for (let i = 0; i < inputSources.length; i++) {
    const src = inputSources[i];
    if (!src || !src.gamepad) continue;
    const handedness = src.handedness;
    if (handedness !== 'left' && handedness !== 'right') continue;

    // Pose: targetRaySpace or gripSpace
    const space = src.gripSpace || src.targetRaySpace;
    let posArr = null;
    let quatArr = null;
    if (space) {
      const pose = frame.getPose(space, xrRefSpace);
      if (pose && pose.transform) {
        const p = pose.transform.position;
        const q = pose.transform.orientation;
        posArr = [p.x, p.y, p.z];
        // WebXR (x, y, z, w) -> Python (w, x, y, z)
        quatArr = [q.w, q.x, q.y, q.z];
      }
    }

    const gp = src.gamepad;
    const btns = gp.buttons || [];
    const axes = gp.axes || [0.0, 0.0];

    // Trigger = Button 0
    let triggerVal = 0.0;
    if (btns.length > 0) {
      triggerVal = typeof btns[0].value === 'number' ? btns[0].value : (btns[0].pressed ? 1.0 : 0.0);
    }

    // Grip = Button 1
    let gripVal = 0.0;
    let gripPressed = false;
    if (btns.length > 1) {
      gripVal = typeof btns[1].value === 'number' ? btns[1].value : (btns[1].pressed ? 1.0 : 0.0);
      gripPressed = gripVal >= 0.25 || Boolean(btns[1].pressed);
    }

    // Thumbstick click = Button 3
    let thumbClick = false;
    if (btns.length > 3) {
      thumbClick = Boolean(btns[3].pressed || (typeof btns[3].value === 'number' && btns[3].value > 0.5));
    }

    // Primary button (A on right, X on left) = Button 4
    let btnPrimary = false;
    if (btns.length > 4) {
      btnPrimary = Boolean(btns[4].pressed || (typeof btns[4].value === 'number' && btns[4].value > 0.5));
    }

    // Secondary button (B on right, Y on left) = Button 5
    let btnSecondary = false;
    if (btns.length > 5) {
      btnSecondary = Boolean(btns[5].pressed || (typeof btns[5].value === 'number' && btns[5].value > 0.5));
    }

    // Thumbstick axes
    let stickX = axes.length > 0 ? axes[0] : 0.0;
    let stickY = axes.length > 1 ? axes[1] : 0.0;
    if (Math.abs(stickX) < 0.05 && Math.abs(stickY) < 0.05 && axes.length >= 4) {
      if (Math.abs(axes[2]) > 0.05 || Math.abs(axes[3]) > 0.05) {
        stickX = axes[2];
        stickY = axes[3];
      }
    }

    const handData = {
      pos: posArr,
      quat: quatArr,
      grip: gripPressed,
      grip_val: gripVal,
      trigger: triggerVal,
      thumbstick: [stickX, stickY],
      thumbstick_click: thumbClick,
      buttons: {
        primary: btnPrimary,   // X on Left, A on Right
        secondary: btnSecondary // Y on Left, B on Right
      }
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

  // Send over /ws_panel at display rate
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
  latestStatusData = s;
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

  // Sync target marker positions when not dragging
  if (!isDraggingGizmo) {
    if (followActualWrist) {
      if (targetSphereL && s.ee_l) targetSphereL.position.set(s.ee_l[0], s.ee_l[1], s.ee_l[2]);
      if (targetSphereR && s.ee_r) targetSphereR.position.set(s.ee_r[0], s.ee_r[1], s.ee_r[2]);
    } else {
      if (targetSphereL && s.tgt_l) targetSphereL.position.set(s.tgt_l[0], s.tgt_l[1], s.tgt_l[2]);
      if (targetSphereR && s.tgt_r) targetSphereR.position.set(s.tgt_r[0], s.tgt_r[1], s.tgt_r[2]);
    }
    if (targetSphereL && s.rot_l && gizmoMode !== 'rotate') {
      const m = new THREE.Matrix4();
      m.set(
        s.rot_l[0][0], s.rot_l[0][1], s.rot_l[0][2], 0,
        s.rot_l[1][0], s.rot_l[1][1], s.rot_l[1][2], 0,
        s.rot_l[2][0], s.rot_l[2][1], s.rot_l[2][2], 0,
        0, 0, 0, 1
      );
      targetSphereL.quaternion.setFromRotationMatrix(m);
    }
    if (targetSphereR && s.rot_r && gizmoMode !== 'rotate') {
      const m = new THREE.Matrix4();
      m.set(
        s.rot_r[0][0], s.rot_r[0][1], s.rot_r[0][2], 0,
        s.rot_r[1][0], s.rot_r[1][1], s.rot_r[1][2], 0,
        s.rot_r[2][0], s.rot_r[2][1], s.rot_r[2][2], 0,
        0, 0, 0, 1
      );
      targetSphereR.quaternion.setFromRotationMatrix(m);
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
    } else if (s.warning && s.warning.toLowerCase().includes('clamp')) {
      txt += ` ⚠️ CLAMPED`;
      hudFeedback.style.color = '#ff5252';
      hudFeedback.style.borderColor = '#ff5252';
      hudFeedback.style.backgroundColor = 'rgba(18, 22, 32, 0.85)';
    } else {
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

        @web.middleware
        async def static_cache_middleware(request, handler):
            resp = await handler(request)
            if request.path.startswith("/web_meshes/") or request.path.startswith("/static/"):
                resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
            return resp

        self.app = web.Application(middlewares=[static_cache_middleware])
        # Route 1: Direct panel serving on / and /panel (no redirect on /)
        self.app.router.add_get("/", self.handle_panel)
        self.app.router.add_get("/panel", self.handle_panel)
        self.app.router.add_get("/vr", self.handle_vr)
        self.app.router.add_get("/health", self.handle_health)
        self.app.router.add_get("/ws_panel", self.handle_ws)
        self.app.router.add_post("/api/command", self.handle_post_cmd)
        self.app.router.add_get("/api/status", self.handle_get_status)
        self.app.router.add_post("/api/quest/launch", self.handle_post_quest_launch)

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
        collision_blocked = bool(st.get("collision_blocked", False))
        min_distance_mm = float(st.get("min_distance_mm", 999.0))
        quest_info = {
            "adb": False,
            "device": None,
            "reverse": False,
            "xr_session": bool(st.get("xr_active", False) or st.get("xr_controllers_active", False))
        }
        if hasattr(self, "quest_manager") and self.quest_manager is not None:
            q_st = self.quest_manager.get_status()
            quest_info["adb"] = q_st.get("adb_installed", False)
            quest_info["device"] = q_st.get("device")
            quest_info["device_state"] = q_st.get("device_state")
            quest_info["reverse"] = q_st.get("reverse_active", False)
            quest_info["quest_url"] = q_st.get("quest_url")

        health_resp = {
            "status": "ERROR" if nan_detected else "OK",
            "nan_detected": nan_detected,
            "collision_blocked": collision_blocked,
            "min_distance_mm": min_distance_mm,
            "sim": "RUNNING" if st.get("sim_running", True) else "STOPPED",
            "viewer": viewer_status,
            "server": "LISTENING",
            "panel": "READY",
            "xr_receiver": "CONNECTED" if st.get("xr_active", False) else "IDLE",
            "safety": "ESTOP_TRIGGERED" if st.get("estop", False) else ("COLLISION_BLOCKED" if collision_blocked else "NORMAL"),
            "quest": quest_info,
            "details": st
        }
        return web.json_response(health_resp)

    async def handle_panel(self, request):
        return web.Response(text=HTML_PANEL, content_type="text/html")

    async def handle_vr(self, request):
        return web.Response(text=HTML_VR, content_type="text/html")

    async def handle_post_quest_launch(self, request):
        if hasattr(self, "quest_manager") and self.quest_manager is not None:
            ok = self.quest_manager.launch_quest_browser(force=True)
            return web.json_response({"status": "ok" if ok else "failed"})
        return web.json_response({"status": "error", "message": "Quest ADB manager not available"}, status=503)

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
            site_v4 = web.TCPSite(runner, "0.0.0.0", self.port, ssl_context=ssl_context)
            self.loop.run_until_complete(site_v4.start())
            try:
                site_v6 = web.TCPSite(runner, "::", self.port, ssl_context=ssl_context)
                self.loop.run_until_complete(site_v6.start())
            except Exception:
                pass  # Fallback to IPv4 only if dual-stack :: bind is unavailable
            self.loop.run_forever()

        self.thread = threading.Thread(target=_runner, daemon=True)
        self.thread.start()

    def stop(self):
        """Clean shutdown of aiohttp server and event loop."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)
