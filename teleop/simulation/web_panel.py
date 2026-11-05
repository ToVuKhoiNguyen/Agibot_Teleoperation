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
import hashlib
import struct
from aiohttp import web
from .vr_page import HTML_VR

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STATIC_JS_DIR = os.path.join(PROJECT_ROOT, "assets", "static", "js")
WEB_MESHES_DIR = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "web_meshes")
BUNDLE_BIN_PATH = os.path.join(WEB_MESHES_DIR, "bundle.bin")
BUNDLE_GZ_PATH = os.path.join(WEB_MESHES_DIR, "bundle.bin.gz")

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
  #mesh-progress-bar-container {
    position: absolute;
    bottom: 12px;
    left: 50%;
    transform: translateX(-50%);
    width: 260px;
    height: 8px;
    background: rgba(16, 22, 34, 0.85);
    border: 1px solid #3c4b69;
    border-radius: 4px;
    overflow: hidden;
    z-index: 800;
    display: none;
  }
  #mesh-progress-bar {
    width: 0%;
    height: 100%;
    background: linear-gradient(90deg, #00d2ff, #00e676);
    transition: width 0.15s ease-out;
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
  

  
  .hud-notice {
    font-size: 0.72rem;
    color: #64b5f6;
    margin-top: 4px;
    line-height: 1.25;
  }
  .btn-ind {
    display: inline-block; padding: 1px 5px; border-radius: 3px; background: #1c2533;
    color: #607d8b; font-weight: bold; font-size: 9px; border: 1px solid #2d3b4e;
  }
  .btn-ind.active { background: #00bcd4; color: #000; border-color: #80deea; font-weight: 900; }
  .badge-clutch { font-weight: bold; text-transform: uppercase; }
  .tab-btn { background: #1a2330; border: 1px solid #2a3b50; color: #90a4ae; padding: 4px; border-radius: 3px; cursor: pointer; }
  .tab-btn.active-tab { background: #1565c0; color: #fff; border-color: #42a5f5; font-weight: bold; }
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
      <button onclick="adjustGizmoSize(-0.05)" title="Decrease gizmo size">- Size</button>
      <button onclick="adjustGizmoSize(0.05)" title="Increase gizmo size">+ Size</button>
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
      <button id="btn-enter-vr" style="display:none;" onclick="enterVRDirectly()">Enter VR (Quest)</button>
      <div id="gizmo-feedback" class="hud-badge" style="display:none; color:#ffb74d;"></div>
    </div>
    
    <!-- DEBUG OVERLAY (?debug=1) -->
    <div id="debug-overlay" style="display:none; position:absolute; top:48px; left:12px; z-index:99; background:rgba(10,14,23,0.88); border:1px solid #223048; border-radius:6px; padding:6px 10px; font-family:monospace; font-size:11px; color:#00e5ff; line-height:1.45; pointer-events:none; box-shadow:0 4px 12px rgba(0,0,0,0.5);">
      <div>FPS: <b id="dbg-fps" style="color:#fff;">0</b> (<span id="dbg-frametime">0</span> ms)</div>
      <div>Calls: <b id="dbg-calls" style="color:#fff;">0</b> | Tris: <b id="dbg-tris" style="color:#fff;">0</b></div>
      <div>DPR: <b id="dbg-dpr" style="color:#fff;">1.0</b> | Quality: <b id="dbg-quality" style="color:#fff;">low</b></div>
      <div>Stream: <b id="dbg-rate" style="color:#fff;">0</b> Hz | Age: <b id="dbg-age" style="color:#fff;">0</b> ms</div>
    </div>

    <!-- CORNER VR BUTTON (Shown only if immersive-vr supported) -->
    <button id="btn-vr-corner" style="display:none; position:absolute; bottom:16px; right:16px; z-index:100; padding:8px 18px; background:#00c853; color:#fff; border:1px solid #00e676; border-radius:6px; font-weight:bold; font-size:14px; cursor:pointer; box-shadow:0 4px 12px rgba(0,0,0,0.5);" onclick="enterVRDirectly()">VR</button>
    <div id="vr-error-overlay" style="display:none; position:absolute; bottom:56px; right:16px; z-index:100; padding:6px 10px; background:rgba(211,47,47,0.9); color:#fff; border-radius:4px; font-size:12px; max-width:280px; word-break:break-all;"></div>

    <div id="mesh-progress-bar-container">
      <div id="mesh-progress-bar"></div>
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

    <!-- PRESETS & SAFETY -->
    <div class="card">
      <h2>Preset Motions & Demos</h2>
      <button class="btn-estop" id="btn_estop" onclick="sendCmd({cmd:'estop'})">EMERGENCY STOP</button>
      <button class="btn-clear" id="btn_clear_estop" onclick="sendCmd({cmd:'clear_estop'})">Clear E-STOP / Resume</button>
      <div style="display:flex; gap:6px; margin-top:6px; margin-bottom:8px;">
        <button style="flex:1;" onclick="sendCmd({cmd:'hold_toggle'})">Toggle Hold</button>
        <button style="flex:1;" onclick="sendCmd({cmd:'reset'})">Reset (instant)</button>
        <button style="flex:1; background:#1b4d3e; border-color:#2e7d32;" onclick="sendCmd({cmd:'preset', pose:'home'})">Go Home (smooth)</button>
      </div>
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

    <!-- QUEST CONTROLLER STATUS -->
    <div class="card" id="card-quest-status">
      <h2>🎮 Controller Status</h2>
      <div style="display:flex; justify-content:space-between; font-size:10px; margin-bottom:6px; color:#90a4ae;">
        <span>Packets: <b id="xr_pkt_cnt" style="color:#fff;">0</b></span>
        <span>Rate: <b id="xr_rate" style="color:#fff;">0 Hz</b></span>
        <span>Age: <b id="xr_age" style="color:#4caf50;">0 ms</b></span>
      </div>

      <div style="display:grid; grid-template-columns: 1fr 1fr; gap:6px; font-size:10px;">
        <!-- Left Controller -->
        <div style="background:#101722; padding:5px; border-radius:4px; border:1px solid #1e3a5f;">
          <div style="font-weight:bold; color:#64b5f6; display:flex; justify-content:space-between; align-items:center;">
            <span>LEFT</span>
            <span id="xr_clutch_l" class="badge-clutch" style="background:#37474f; color:#eceff1; padding:1px 4px; border-radius:3px; font-size:9px;">LOCKED</span>
          </div>
          <div style="margin-top:3px; font-size:9px;">Grasp: <b id="xr_grasp_l" style="color:#ffb74d;">CYLINDER</b></div>
          <div style="display:flex; align-items:center; gap:3px; margin-top:2px;">
            <span style="width:24px;">Grip:</span>
            <div style="flex:1; background:#1c2533; height:6px; border-radius:3px; overflow:hidden;">
              <div id="xr_bar_grip_l" style="width:0%; height:100%; background:#2196f3;"></div>
            </div>
            <span id="xr_val_grip_l" style="width:22px; text-align:right;">0%</span>
          </div>
          <div style="display:flex; align-items:center; gap:3px; margin-top:2px;">
            <span style="width:24px;">Trig:</span>
            <div style="flex:1; background:#1c2533; height:6px; border-radius:3px; overflow:hidden;">
              <div id="xr_bar_trig_l" style="width:0%; height:100%; background:#00bcd4;"></div>
            </div>
            <span id="xr_val_trig_l" style="width:22px; text-align:right;">0%</span>
          </div>
          <div style="display:flex; justify-content:space-between; align-items:center; margin-top:3px;">
            <div style="display:flex; gap:2px;">
              <span id="xr_btn_x" class="btn-ind">X</span>
              <span id="xr_btn_y" class="btn-ind">Y</span>
              <span id="xr_btn_menul" class="btn-ind" title="Menu button (may be system reserved)">M</span>
            </div>
            <span id="xr_prec_l" style="display:none; font-size:8px; background:#00838f; color:#fff; padding:1px 2px; border-radius:2px;">PREC 0.3x</span>
          </div>
          <div style="display:flex; justify-content:space-between; margin-top:3px; font-size:9px;">
            <span>Close: <b id="xr_close_l">0%</b></span>
            <span>Stick: <b id="xr_stick_l">0, 0</b></span>
          </div>
          <div id="xr_home_box_l" style="display:none; margin-top:3px;">
            <div style="font-size:9px; color:#ff9800;">Hold Home:</div>
            <div style="background:#1c2533; height:4px; border-radius:2px; overflow:hidden;">
              <div id="xr_home_bar_l" style="width:0%; height:100%; background:#ff9800;"></div>
            </div>
          </div>
        </div>

        <!-- Right Controller -->
        <div style="background:#101722; padding:5px; border-radius:4px; border:1px solid #1e3a5f;">
          <div style="font-weight:bold; color:#81c784; display:flex; justify-content:space-between; align-items:center;">
            <span>RIGHT</span>
            <span id="xr_clutch_r" class="badge-clutch" style="background:#37474f; color:#eceff1; padding:1px 4px; border-radius:3px; font-size:9px;">LOCKED</span>
          </div>
          <div style="margin-top:3px; font-size:9px;">Grasp: <b id="xr_grasp_r" style="color:#ffb74d;">CYLINDER</b></div>
          <div style="display:flex; align-items:center; gap:3px; margin-top:2px;">
            <span style="width:24px;">Grip:</span>
            <div style="flex:1; background:#1c2533; height:6px; border-radius:3px; overflow:hidden;">
              <div id="xr_bar_grip_r" style="width:0%; height:100%; background:#4caf50;"></div>
            </div>
            <span id="xr_val_grip_r" style="width:22px; text-align:right;">0%</span>
          </div>
          <div style="display:flex; align-items:center; gap:3px; margin-top:2px;">
            <span style="width:24px;">Trig:</span>
            <div style="flex:1; background:#1c2533; height:6px; border-radius:3px; overflow:hidden;">
              <div id="xr_bar_trig_r" style="width:0%; height:100%; background:#8bc34a;"></div>
            </div>
            <span id="xr_val_trig_r" style="width:22px; text-align:right;">0%</span>
          </div>
          <div style="display:flex; justify-content:space-between; align-items:center; margin-top:3px;">
            <div style="display:flex; gap:2px;">
              <span id="xr_btn_a" class="btn-ind">A</span>
              <span id="xr_btn_b" class="btn-ind">B</span>
              <span id="xr_btn_menur" class="btn-ind" title="Menu button (may be system reserved)">M</span>
            </div>
            <span id="xr_prec_r" style="display:none; font-size:8px; background:#00838f; color:#fff; padding:1px 2px; border-radius:2px;">PREC 0.3x</span>
          </div>
          <div style="display:flex; justify-content:space-between; margin-top:3px; font-size:9px;">
            <span>Close: <b id="xr_close_r">0%</b></span>
            <span>Stick: <b id="xr_stick_r">0, 0</b></span>
          </div>
          <div id="xr_home_box_r" style="display:none; margin-top:3px;">
            <div style="font-size:9px; color:#ff9800;">Hold Home:</div>
            <div style="background:#1c2533; height:4px; border-radius:2px; overflow:hidden;">
              <div id="xr_home_bar_r" style="width:0%; height:100%; background:#ff9800;"></div>
            </div>
          </div>
        </div>
      </div>

      <!-- Last 8 Events Log -->
      <div style="margin-top:5px;">
        <div style="font-size:9px; color:#78909c; margin-bottom:2px;">Last Events:</div>
        <div id="xr_events_log" style="background:#070a0e; border:1px solid #1a2533; border-radius:3px; padding:3px 5px; font-family:monospace; font-size:9px; height:50px; overflow-y:auto; color:#90a4ae;">
          <div>-- Ready --</div>
        </div>
      </div>
    </div>

    <!-- HAND / FINGER TEST -->
    <div class="card" id="card-hand-test">
      <h2>✋ Hand / Finger Test</h2>
      <div style="display:flex; gap:4px; margin-bottom:6px;">
        <button id="hand_tab_l" class="tab-btn active-tab" onclick="selectHandTab('left')" style="flex:1;">Left Hand</button>
        <button id="hand_tab_r" class="tab-btn" onclick="selectHandTab('right')" style="flex:1;">Right Hand</button>
      </div>

      <!-- Grasp Presets -->
      <div style="font-size:10px; color:#90a4ae; margin-bottom:3px;">Grasp Presets:</div>
      <div class="btn-grid" style="grid-template-columns: repeat(3, 1fr); margin-bottom:5px;">
        <button onclick="sendHandGrasp('open')">Open</button>
        <button onclick="sendHandGrasp('cylinder')">Close (Fist)</button>
        <button onclick="sendHandGrasp('pinch')">Pinch</button>
        <button onclick="sendHandGrasp('tripod')">Tripod</button>
        <button onclick="sendHandGrasp('point')">Point</button>
        <button onclick="sendHandGrasp('thumbs_up')">Thumbs-up</button>
      </div>

      <div class="slider-row">
        <span>Closure:</span>
        <input type="range" id="test_closure_slider" min="0" max="100" value="0" oninput="onHandClosureSlider(this.value)">
        <span id="test_closure_val">0%</span>
      </div>

      <!-- Per-Finger Curls (0..1) -->
      <div style="font-size:10px; color:#90a4ae; margin-top:5px; margin-bottom:2px;">Per-Finger Curls (0..1):</div>
      <div id="finger_sliders_container" style="display:grid; grid-template-columns:1fr 1fr; gap:3px; font-size:10px;">
        <div style="display:flex; align-items:center; gap:2px;"><span>Th:</span><input type="range" id="fcurl_thumb" min="0" max="100" value="0" oninput="onFingerCurlChange()"><span id="fcurl_thumb_val" style="width:20px;">0</span></div>
        <div style="display:flex; align-items:center; gap:2px;"><span>Ix:</span><input type="range" id="fcurl_index" min="0" max="100" value="0" oninput="onFingerCurlChange()"><span id="fcurl_index_val" style="width:20px;">0</span></div>
        <div style="display:flex; align-items:center; gap:2px;"><span>Md:</span><input type="range" id="fcurl_mid" min="0" max="100" value="0" oninput="onFingerCurlChange()"><span id="fcurl_mid_val" style="width:20px;">0</span></div>
        <div style="display:flex; align-items:center; gap:2px;"><span>Rg:</span><input type="range" id="fcurl_ring" min="0" max="100" value="0" oninput="onFingerCurlChange()"><span id="fcurl_ring_val" style="width:20px;">0</span></div>
        <div style="display:flex; align-items:center; gap:2px; grid-column:span 2;"><span>Pk:</span><input type="range" id="fcurl_pinky" min="0" max="100" value="0" oninput="onFingerCurlChange()"><span id="fcurl_pinky_val" style="width:20px;">0</span></div>
      </div>

      <!-- Per-Joint Active Sliders (6 active) -->
      <div style="font-size:10px; color:#90a4ae; margin-top:6px; margin-bottom:2px;">Direct Joint Control (Safety Clamped):</div>
      <div id="joint_sliders_container" style="font-size:9px; background:#0b1118; padding:4px; border-radius:3px; border:1px solid #1a2330;">
        <!-- Populated dynamically via JS -->
      </div>

      <!-- Mimic Coupled Joints -->
      <div style="font-size:10px; color:#90a4ae; margin-top:5px; margin-bottom:2px;">Coupled Mimic Joints (Read-Only):</div>
      <div id="mimic_joints_container" style="display:grid; grid-template-columns:1fr 1fr; gap:2px; font-size:9px; background:#090d12; padding:3px; border-radius:3px; color:#80deea;">
        <!-- thumb_2, thumb_3, index_2, middle_2, ring_2, pinky_2 -->
      </div>

      <!-- Sweep Test Controls -->
      <div style="font-size:10px; color:#90a4ae; margin-top:6px; margin-bottom:3px;">Sweep Tests:</div>
      <div style="display:flex; gap:3px; margin-bottom:4px;">
        <button style="flex:2; font-size:9px; padding:4px;" onclick="sendCmd({cmd:'hand_sweep', mode:'single_joints', side:currentHandTab})">Sweep Joints (1s)</button>
        <button style="flex:2; font-size:9px; padding:4px;" onclick="sendCmd({cmd:'hand_sweep', mode:'all_fingers', side:currentHandTab})">Sweep All (3x)</button>
        <button style="flex:1; background:#522828; font-size:9px; padding:4px;" onclick="sendCmd({cmd:'hand_stop'})">Stop</button>
      </div>
      <div id="sweep_status_area" style="font-size:9px; color:#80cbc4; background:#002422; padding:3px 5px; border-radius:3px; border:1px solid #004d40;">
        Ready
      </div>
    </div>


  </div>
</div>

<script>
// Register Service Worker for near-instant cached 3D loading
if ('serviceWorker' in navigator) {
  window.addEventListener('load', () => {
    navigator.serviceWorker.register('/static/js/sw.js').catch(err => console.debug('SW reg info:', err));
  });
}

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
let sharedMaterialCache = {}; // rgba_key -> THREE.MeshLambertMaterial
let stlLoader = new THREE.STLLoader();

// Moving geom IDs list in fixed index order
const MOVING_GEOM_IDS = [
  44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 67, 69, 71, 73,
  75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105,
  107, 109, 111, 113, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136
];
const MOVING_GEOMS_SET = new Set(MOVING_GEOM_IDS);

// Interpolation state for smooth 60fps rendering from 30Hz stream
let posePrev = null; // array of {x, y, z, qx, qy, qz, qw}
let poseCurr = null;
let posePrevTime = 0;
let poseCurrTime = 0;
let sceneNeedsRender = true;

// Reusable math objects to eliminate per-frame allocations
const _tmpV = new THREE.Vector3();
const _tmpQ = new THREE.Quaternion();

// Wrist markers & targets
let targetSphereL, targetSphereR;
let labelSpriteL, labelSpriteR;
let actualSphereL, actualSphereR;
let activeGizmoArm = 'left';
let gizmoMode = 'both'; // 'both' | 'translate' | 'rotate' (DEFAULT = both)
let gizmoSpace = 'world';
const GIZMO_SCALE = (typeof window !== 'undefined' && window.GIZMO_SCALE !== undefined) ? window.GIZMO_SCALE : 0.455;
const HANDLE_SCALE = (typeof window !== 'undefined' && window.HANDLE_SCALE !== undefined) ? window.HANDLE_SCALE : 2.5;
const GIZMO_HANDLE_SCALE = HANDLE_SCALE;
if (typeof window !== 'undefined') {
  window.GIZMO_SCALE = GIZMO_SCALE;
  window.HANDLE_SCALE = HANDLE_SCALE;
  window.GIZMO_HANDLE_SCALE = HANDLE_SCALE;
}
let GIZMO_SIZE = GIZMO_SCALE; // Equal extent size for translate and rotate controls
let followActualWrist = true;
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
  sprite.scale.set(0.03 * HANDLE_SCALE, 0.03 * HANDLE_SCALE, 0.03 * HANDLE_SCALE);
  sprite.renderOrder = 999;
  return sprite;
}

const urlParams = new URLSearchParams(window.location.search);
const isDebug = urlParams.get('debug') === '1';
const qualityMode = (urlParams.get('quality') || 'low').toLowerCase();

function init3D() {
  const container = document.getElementById('viewport-container');
  const canvas = document.getElementById('canvas3d');

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e111a);

  // Camera in robot view (looking from front-quarter)
  camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.05, 50);
  camera.position.set(1.8, -1.8, 1.4); // MuJoCo world coords
  camera.up.set(0, 0, 1);

  const debugEl = document.getElementById('debug-overlay');
  if (debugEl && isDebug) {
    debugEl.style.display = 'block';
    const qEl = document.getElementById('dbg-quality');
    if (qEl) qEl.innerText = qualityMode;
  }

  const useAntialias = (qualityMode === 'high');
  renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: useAntialias, powerPreference: "high-performance" });
  renderer.setSize(container.clientWidth, container.clientHeight);
  if (qualityMode === 'high') {
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
  } else {
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.0));
  }
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

  // Target Markers (Spheres) - mesh is transparent anchor; origin grey sphere is rendered by gizmo
  const sphereGeo = new THREE.SphereGeometry(0.005, 8, 8);
  const sphereMatL = new THREE.MeshStandardMaterial({ color: 0x00e676, emissive: 0x00a844, roughness: 0.3, transparent: true, opacity: 0.0, depthTest: false, visible: false });
  targetSphereL = new THREE.Mesh(sphereGeo, sphereMatL);
  targetSphereL.position.set(0.23457, 0.50362, 1.25004);
  targetSphereL.quaternion.set(-0.24176, 0.81643, 0.51972, -0.06989);
  targetSphereL.userData = { isMarker: true, side: 'left' };
  targetSphereL.renderOrder = 900;
  scene.add(targetSphereL);

  labelSpriteL = createTextSprite('L', '#00c853');
  labelSpriteL.position.set(0, 0, 0.025 * HANDLE_SCALE);
  labelSpriteL.raycast = function() {};
  targetSphereL.add(labelSpriteL);

  const sphereMatR = new THREE.MeshStandardMaterial({ color: 0x2979ff, emissive: 0x1565c0, roughness: 0.3, transparent: true, opacity: 0.0, depthTest: false, visible: false });
  targetSphereR = new THREE.Mesh(sphereGeo, sphereMatR);
  targetSphereR.position.set(0.23457, -0.50362, 1.25004);
  targetSphereR.quaternion.set(0.81771, -0.16138, 0.05290, 0.55001);
  targetSphereR.userData = { isMarker: true, side: 'right' };
  targetSphereR.renderOrder = 900;
  scene.add(targetSphereR);

  labelSpriteR = createTextSprite('R', '#1565c0');
  labelSpriteR.position.set(0, 0, 0.025 * HANDLE_SCALE);
  labelSpriteR.raycast = function() {};
  targetSphereR.add(labelSpriteR);

  // Actual EE feedback spheres (small translucent)
  const actualGeo = new THREE.SphereGeometry(0.008, 12, 12);
  const actualMatL = new THREE.MeshBasicMaterial({ color: 0x69f0ae, wireframe: true });
  actualSphereL = new THREE.Mesh(actualGeo, actualMatL);
  scene.add(actualSphereL);

  const actualMatR = new THREE.MeshBasicMaterial({ color: 0x82b1ff, wireframe: true });
  actualSphereR = new THREE.Mesh(actualGeo, actualMatR);
  scene.add(actualSphereR);

  // Setup Combined TransformControls with unified GIZMO_SIZE
  // Left Marker Controls
  transformControlsL_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_trans.size = GIZMO_SIZE;
  transformControlsL_trans.space = gizmoSpace;
  transformControlsL_trans.setMode('translate');
  transformControlsL_trans.attach(targetSphereL);

  transformControlsL_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsL_rot.size = GIZMO_SIZE;
  transformControlsL_rot.space = gizmoSpace;
  transformControlsL_rot.setMode('rotate');
  transformControlsL_rot.attach(targetSphereL);

  setupGizmoPair(transformControlsL_trans, transformControlsL_rot, targetSphereL, 'left');
  addGizmoToScene(transformControlsL_trans);
  addGizmoToScene(transformControlsL_rot);

  // Right Marker Controls
  transformControlsR_trans = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_trans.size = GIZMO_SIZE;
  transformControlsR_trans.space = gizmoSpace;
  transformControlsR_trans.setMode('translate');
  transformControlsR_trans.attach(targetSphereR);

  transformControlsR_rot = new THREE.TransformControls(camera, renderer.domElement);
  transformControlsR_rot.size = GIZMO_SIZE;
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

  renderer.domElement.addEventListener('pointerdown', function(e) {
    if (transCtrl.axis && rotCtrl.axis) {
      if ((transCtrl.intersectDistance || 0) <= (rotCtrl.intersectDistance || 0)) {
        rotCtrl.axis = null;
      } else {
        transCtrl.axis = null;
      }
    }
  }, true);

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
  sceneNeedsRender = true;
}

function adjustGizmoSize(delta) {
  GIZMO_SIZE = Math.max(0.15, Math.min(1.2, GIZMO_SIZE + delta));
  if (transformControlsL_trans) transformControlsL_trans.size = GIZMO_SIZE;
  if (transformControlsL_rot) transformControlsL_rot.size = GIZMO_SIZE;
  if (transformControlsR_trans) transformControlsR_trans.size = GIZMO_SIZE;
  if (transformControlsR_rot) transformControlsR_rot.size = GIZMO_SIZE;
  sceneNeedsRender = true;
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

function getSharedMaterial(rgba) {
  const key = `${rgba[0]}_${rgba[1]}_${rgba[2]}_${rgba[3]}`;
  if (!sharedMaterialCache[key]) {
    sharedMaterialCache[key] = new THREE.MeshLambertMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      transparent: rgba[3] < 0.99,
      opacity: rgba[3]
    });
  }
  return sharedMaterialCache[key];
}

let bundleLoadedPromise = null;
let bundleIndex = null;
let bundleBuffer = null;

function loadBundle() {
  if (bundleLoadedPromise) return bundleLoadedPromise;
  const pBarCont = document.getElementById('mesh-progress-bar-container');
  const pBar = document.getElementById('mesh-progress-bar');
  if (pBarCont) pBarCont.style.display = 'block';

  bundleLoadedPromise = fetch('/web_meshes/bundle.bin')
    .then(resp => {
      if (!resp.ok) throw new Error('Bundle fetch error ' + resp.status);
      const contentLength = resp.headers.get('content-length');
      if (!contentLength || !resp.body) return resp.arrayBuffer();

      const total = parseInt(contentLength, 10);
      let loaded = 0;
      const reader = resp.body.getReader();
      const chunks = [];

      function read() {
        return reader.read().then(({ done, value }) => {
          if (done) {
            const combined = new Uint8Array(loaded);
            let offset = 0;
            for (const chunk of chunks) {
              combined.set(chunk, offset);
              offset += chunk.length;
            }
            if (pBar) pBar.style.width = '100%';
            setTimeout(() => { if (pBarCont) pBarCont.style.display = 'none'; }, 200);
            return combined.buffer;
          }
          chunks.push(value);
          loaded += value.length;
          if (pBar && total > 0) {
            pBar.style.width = `${Math.min(98, Math.round((loaded / total) * 100))}%`;
          }
          return read();
        });
      }
      return read();
    })
    .then(buf => {
      const dv = new DataView(buf);
      const headerLen = dv.getUint32(0, true);
      const dec = new TextDecoder('utf-8');
      const jsonStr = dec.decode(new Uint8Array(buf, 4, headerLen));
      bundleIndex = JSON.parse(jsonStr);
      bundleBuffer = buf;
      const binStart = 4 + headerLen;

      // Unpack geometries into cache
      for (const meshName in bundleIndex) {
        if (meshGeomCache[meshName]) continue;
        const entry = bundleIndex[meshName];
        const vByteOffset = binStart + entry.v_offset;
        const fByteOffset = binStart + entry.f_offset;

        // Safe slice handles any arbitrary byte alignment safely
        const vBytes = buf.slice(vByteOffset, vByteOffset + entry.v_count * 12);
        const verts = new Float32Array(vBytes);
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(verts, 3));

        if (entry.index_type === 'uint16') {
          const fBytes = buf.slice(fByteOffset, fByteOffset + entry.f_count * 2);
          const faces = new Uint16Array(fBytes);
          geo.setIndex(new THREE.BufferAttribute(faces, 1));
        } else {
          const fBytes = buf.slice(fByteOffset, fByteOffset + entry.f_count * 4);
          const faces = new Uint32Array(fBytes);
          geo.setIndex(new THREE.BufferAttribute(faces, 1));
        }
        geo.computeVertexNormals();
        meshGeomCache[meshName] = geo;
      }
      return true;
    })
    .catch(err => {
      console.warn('Bundle loading failed, falling back to individual STL files:', err);
      if (pBarCont) pBarCont.style.display = 'none';
      return false;
    });
  return bundleLoadedPromise;
}

// Build robot link hierarchy from static manifest sent by server
async function buildRobotHierarchy(manifest) {
  await loadBundle();

  const geoms = manifest.geoms || [];
  if (geoms.length > 0) {
    geoms.forEach(g => {
      const gid = parseInt(g.geom_id, 10);
      if (geomObjectMap[gid]) return;
      const meshName = g.mesh; // e.g. "body.stl"
      const rgba = g.rgba;

      const geomMeshObj = new THREE.Mesh();
      geomMeshObj.name = g.body;
      geomMeshObj.userData = { geom_id: gid, body: g.body };
      geomMeshObj.material = getSharedMaterial(rgba);
      geomMeshObj.frustumCulled = false; // Never cull robot body parts

      // Initialize mesh pose directly from manifest
      if (g.pos) geomMeshObj.position.set(g.pos[0], g.pos[1], g.pos[2]);
      if (g.quat) geomMeshObj.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]); // w,x,y,z -> x,y,z,w
      geomMeshObj.updateMatrix();
      geomMeshObj.updateMatrixWorld(true);

      // Static links (base, legs, torso, head) don't need matrixAutoUpdate
      if (!MOVING_GEOMS_SET.has(gid)) {
        geomMeshObj.matrixAutoUpdate = false;
      } else {
        geomMeshObj.matrixAutoUpdate = true;
      }

      geomObjectMap[gid] = geomMeshObj;
      robotGroup.add(geomMeshObj);

      loadMeshGeometry(meshName, geomMeshObj);
    });
  }

  // Request latest pose snapshot if socket is already open
  sendCmd({ cmd: 'request_full_pose' });

  // Verification assert: check every link has a mesh and pose
  setTimeout(() => {
    let uninitializedCount = 0;
    geoms.forEach(g => {
      const gid = parseInt(g.geom_id, 10);
      const m = geomObjectMap[gid];
      if (!m || !m.geometry) {
        console.warn(`[WARN] Robot link without geometry or pose: geom_id=${gid}, body=${g.body}`);
        uninitializedCount++;
      }
    });
    if (uninitializedCount > 0) {
      console.warn(`[WARN] Total uninitialized robot links: ${uninitializedCount}`);
    }
  }, 1000);

  document.getElementById('sync-badge').innerText = "LIVE (MUJOCO)";
  document.getElementById('sync-badge').className = "tag tag-ok";
  sceneNeedsRender = true;
}

let meshLoadingPending = {};

function loadMeshGeometry(meshName, targetMesh) {
  if (meshGeomCache[meshName]) {
    targetMesh.geometry = meshGeomCache[meshName];
    if (targetMesh.geometry && !targetMesh.geometry.boundingSphere) {
      targetMesh.geometry.computeBoundingSphere();
      targetMesh.geometry.computeBoundingBox();
    }
    targetMesh.updateMatrix();
    targetMesh.updateMatrixWorld(true);
    sceneNeedsRender = true;
    return;
  }
  if (meshLoadingPending[meshName]) {
    meshLoadingPending[meshName].push(targetMesh);
    return;
  }
  meshLoadingPending[meshName] = [targetMesh];
  stlLoader.load(`/web_meshes/${meshName}`, function (geometry) {
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();
    geometry.computeBoundingBox();
    meshGeomCache[meshName] = geometry;
    const targets = meshLoadingPending[meshName] || [];
    targets.forEach(t => {
      t.geometry = geometry;
      t.updateMatrix();
      t.updateMatrixWorld(true);
    });
    delete meshLoadingPending[meshName];
    sceneNeedsRender = true;
  }, undefined, function (err) {
    console.warn(`Mesh load failed: ${meshName}`, err);
    delete meshLoadingPending[meshName];
  });
}

// Stream update from MuJoCo (Per-GEOM world pose)
function applyMuJoCoPoses(poses) {
  latestStreamPoses = poses;
  for (const key in poses) {
    const meshObj = geomObjectMap[key];
    if (meshObj) {
      const p = poses[key]; // [x, y, z, qx, qy, qz, qw] Three.js quaternion order
      meshObj.position.set(p[0], p[1], p[2]);
      meshObj.quaternion.set(p[3], p[4], p[5], p[6]);
      meshObj.updateMatrix();
      meshObj.updateMatrixWorld(true);
    }
  }
  lastMovementTime = performance.now();
  sceneNeedsRender = true;
}

let lastServerTimestamp = 0;
let lastMessageAgeMs = 0;
let lastMovementTime = performance.now();
let latestMovingPoses = null;
let lastF32Snapshot = null;
let msgCounter = 0;
let lastRateCalcTime = performance.now();
let measuredStateRateHz = 0;

// Receive binary float32 buffer for moving links: 48 * 7 float32
function handleBinaryPoses(buffer) {
  const dv = new DataView(buffer);
  const magic = dv.getUint8(0);
  if (magic !== 0xAA) return;

  msgCounter++;
  const nowPerf = performance.now();
  if (nowPerf - lastRateCalcTime >= 1000) {
    measuredStateRateHz = (msgCounter * 1000) / (nowPerf - lastRateCalcTime);
    msgCounter = 0;
    lastRateCalcTime = nowPerf;
  }

  const version = dv.getUint8(1);
  let offset = 4;
  let serverTs = 0;
  if (version === 0x02) {
    serverTs = dv.getFloat64(4, true);
    offset = 12;
    const nowWall = Date.now() / 1000.0;
    lastMessageAgeMs = Math.max(0, (nowWall - serverTs) * 1000.0);
  }

  const count = dv.getUint8(2);
  const f32 = new Float32Array(buffer, offset, count * 7);

  // Check if any pose actually changed compared to last snapshot
  let poseChanged = false;
  if (!lastF32Snapshot || lastF32Snapshot.length !== f32.length) {
    poseChanged = true;
    lastF32Snapshot = new Float32Array(f32);
  } else {
    for (let j = 0; j < f32.length; j++) {
      if (Math.abs(f32[j] - lastF32Snapshot[j]) > 1e-4) {
        poseChanged = true;
        break;
      }
    }
    if (poseChanged) {
      lastF32Snapshot.set(f32);
    }
  }

  if (poseChanged || !latestMovingPoses) {
    const poses = [];
    for (let i = 0; i < count; i++) {
      const off = i * 7;
      poses.push({
        x: f32[off], y: f32[off+1], z: f32[off+2],
        qx: f32[off+3], qy: f32[off+4], qz: f32[off+5], qw: f32[off+6]
      });
    }
    latestMovingPoses = poses;
    lastMovementTime = performance.now();
    sceneNeedsRender = true;
  }
}

// Zero-delay direct pose application + fast exponential smoothing (tau ~ 25 ms)
function applyLatestMovingPoses(dtSec) {
  if (!latestMovingPoses) return;
  const factor = Math.min(1.0, dtSec / 0.025); // fast convergence

  for (let i = 0; i < MOVING_GEOM_IDS.length; i++) {
    const gid = MOVING_GEOM_IDS[i];
    const obj = geomObjectMap[gid];
    if (!obj) continue;

    const tgt = latestMovingPoses[i];
    if (!tgt) continue;

    if (factor >= 0.95) {
      obj.position.set(tgt.x, tgt.y, tgt.z);
      obj.quaternion.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
    } else {
      // Lerp position toward latest target
      obj.position.x += (tgt.x - obj.position.x) * factor;
      obj.position.y += (tgt.y - obj.position.y) * factor;
      obj.position.z += (tgt.z - obj.position.z) * factor;
      // Slerp quaternion toward latest target
      _tmpQ.set(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
      obj.quaternion.slerp(_tmpQ, factor);
    }
  }
}

let lastRenderTime = performance.now();
let renderFrameCount = 0;
let lastFpsUpdateTime = performance.now();
let lastFrameDurationMs = 0;

function animate(currentTime) {
  if (inXRSession) return; // In XR session, onXRFrame handles rendering
  requestAnimationFrame(animate);

  const dtSec = Math.min(0.05, Math.max(0.001, (currentTime - lastRenderTime) / 1000.0));
  lastRenderTime = currentTime;

  // Apply latest poses with zero buffer delay
  if (latestMovingPoses) {
    applyLatestMovingPoses(dtSec);
  }

  // Active rendering while robot moves; on-demand when idle > 1.0s
  const isMoving = (currentTime - lastMovementTime < 1000);
  if (isMoving || sceneNeedsRender || orbitControls.state !== -1 || isDraggingGizmo) {
    const t0 = performance.now();
    orbitControls.update();
    renderer.render(scene, camera);
    lastFrameDurationMs = performance.now() - t0;
    renderFrameCount++;
    if (!isMoving && orbitControls.state === -1 && !isDraggingGizmo) {
      sceneNeedsRender = false;
    }
  }

  // Update debug stats periodically
  if (currentTime - lastFpsUpdateTime >= 250) {
    const elapsed = currentTime - lastFpsUpdateTime;
    const currentFps = Math.round((renderFrameCount * 1000) / elapsed);
    renderFrameCount = 0;
    lastFpsUpdateTime = currentTime;
    if (isDebug) {
      const fpsEl = document.getElementById('dbg-fps');
      const timeEl = document.getElementById('dbg-frametime');
      const callsEl = document.getElementById('dbg-calls');
      const trisEl = document.getElementById('dbg-tris');
      const dprEl = document.getElementById('dbg-dpr');
      const rateEl = document.getElementById('dbg-rate');
      const ageEl = document.getElementById('dbg-age');
      if (fpsEl) fpsEl.innerText = currentFps;
      if (timeEl) timeEl.innerText = lastFrameDurationMs.toFixed(1);
      if (callsEl) callsEl.innerText = renderer.info.render.calls;
      if (trisEl) trisEl.innerText = renderer.info.render.triangles.toLocaleString();
      if (dprEl) dprEl.innerText = renderer.getPixelRatio().toFixed(1);
      if (rateEl) rateEl.innerText = measuredStateRateHz.toFixed(0);
      if (ageEl) ageEl.innerText = lastMessageAgeMs.toFixed(0);
    }
  }
}

// ==========================================
// WebXR Immersive-VR Session Management
// ==========================================
function checkXRSupport() {
  const btn = document.getElementById('btn-vr-corner');
  if (!btn) return;
  btn.style.display = 'none';
  if (!window.isSecureContext || !navigator.xr) return;

  navigator.xr.isSessionSupported('immersive-vr').then(supported => {
    if (supported) {
      btn.style.display = 'block';
    }
  }).catch(err => {
    console.warn('isSessionSupported(immersive-vr) error:', err);
  });
}

function toggleXRRotation(checked) {
  xrRotationEnabled = Boolean(checked);
}

function enterVRDirectly() {
  if (xrSession) {
    xrSession.end();
    return;
  }
  if (!navigator.xr) return;

  const errOverlay = document.getElementById('vr-error-overlay');
  if (errOverlay) errOverlay.style.display = 'none';

  // Synchronous requestSession: no await/fetch before it to maintain user gesture
  navigator.xr.requestSession('immersive-vr', {
    optionalFeatures: ['local-floor', 'hand-tracking']
  }).then(session => {
    onXRSessionStarted(session, 'immersive-vr');
  }).catch(err => {
    console.error('Failed to start WebXR session:', err);
    const msg = (err && (err.name ? (err.name + ': ' + err.message) : err.message)) || 'Unknown VR Error';
    if (errOverlay) {
      errOverlay.innerText = msg;
      errOverlay.style.display = 'block';
    }
  });
}

function toggleXRSession() {
  enterVRDirectly();
}

async function onXRSessionStarted(session, mode) {
  xrSession = session;
  inXRSession = true;
  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) {
    btnCorner.innerText = 'Exit VR';
    btnCorner.style.background = '#d32f2f';
    btnCorner.style.borderColor = '#ff5252';
  }
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

  // Background handling: dark room
  scene.background = new THREE.Color(0x0a0d14);

  // Position robotGroup in front of user at floor level: ~0.8m forward in -Z, feet at Y=0
  robotGroup.position.set(0, 0, -0.8);
  robotGroup.rotation.set(-Math.PI / 2, 0, 0); // Convert MuJoCo Z-up to Three.js WebXR Y-up

  // Session end listener
  session.addEventListener('end', onXRSessionEnded);

  // Setup WebXR rendering context with foveation & frame scaling
  if (renderer.xr) {
    await renderer.xr.setSession(session);
    if (typeof renderer.xr.setFoveation === 'function') {
      renderer.xr.setFoveation(1.0);
    }
    if (typeof renderer.xr.setFramebufferScaleFactor === 'function') {
      renderer.xr.setFramebufferScaleFactor(0.8);
    }
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

  const btnCorner = document.getElementById('btn-vr-corner');
  if (btnCorner) {
    btnCorner.innerText = 'VR';
    btnCorner.style.background = '#00c853';
    btnCorner.style.borderColor = '#00e676';
  }
  const btn = document.getElementById('btn-enter-vr');
  if (btn) {
    btn.innerText = 'Enter VR (Quest)';
    btn.style.background = '#00c853';
    btn.style.borderColor = '#00e676';
  }

  // Restore DOM elements
  const hud = document.getElementById('viewport-hud');
  if (hud) hud.style.display = 'flex';
  const sidebar = document.getElementById('sidebar');
  if (sidebar) sidebar.style.display = 'block';

  const xrTag = document.getElementById('xr-tag');
  if (xrTag) {
    xrTag.innerText = 'XR: IDLE';
    xrTag.className = 'tag tag-warn';
  }

  // Restore robot group pose for desktop view
  robotGroup.position.set(0, 0, 0);
  robotGroup.rotation.set(0, 0, 0);
  scene.background = new THREE.Color(0x0e111a);
  updateGizmoVisibility();
  sceneNeedsRender = true;
  requestAnimationFrame(animate);
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

    // Menu button = Button 6 (or reserved by system)
    let btnMenu = false;
    if (btns.length > 6) {
      btnMenu = Boolean(btns[6].pressed || (typeof btns[6].value === 'number' && btns[6].value > 0.5));
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
      btn_menu: btnMenu,
      buttons: {
        primary: btnPrimary,   // X on Left, A on Right
        secondary: btnSecondary, // Y on Left, B on Right
        menu: btnMenu
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
    const clutchedL = Boolean(s.xr_grip_l || s.held);
    const clutchedR = Boolean(s.xr_grip_r || s.held);

    if (targetSphereL) {
      const pL = (clutchedL || !followActualWrist) ? (s.tgt_l || s.ee_l) : (s.ee_l || s.tgt_l);
      if (pL) targetSphereL.position.set(pL[0], pL[1], pL[2]);
      const rL = (clutchedL && s.tgt_rot_l) ? s.tgt_rot_l : (s.rot_l || s.tgt_rot_l);
      if (rL && gizmoMode !== 'rotate') {
        const m = new THREE.Matrix4();
        m.set(
          rL[0][0], rL[0][1], rL[0][2], 0,
          rL[1][0], rL[1][1], rL[1][2], 0,
          rL[2][0], rL[2][1], rL[2][2], 0,
          0, 0, 0, 1
        );
        targetSphereL.quaternion.setFromRotationMatrix(m);
      }
    }

    if (targetSphereR) {
      const pR = (clutchedR || !followActualWrist) ? (s.tgt_r || s.ee_r) : (s.ee_r || s.tgt_r);
      if (pR) targetSphereR.position.set(pR[0], pR[1], pR[2]);
      const rR = (clutchedR && s.tgt_rot_r) ? s.tgt_rot_r : (s.rot_r || s.tgt_rot_r);
      if (rR && gizmoMode !== 'rotate') {
        const m = new THREE.Matrix4();
        m.set(
          rR[0][0], rR[0][1], rR[0][2], 0,
          rR[1][0], rR[1][1], rR[1][2], 0,
          rR[2][0], rR[2][1], rR[2][2], 0,
          0, 0, 0, 1
        );
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
  document.getElementById('slider_hand_l').value = Math.round(s.hand_closure_l * 100);
  document.getElementById('slider_hand_l_val').innerText = Math.round(s.hand_closure_l * 100) + '%';
  document.getElementById('slider_hand_r').value = Math.round(s.hand_closure_r * 100);
  document.getElementById('slider_hand_r_val').innerText = Math.round(s.hand_closure_r * 100) + '%';

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
      if (valGripL) valGripL.innerText = Math.round((l.grip_val || 0) * 100) + '%';

      const barTrigL = document.getElementById('xr_bar_trig_l');
      if (barTrigL) barTrigL.style.width = Math.round((l.trigger_val || 0) * 100) + '%';
      const valTrigL = document.getElementById('xr_val_trig_l');
      if (valTrigL) valTrigL.innerText = Math.round((l.trigger_val || 0) * 100) + '%';

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
      if (valGripR) valGripR.innerText = Math.round((r.grip_val || 0) * 100) + '%';

      const barTrigR = document.getElementById('xr_bar_trig_r');
      if (barTrigR) barTrigR.style.width = Math.round((r.trigger_val || 0) * 100) + '%';
      const valTrigR = document.getElementById('xr_val_trig_r');
      if (valTrigR) valTrigR.innerText = Math.round((r.trigger_val || 0) * 100) + '%';

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
        self._xr_lock = threading.Lock()
        self._latest_xr_cmd = None
        self.ws_clients = set()
        self.latest_status = {}
        self.status_lock = threading.Lock()

        # Precompute moving geoms and bundle etag
        self._moving_geom_ids = [
            44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 67, 69, 71, 73,
            75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105,
            107, 109, 111, 113, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136
        ]
        self._bundle_raw = None
        self._bundle_gz = None
        self._bundle_etag = None
        self._load_bundle_cache()

        # Slot for server latest-wins pose streaming with dedicated worker thread
        self._latest_moving_binary = None
        self._latest_full_json = None
        self._stream_running = False
        self._stream_worker_thread = None
        self._stream_event = threading.Event()
        self._last_full_broadcast_t = 0.0

        # Visual bodies manifest & geometry mapping
        self.visual_manifest = self._build_visual_manifest()

        @web.middleware
        async def static_cache_middleware(request, handler):
            resp = await handler(request)
            if request.path.startswith("/web_meshes/") or request.path.startswith("/static/"):
                resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
            return resp

        self.app = web.Application(middlewares=[static_cache_middleware])
        # Route 1: Single URL serving on /, /panel, and /vr (no redirects, exact same page)
        self.app.router.add_get("/", self.handle_panel)
        self.app.router.add_get("/panel", self.handle_panel)
        self.app.router.add_get("/vr", self.handle_panel)
        self.app.router.add_get("/health", self.handle_health)
        self.app.router.add_get("/web_meshes/bundle.bin", self.handle_bundle_bin)
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

    def _load_bundle_cache(self):
        try:
            if os.path.exists(BUNDLE_BIN_PATH):
                with open(BUNDLE_BIN_PATH, "rb") as f:
                    self._bundle_raw = f.read()
            if os.path.exists(BUNDLE_GZ_PATH):
                with open(BUNDLE_GZ_PATH, "rb") as f:
                    self._bundle_gz = f.read()
            if self._bundle_raw:
                h = hashlib.md5(self._bundle_raw).hexdigest()
                self._bundle_etag = f'"{h}"'
        except Exception as e:
            print(f"[WARN] Failed to cache bundle in memory: {e}")

    async def handle_bundle_bin(self, request):
        if self._bundle_etag:
            req_etag = request.headers.get("If-None-Match")
            if req_etag == self._bundle_etag:
                return web.Response(status=304)

        accept_enc = request.headers.get("Accept-Encoding", "")
        if "gzip" in accept_enc and self._bundle_gz:
            resp = web.Response(body=self._bundle_gz, content_type="application/octet-stream")
            resp.headers["Content-Encoding"] = "gzip"
        elif self._bundle_raw:
            resp = web.Response(body=self._bundle_raw, content_type="application/octet-stream")
        else:
            return web.Response(status=404, text="Bundle not found")

        resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
        if self._bundle_etag:
            resp.headers["ETag"] = self._bundle_etag
        return resp

    async def handle_root_redirect(self, request):
        return web.Response(text=HTML_PANEL, content_type="text/html")

    async def handle_health(self, request):
        with self.status_lock:
            st = self.latest_status.copy()
        viewer_status = "ACTIVE" if st.get("viewer_running", False) else ("ENABLED" if st.get("viewer_enabled", False) else "OFF")
        nan_detected = bool(st.get("nan_detected", False))
        collision_blocked = bool(st.get("collision_blocked", False))
        hand_collision_blocked = bool(st.get("hand_collision_blocked", False))
        min_distance_mm = float(st.get("min_distance_mm", 999.0))
        min_hand_distance_mm = float(st.get("min_hand_distance_mm", 999.0))
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
            "hand_collision_blocked": hand_collision_blocked,
            "min_distance_mm": min_distance_mm,
            "min_hand_distance_mm": min_hand_distance_mm,
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
        return web.Response(
            text=HTML_PANEL,
            content_type="text/html",
            headers={
                "Cache-Control": "no-cache, no-store, must-revalidate",
                "Pragma": "no-cache",
                "Expires": "0"
            }
        )

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
            with self.status_lock:
                latest_p = getattr(self, "_latest_full_poses_dict", None)
                if latest_p:
                    full_json = json.dumps({"type": "poses", "poses": latest_p})
                else:
                    full_json = self._latest_full_json
            if full_json:
                await ws.send_str(full_json)
        except Exception:
            pass

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get("cmd") == "request_full_pose":
                            with self.status_lock:
                                fj = self._latest_full_json
                            if fj and not ws.closed:
                                asyncio.ensure_future(ws.send_str(fj), loop=self.loop)
                        elif data.get("cmd") == "xr_controllers":
                            if hasattr(self, "xr_receiver") and self.xr_receiver is not None:
                                self.xr_receiver.handle_message(data)
                            with self._xr_lock:
                                self._latest_xr_cmd = data
                        else:
                            if hasattr(self, "xr_receiver") and self.xr_receiver is not None:
                                self.xr_receiver.handle_message(data)
                            self.cmd_queue.put(data)
                    except Exception:
                        pass
                elif msg.type == web.WSMsgType.BINARY:
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
        """Streams 30 Hz MuJoCo link poses to 3D browser viewport via latest-wins slot and dedicated thread."""
        now_ts = time.time()
        # Pack timestamp as float64 (8 bytes) after magic 0xAA and count (2 bytes) + 2 pad = 12 bytes header
        arr = []
        for gid in self._moving_geom_ids:
            p = poses_dict.get(str(gid))
            if p and len(p) == 7:
                arr.extend(p)
            else:
                arr.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        # Header: magic 0xAA, version 0x02, count (1 byte), spare (1 byte), timestamp (double, 8 bytes)
        hdr = struct.pack("<BBBBd", 0xAA, 0x02, len(self._moving_geom_ids), 0, now_ts)
        f_bytes = struct.pack(f"<{len(arr)}f", *arr)
        bin_payload = hdr + f_bytes

        full_json = None
        if now_ts - self._last_full_broadcast_t >= 2.0:
            self._last_full_broadcast_t = now_ts
            full_json = json.dumps({"type": "poses", "poses": poses_dict})

        with self.status_lock:
            self._latest_moving_binary = bin_payload
            self._latest_full_poses_dict = poses_dict
            if full_json:
                self._latest_full_json = full_json
            elif self._latest_full_json is None:
                self._latest_full_json = json.dumps({"type": "poses", "poses": poses_dict})

        if self.loop is not None and len(self.ws_clients) > 0:
            self.loop.call_soon_threadsafe(self._dispatch_immediate_stream, bin_payload, full_json)

    def _dispatch_immediate_stream(self, bin_data, full_json):
        for ws in list(self.ws_clients):
            if not ws.closed:
                transport = ws._req.transport if hasattr(ws, "_req") and ws._req else None
                if transport and hasattr(transport, "get_write_buffer_size") and transport.get_write_buffer_size() > 65536:
                    continue  # Skip lagging client
                try:
                    if full_json:
                        asyncio.ensure_future(ws.send_str(full_json), loop=self.loop)
                    asyncio.ensure_future(ws.send_bytes(bin_data), loop=self.loop)
                except Exception:
                    pass

    def get_command(self):
        try:
            return self.cmd_queue.get_nowait()
        except queue.Empty:
            with self._xr_lock:
                pkt = self._latest_xr_cmd
                self._latest_xr_cmd = None
                return pkt

    def start(self):
        def _runner():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            def _exception_handler(loop, context):
                pass
            self.loop.set_exception_handler(_exception_handler)

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
