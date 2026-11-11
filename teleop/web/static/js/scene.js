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
let vrRig, robotGroup;
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
  if (!isDraggingGizmo) return; // Drop updates after gizmo_end
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

function flushTargetUpdates(dragEndArm) {
  rafScheduled = false;
  if (pendingTargetL) {
    sendCmd(pendingTargetL);
    pendingTargetL = null;
  }
  if (pendingTargetR) {
    sendCmd(pendingTargetR);
    pendingTargetR = null;
  }
  if (dragEndArm) {
    sendCmd({cmd: 'gizmo_end', arm: dragEndArm});
  }
}

function createTextSprite(text, bgColor) {
  const canvas = document.createElement('canvas'); canvas.width = 128; canvas.height = 128;
  const ctx = canvas.getContext('2d'); ctx.fillStyle = bgColor;
  ctx.beginPath(); ctx.arc(64, 64, 56, 0, 2 * Math.PI); ctx.fill();
  ctx.lineWidth = 8; ctx.strokeStyle = '#ffffff'; ctx.stroke();
  ctx.fillStyle = '#ffffff'; ctx.font = 'bold 64px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, 64, 66);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false, depthWrite: false }));
  sprite.scale.set(0.03 * HANDLE_SCALE, 0.03 * HANDLE_SCALE, 0.03 * HANDLE_SCALE);
  sprite.renderOrder = 999;
  return sprite;
}

function updateSpriteText(sprite, text, bgColor) {
  if (!sprite || !sprite.material) return;
  const canvas = document.createElement('canvas'); canvas.width = 512; canvas.height = 128;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = bgColor || '#00e5ff';
  if (ctx.roundRect) ctx.roundRect(4, 4, 504, 120, 12);
  else ctx.rect(4, 4, 504, 120);
  ctx.fill(); ctx.lineWidth = 4; ctx.strokeStyle = '#ffffff'; ctx.stroke();
  ctx.fillStyle = '#ffffff'; ctx.font = 'bold 20px monospace'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  const lines = (text || '').split('\n');
  if (lines.length === 1) { ctx.fillText(lines[0], 256, 64); }
  else {
    const startY = 64 - (lines.length - 1) * 14;
    lines.forEach((l, i) => ctx.fillText(l, 256, startY + i * 28));
  }
  if (sprite.material.map) sprite.material.map.dispose();
  sprite.material.map = new THREE.CanvasTexture(canvas);
  sprite.material.needsUpdate = true;
  sprite.scale.set(0.6, 0.15, 1.0);
}
window.updateSpriteText = updateSpriteText;

const urlParams = new URLSearchParams(window.location.search);
const isDebug = urlParams.get('debug') === '1';
const showStats = urlParams.get('stats') !== '0';
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

  const statsEl = document.getElementById('stats-overlay');
  if (statsEl && showStats) {
    statsEl.style.display = 'block';
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

  // VR Rig Group (contains robot + wrist markers)
  vrRig = new THREE.Group();
  scene.add(vrRig);

  // Robot Root Group
  robotGroup = new THREE.Group();
  vrRig.add(robotGroup);

  // Orbit Controls
  orbitControls = new THREE.OrbitControls(camera, renderer.domElement);
  orbitControls.target.set(0.2, 0.0, 1.0);
  orbitControls.enableDamping = true;
  orbitControls.dampingFactor = 0.05;
  orbitControls.update();


  if (typeof initGizmos === 'function') initGizmos();
}
