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
let minClockSkew = 999999.0;
let poseBuffer = [];

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
    const rawDiff = nowWall - serverTs;
    if (rawDiff < minClockSkew) {
      minClockSkew = rawDiff;
    }
    lastMessageAgeMs = Math.max(0, (rawDiff - minClockSkew) * 1000.0);
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
    // Stamp with the server SAMPLE time (mapped to this clock), not the arrival time: frames are sampled at
    // 100 Hz but sent at ~56 Hz and arrive in bursts, so arrival-time playback judders on fast motion.
    let tSample = nowPerf;
    if (serverTs > 0 && minClockSkew < 999990) {
      tSample = (serverTs + minClockSkew) * 1000.0 - (Date.now() - nowPerf);
      if (Math.abs(tSample - nowPerf) > 500) tSample = nowPerf;   // clock jump: fall back to arrival time
    }
    if (poseBuffer.length > 0 && tSample <= poseBuffer[poseBuffer.length - 1].t) tSample = poseBuffer[poseBuffer.length - 1].t + 0.01;
    poseBuffer.push({ t: tSample, poses: poses });
    if (poseBuffer.length > 8) {
      poseBuffer.shift();
    }
    lastMovementTime = performance.now();
    sceneNeedsRender = true;
  }
}
