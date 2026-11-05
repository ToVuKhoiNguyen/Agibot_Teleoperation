
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

