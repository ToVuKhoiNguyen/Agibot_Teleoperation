// Practice scene (main.py --sim --scene pick_bin): table, bin and objects drawn next to the robot, desktop and Quest.
// Server: teleop/simulation/task_scene.py web_def() once per connection, web_poses() at 50 Hz.
// Bodies live in robotGroup (MuJoCo world frame, same as the robot links) so they follow the VR rig too.
let taskBodies = {};        // body name -> THREE.Group
let taskStatusEl = null;

function buildTaskScene(def) {
  if (typeof robotGroup === 'undefined' || !robotGroup) { setTimeout(() => buildTaskScene(def), 200); return; }
  for (const k in taskBodies) robotGroup.remove(taskBodies[k]);   // reconnect: rebuild
  taskBodies = {};
  (def.geoms || []).forEach(g => {
    let grp = taskBodies[g.body];
    if (!grp) {
      grp = new THREE.Group(); grp.name = g.body;
      taskBodies[g.body] = grp; robotGroup.add(grp);
    }
    let geo;
    if (g.type === 'box') geo = new THREE.BoxGeometry(2 * g.size[0], 2 * g.size[1], 2 * g.size[2]);
    else if (g.type === 'cylinder') {
      geo = new THREE.CylinderGeometry(g.size[0], g.size[0], 2 * g.size[1], 24);
      geo.rotateX(Math.PI / 2);                     // three.js cylinders run along y, MuJoCo along z
    } else geo = new THREE.SphereGeometry(g.size[0], 20, 14);
    const mat = new THREE.MeshLambertMaterial({ color: new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]),
                                                transparent: g.rgba[3] < 1, opacity: g.rgba[3] });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(g.pos[0], g.pos[1], g.pos[2]);
    mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);   // MuJoCo w,x,y,z -> three x,y,z,w
    mesh.raycast = function () {};                  // never picked by the gizmo / robot raycaster
    mesh.frustumCulled = false;
    grp.add(mesh);
  });
  sceneNeedsRender = true;
}

function applyTaskPoses(msg) {
  const poses = msg.poses || {};
  for (const k in poses) {
    const grp = taskBodies[k]; if (!grp) continue;
    const p = poses[k];
    grp.position.set(p[0], p[1], p[2]);
    grp.quaternion.set(p[3], p[4], p[5], p[6]);
  }
  if (!taskStatusEl) taskStatusEl = document.getElementById('sync-badge');
  if (taskStatusEl && msg.total) taskStatusEl.title = `pick_bin: ${msg.in_bin}/${msg.total} objects in the bin`;
  sceneNeedsRender = true;
}
