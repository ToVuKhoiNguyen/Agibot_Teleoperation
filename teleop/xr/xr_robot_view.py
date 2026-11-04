"""
teleop/xr/xr_robot_view.py
Optional XR Digital Twin View for Meta Quest 3S Headset.
Renders low-poly meshes of the AGIBOT A2 Ultra in front of the operator
inside the WebXR session so they can see the robot simulation in VR.
"""

import os
import glob
import numpy as np
import trimesh

try:
    from vuer.schemas import TriMesh, Group
except ImportError:
    TriMesh = None
    Group = None

class XRRobotRenderer:
    def __init__(self, scene_xml: str, max_faces_per_link: int = 250):
        self.scene_xml = scene_xml
        self.max_faces = max_faces_per_link
        self.meshes = {} # bname -> list of (vertices, faces, pos, quat, color)
        self._load_and_decimate()

    def _load_and_decimate(self):
        import mujoco
        model = mujoco.MjModel.from_xml_path(self.scene_xml)
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        visual_dir = os.path.join(project_root, "assets", "agibot_a2_ultra", "meshes", "visual")

        for i in range(model.ngeom):
            if model.geom_group[i] == 1 and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                bid = model.geom_bodyid[i]
                bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
                mid = model.geom_dataid[i]
                mname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
                stl_path = os.path.join(visual_dir, f"{mname}.stl")
                if not os.path.exists(stl_path):
                    continue
                try:
                    m = trimesh.load(stl_path)
                    if isinstance(m, trimesh.Scene):
                        geoms = [g for g in m.geometry.values() if isinstance(g, trimesh.Trimesh)]
                        m = trimesh.util.concatenate(geoms) if geoms else trimesh.Trimesh()
                    if len(m.faces) > self.max_faces:
                        m = m.simplify_quadric_decimation(face_count=self.max_faces)
                    verts = np.asarray(m.vertices, dtype=np.float32)
                    faces = np.asarray(m.faces, dtype=np.uint32)
                    color = [float(c) for c in model.geom_rgba[i][:3]]
                    if bname not in self.meshes:
                        self.meshes[bname] = []
                    self.meshes[bname].append({
                        "verts": verts,
                        "faces": faces,
                        "color": color,
                        "gpos": model.geom_pos[i].copy(),
                        "gquat": model.geom_quat[i].copy()
                    })
                except Exception:
                    pass

    def get_xr_schemas(self, body_poses: dict):
        """Generates list of Vuer schema elements for XR display."""
        if TriMesh is None:
            return []
        elements = []
        for bname, geoms in self.meshes.items():
            if bname not in body_poses:
                continue
            pose = body_poses[bname] # [x, y, z, qw, qx, qy, qz]
            for idx, g in enumerate(geoms):
                key = f"xr_robot_{bname}_{idx}"
                elem = TriMesh(
                    key=key,
                    vertices=g["verts"],
                    faces=g["faces"],
                    position=[pose[0], pose[1], pose[2]],
                    rotation=[pose[4], pose[5], pose[6], pose[3]],
                    color=g["color"]
                )
                elements.append(elem)
        return elements
