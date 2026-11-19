#!/usr/bin/env python3
"""
decimate_meshes.py
Decimates visual STL meshes for the AGIBOT A2 Ultra into assets/agibot_a2_ultra/web_meshes/
to optimize loading time and memory in the WebGL / WebXR viewer.
Leaves the original meshes untouched.
"""

import os
import glob
import trimesh

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "meshes", "visual")
DST_DIR = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "web_meshes")

def decimate_all():
    os.makedirs(DST_DIR, exist_ok=True)
    stl_files = sorted(glob.glob(os.path.join(SRC_DIR, "*.stl")))
    print(f"[INFO] Found {len(stl_files)} STL meshes in {SRC_DIR}")

    for path in stl_files:
        name = os.path.basename(path)
        out_path = os.path.join(DST_DIR, name)
        
        mesh = trimesh.load(path)
        if isinstance(mesh, trimesh.Scene):
            geoms = [g for g in mesh.geometry.values() if isinstance(g, trimesh.Trimesh)]
            mesh = trimesh.util.concatenate(geoms) if geoms else trimesh.Trimesh()

        n_faces = len(mesh.faces)
        # Skip simplification for complex thin linkages/paws to prevent bounding box shifts
        if "paw" in name or "hand" in name or "base_link" in name:
            print(f"  Copied {name:31s}: {n_faces:6d} faces (unsimplified to maintain < 1mm accuracy)")
            mesh.export(out_path)
            continue

        if n_faces > 10000:
            reduction = 1.0 - (10000.0 / float(n_faces))
            dec = mesh.simplify_quadric_decimation(percent=reduction)
            # Verify centroid and bounding box error <= 1mm
            c_err = float(trimesh.util.np.linalg.norm(dec.centroid - mesh.centroid))
            b_err = float(trimesh.util.np.linalg.norm(dec.bounds - mesh.bounds))
            if c_err > 0.001 or b_err > 0.001:
                # If error exceeds 1 mm, keep original to guarantee fidelity
                print(f"  Preserved {name:28s}: {n_faces:6d} faces (decimation shifted bbox {b_err*1000:.2f}mm)")
            else:
                mesh = dec
                print(f"  Decimated {name:28s}: {n_faces:6d} -> {len(mesh.faces):6d} faces")
        else:
            print(f"  Copied {name:31s}: {n_faces:6d} faces")
        
        mesh.export(out_path)

    print("[SUCCESS] All web meshes cached in", DST_DIR)

if __name__ == "__main__":
    decimate_all()
