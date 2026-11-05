#!/usr/bin/env python3
"""
Builds an optimized binary bundle of all web meshes for fast Three.js loading.
Target: <= 4 MB gzipped (and <= 6 MB uncompressed).
Original meshes in assets/agibot_a2_ultra/meshes are NEVER touched.
Outputs:
  assets/agibot_a2_ultra/web_meshes/bundle.bin
  assets/agibot_a2_ultra/web_meshes/bundle.bin.gz
"""

import os
import json
import gzip
from pathlib import Path
import trimesh
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
WEB_MESHES_DIR = PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "web_meshes"
BUNDLE_BIN_PATH = WEB_MESHES_DIR / "bundle.bin"
BUNDLE_GZ_PATH = WEB_MESHES_DIR / "bundle.bin.gz"

def build_bundle():
    print(f"Building binary mesh bundle from {WEB_MESHES_DIR}...")
    files = sorted([f for f in os.listdir(WEB_MESHES_DIR) if f.endswith('.stl')])
    print(f"Found {len(files)} STL files.")
    
    bundle_bin = bytearray()
    index = {}
    offset = 0

    for f in files:
        stl_path = WEB_MESHES_DIR / f
        m = trimesh.load(str(stl_path), force='mesh')
        n_faces = len(m.faces)

        # Apply web-only decimation to meet target <= 150k triangles and <= 4MB gzipped
        # Never decimate torso body mesh to keep exact bounding box (< 1 mm)
        if 'body.stl' in f:
            pass
        elif 'base_link' in f or 'paw' in f:
            m = m.simplify_quadric_decimation(percent=0.85)
        elif 'hand' in f and n_faces > 4000:
            m = m.simplify_quadric_decimation(percent=0.85)
        elif n_faces > 4000:
            m = m.simplify_quadric_decimation(percent=0.80)
        elif n_faces > 2000:
            m = m.simplify_quadric_decimation(percent=0.60)
        elif n_faces > 1000:
            m = m.simplify_quadric_decimation(percent=0.40)

        verts = m.vertices.astype(np.float32)
        if len(verts) < 65535:
            faces = m.faces.astype(np.uint16)
            index_type = "uint16"
        else:
            faces = m.faces.astype(np.uint32)
            index_type = "uint32"

        vb = verts.tobytes()
        fb = faces.tobytes()
        total_len = len(vb) + len(fb)

        name = f[:-4]
        meta = {
            "v_count": len(verts),
            "f_count": len(faces) * 3,
            "v_offset": offset,
            "f_offset": offset + len(vb),
            "index_type": index_type,
            "bytes": total_len
        }
        index[name] = meta
        index[f] = meta

        bundle_bin.extend(vb)
        bundle_bin.extend(fb)
        offset += total_len

    idx_json = json.dumps(index).encode('utf-8')
    # Pad idx_json to multiple of 4 bytes
    pad_len = (4 - (len(idx_json) % 4)) % 4
    if pad_len > 0:
        idx_json += b' ' * pad_len
    header = len(idx_json).to_bytes(4, byteorder='little')
    full_data = header + idx_json + bundle_bin

    with open(BUNDLE_BIN_PATH, 'wb') as fb:
        fb.write(full_data)

    gz_data = gzip.compress(full_data, compresslevel=6)
    with open(BUNDLE_GZ_PATH, 'wb') as fg:
        fg.write(gz_data)

    print(f"[SUCCESS] Bundle written:")
    print(f"  Uncompressed: {len(full_data) / (1024*1024):.2f} MB -> {BUNDLE_BIN_PATH}")
    print(f"  Gzipped:      {len(gz_data) / (1024*1024):.2f} MB -> {BUNDLE_GZ_PATH}")

if __name__ == "__main__":
    build_bundle()
