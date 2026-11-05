"""
teleop/web/server/manifest_builder.py
Constructs visual geom manifests from MuJoCo MJCF and caches binary mesh bundles.
"""

import os
import hashlib
from aiohttp import web
import mujoco


def build_visual_manifest(scene_xml: str, project_root: str) -> dict:
    """Constructs static geoms visual manifest from scene MJCF."""
    if scene_xml and os.path.exists(scene_xml):
        model = mujoco.MjModel.from_xml_path(scene_xml)
    else:
        default_xml = os.path.join(project_root, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
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
                "quat": [round(float(x), 6) for x in model.geom_quat[i]],
                "rgba": [round(float(x), 4) for x in model.geom_rgba[i]]
            }
            geoms_list.append(geom_spec)
            if bname not in bodies:
                bodies[bname] = []
            bodies[bname].append(geom_spec)
    return {"type": "manifest", "geoms": geoms_list, "bodies": bodies}


def load_bundle_cache(bundle_bin_path: str, bundle_gz_path: str) -> tuple[bytes, bytes, str]:
    """Loads pre-generated binary mesh bundle into memory and computes ETag."""
    raw = None
    gz = None
    etag = None
    try:
        if os.path.exists(bundle_bin_path):
            with open(bundle_bin_path, "rb") as f:
                raw = f.read()
        if os.path.exists(bundle_gz_path):
            with open(bundle_gz_path, "rb") as f:
                gz = f.read()
        if raw:
            etag = f'"{hashlib.md5(raw).hexdigest()}"'
    except Exception as e:
        print(f"[WARN] Failed to cache bundle in memory: {e}")
    return raw, gz, etag


async def handle_bundle_bin_response(request, bundle_raw: bytes, bundle_gz: bytes, bundle_etag: str):
    """Serves binary mesh bundle with aggressive caching and gzip support."""
    if bundle_etag:
        req_etag = request.headers.get("If-None-Match")
        if req_etag == bundle_etag:
            return web.Response(status=304)

    accept_enc = request.headers.get("Accept-Encoding", "")
    if "gzip" in accept_enc and bundle_gz:
        resp = web.Response(body=bundle_gz, content_type="application/octet-stream")
        resp.headers["Content-Encoding"] = "gzip"
    elif bundle_raw:
        resp = web.Response(body=bundle_raw, content_type="application/octet-stream")
    else:
        return web.Response(status=404, text="Bundle not found")

    resp.headers["Cache-Control"] = "public, max-age=31536000, immutable"
    if bundle_etag:
        resp.headers["ETag"] = bundle_etag
    return resp

