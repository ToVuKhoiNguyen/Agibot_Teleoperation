#!/usr/bin/env python3
"""
scripts/test_web_sync.py
Automated acceptance test for Web Control Panel 3D sync and redirect:
  (a) https://localhost:8012/ redirects to /panel (HTTP 302/200)
  (b) WebSocket connection receives robot visual manifest & live MuJoCo link poses
  (c) Link poses streamed over WebSocket match MuJoCo xpos / xquat within < 1 mm
  (d) Wrist target marker dragging (/api/command set_wrist_target) updates MuJoCo EE poses
  (e) Emergency stop operates cleanly via Web API
"""

import os
import sys
import time
import ssl
import json
import urllib.request
import asyncio
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

PORT = 8012
API_BASE_URL = f"https://localhost:{PORT}"
ROOT_URL = f"https://localhost:{PORT}/"
PANEL_URL = f"https://localhost:{PORT}/panel"
API_CMD_URL = f"https://localhost:{PORT}/api/command"
API_STATUS_URL = f"https://localhost:{PORT}/api/status"
WS_URL = f"wss://localhost:{PORT}/ws_panel"

def get_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx

def test_http_redirect():
    print("[TEST 1] Testing HTTP Redirect / -> /panel ...")
    ctx = get_ssl_context()
    
    # Custom handler to detect redirect
    class NoRedirectHandler(urllib.request.HTTPRedirectHandler):
        def http_error_302(self, req, fp, code, msg, headers):
            return fp
        def http_error_301(self, req, fp, code, msg, headers):
            return fp

    opener = urllib.request.build_opener(urllib.request.HTTPSHandler(context=ctx), NoRedirectHandler)
    req = urllib.request.Request(ROOT_URL)
    try:
        resp = opener.open(req, timeout=3.0)
        status_code = resp.status if hasattr(resp, "status") else resp.code
        loc = resp.headers.get("Location", "")
        print(f"  Response Status: {status_code}, Location: {loc}")
        if status_code in (301, 302) and ("/panel" in loc):
            print("  [PASS] / correctly redirects to /panel")
            return True
        elif status_code == 200 and resp.geturl().endswith("/panel"):
            print("  [PASS] / resolved to /panel")
            return True
    except Exception as e:
        print(f"  [FAIL] Redirect test error: {e}")
        return False
    return False

async def test_websocket_stream():
    print("\n[TEST 2] Testing WebSocket Link Pose Stream vs MuJoCo Source of Truth ...")
    import aiohttp
    
    ssl_ctx = get_ssl_context()
    conn = aiohttp.TCPConnector(ssl=ssl_ctx)
    async with aiohttp.ClientSession(connector=conn) as session:
        async with session.ws_connect(WS_URL) as ws:
            print("  Connected to WebSocket at", WS_URL)
            
            # 1. Expect Manifest
            manifest = None
            start_t = time.time()
            while time.time() - start_t < 3.0:
                msg = await ws.receive_json(timeout=2.0)
                if msg.get("type") == "manifest":
                    manifest = msg
                    break
            
            if manifest and "bodies" in manifest and len(manifest["bodies"]) > 0:
                print(f"  [PASS] Received visual robot manifest: {len(manifest['bodies'])} visual bodies defined.")
            else:
                print("  [FAIL] Failed to receive visual manifest.")
                return False

            # Verify every visual geom's mesh URL returns 200, finite pose, and link->mesh mapping
            print("\n  [ASSET & MANIFEST VERIFICATION] Checking all visual geoms mesh URLs and mappings...")
            manifest_geoms = manifest.get("geoms", [])
            assert len(manifest_geoms) >= 70, f"Expected >= 70 visual geoms, found {len(manifest_geoms)}"
            # Verify entries for torso / pelvis / head (base_link, head_link01, head_link02)
            bodies_present = set(manifest.get("bodies", {}).keys())
            assert "base_link" in bodies_present, "base_link (torso/pelvis) missing from manifest bodies!"
            assert "head_link01" in bodies_present, "head_link01 missing from manifest bodies!"
            assert "head_link02" in bodies_present, "head_link02 missing from manifest bodies!"

            verified_meshes = set()
            for g in manifest_geoms:
                m_name = g["mesh"]
                if m_name in verified_meshes:
                    continue
                mesh_url = f"{API_BASE_URL}/web_meshes/{m_name}"
                mesh_req = urllib.request.Request(mesh_url)
                with urllib.request.urlopen(mesh_req, context=ssl_ctx, timeout=3.0) as m_resp:
                    assert m_resp.status == 200, f"Mesh {mesh_url} returned status {m_resp.status}"
                verified_meshes.add(m_name)
            print(f"  [PASS] All {len(verified_meshes)} unique visual mesh assets return HTTP 200.")

            # 2. Receive live streamed poses
            poses_msg = None
            start_t = time.time()
            while time.time() - start_t < 4.0:
                msg = await ws.receive_json(timeout=2.0)
                if msg.get("type") == "poses":
                    poses_msg = msg.get("poses")
                    break

            if not poses_msg:
                print("  [FAIL] Did not receive streamed link poses.")
                return False

            print(f"  Received poses packet for {len(poses_msg)} links.")

            # Verify all poses are finite
            for k, p in poses_msg.items():
                assert all(np.isfinite(x) for x in p), f"Streamed pose for {k} contains NaN/inf: {p}"
            print("  [PASS] All initial streamed poses are finite and valid.")

            # 3. Check MuJoCo match and Geom Mesh Bounding Box Reconstruction
            import mujoco
            import trimesh
            from scipy.spatial.transform import Rotation as R
            
            scene_xml = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
            from teleop.simulation.mujoco_env import AgibotSimEnv
            ref_env = AgibotSimEnv(scene_xml)
            ref_env.reset()
            model = ref_env.model
            data = ref_env.data
            
            # Fetch status to get current MuJoCo EE
            req = urllib.request.Request(API_STATUS_URL)
            with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
                status_data = json.loads(resp.read().decode("utf-8"))

            base_pose = poses_msg.get("base_link")
            print(f"  Streamed base_link pose: {base_pose}")
            
            # Compare left_arm_link07 streamed pos vs MuJoCo EE pos in status
            left_hand_pose = poses_msg.get("left_arm_link07")
            mu_ee_l = status_data["ee_l"]
            diff_mm = np.linalg.norm(np.array(left_hand_pose[:3]) - np.array(mu_ee_l)) * 1000.0
            print(f"  Left arm link07 stream pos: [{left_hand_pose[0]:.4f}, {left_hand_pose[1]:.4f}, {left_hand_pose[2]:.4f}] m")
            print(f"  MuJoCo EE actual pos      : [{mu_ee_l[0]:.4f}, {mu_ee_l[1]:.4f}, {mu_ee_l[2]:.4f}] m")
            print(f"  Difference: {diff_mm:.3f} mm (Threshold: < 1.0 mm)")

            # Reconstruct world-space mesh bbox center for EVERY visual geom and compare with MuJoCo
            print("\n  [GEOM BBOX SYNC] Verifying all visual geoms world bbox centers (< 1.0 mm)...")
            max_geom_err_mm = 0.0
            total_geoms_tested = 0
            web_meshes_dir = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "web_meshes")

            for gid_str, gpose in poses_msg.items():
                if not gid_str.isdigit():
                    continue
                gid = int(gid_str)
                if model.geom_group[gid] != 1 or model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
                    continue
                
                mid = model.geom_dataid[gid]
                mname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid)
                stl_file = os.path.join(web_meshes_dir, f"{mname}.stl")
                if not os.path.exists(stl_file):
                    continue

                # 1. MuJoCo world bbox center
                vert_adr = model.mesh_vertadr[mid]
                vert_num = model.mesh_vertnum[mid]
                verts_mj = model.mesh_vert[vert_adr : vert_adr + vert_num]
                world_verts_mj = data.geom_xpos[gid] + (data.geom_xmat[gid].reshape(3,3) @ verts_mj.T).T
                mj_bbox_center = (world_verts_mj.min(axis=0) + world_verts_mj.max(axis=0)) / 2.0

                # 2. Reconstruct Three.js world bbox center from streamed geom pose
                # gpose: [x, y, z, qx, qy, qz, qw] (Three.js format)
                p_three = np.array(gpose[:3])
                q_three = np.array(gpose[3:7]) # [x, y, z, w]
                # Assert quaternion order is normalized and valid
                quat_norm = np.linalg.norm(q_three)
                assert abs(quat_norm - 1.0) < 1e-3, f"Invalid quaternion norm for geom {gid}: {quat_norm}"

                rot_three = R.from_quat(q_three).as_matrix()
                mesh_tm = trimesh.load(stl_file)
                world_verts_three = p_three + (rot_three @ mesh_tm.vertices.T).T
                three_bbox_center = (world_verts_three.min(axis=0) + world_verts_three.max(axis=0)) / 2.0

                err_mm = float(np.linalg.norm(three_bbox_center - mj_bbox_center)) * 1000.0
                if err_mm > max_geom_err_mm:
                    max_geom_err_mm = err_mm
                total_geoms_tested += 1

            print(f"  Tested {total_geoms_tested} visual geoms: max bbox error = {max_geom_err_mm:.3f} mm")
            assert total_geoms_tested >= 70, f"Expected >= 70 visual geoms, tested {total_geoms_tested}"
            assert max_geom_err_mm < 1.0, f"Max geom error {max_geom_err_mm:.3f} mm exceeds 1.0 mm threshold!"
            print("  [PASS] All visual geoms world bbox centers match MuJoCo within < 1 mm.")
            
            if diff_mm < 1.0:
                print("  [PASS] Streamed link pose matches MuJoCo simulation (< 1 mm).")
            else:
                print("  [FAIL] Pose difference exceeds 1 mm.")
                return False

            # 4. Drag wrist target marker test
            print("\n[TEST 3] Testing 3D Viewport Target Marker Dragging -> IK -> MuJoCo ...")
            new_target_pos = [mu_ee_l[0] + 0.05, mu_ee_l[1] + 0.03, mu_ee_l[2] + 0.04]
            drag_cmd = {
                "cmd": "set_wrist_target",
                "arm": "left",
                "pos": new_target_pos
            }
            await ws.send_str(json.dumps(drag_cmd))
            await asyncio.sleep(0.8)

            with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
                new_st = json.loads(resp.read().decode("utf-8"))

            new_ee_l = new_st["ee_l"]
            target_err_mm = np.linalg.norm(np.array(new_ee_l) - np.array(new_target_pos)) * 1000.0
            print(f"  Target Marker Position : [{new_target_pos[0]:.3f}, {new_target_pos[1]:.3f}, {new_target_pos[2]:.3f}] m")
            print(f"  MuJoCo Reached EE Pos  : [{new_ee_l[0]:.3f}, {new_ee_l[1]:.3f}, {new_ee_l[2]:.3f}] m")
            print(f"  IK Convergence Error   : {target_err_mm:.2f} mm")
            if target_err_mm < 15.0:
                print("  [PASS] 3D Viewport target dragging drove MuJoCo robot via IK.")
            else:
                print("  [FAIL] Target marker drag did not converge.")
                return False

            # 5. E-STOP Test from Web Interface
            print("\n[TEST 4] Testing Emergency Stop (E-STOP) from Web Interface ...")
            estop_cmd = {"cmd": "estop"}
            await ws.send_str(json.dumps(estop_cmd))
            await asyncio.sleep(0.4)

            with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
                estop_st = json.loads(resp.read().decode("utf-8"))

            if estop_st.get("estop") is True:
                print("  [PASS] E-STOP confirmed active via Web UI.")
            else:
                print("  [FAIL] E-STOP flag not set.")
                return False

            # Clear E-STOP
            await ws.send_str(json.dumps({"cmd": "clear_estop"}))
            await ws.send_str(json.dumps({"cmd": "preset", "pose": "reset"}))
            await asyncio.sleep(0.4)
            print("  Cleared E-STOP and restored home pose.")

            # 6. Motion test (Wave + Circle motions) and re-verify poses are finite
            print("\n[TEST 5] Running 30 s of motion presets and re-verifying all geom poses...")
            # Trigger wave motion
            await ws.send_str(json.dumps({"cmd": "preset", "pose": "wave"}))
            motion_start = time.time()
            rechecked_frames = 0
            while time.time() - motion_start < 30.0:
                # After 15s switch to circle motion
                if time.time() - motion_start > 15.0 and rechecked_frames < 20:
                    await ws.send_str(json.dumps({"cmd": "preset", "pose": "circle"}))
                msg = await ws.receive_json(timeout=2.0)
                if msg.get("type") == "poses":
                    poses = msg.get("poses", {})
                    for gid_k, p_val in poses.items():
                        assert all(np.isfinite(x) for x in p_val), f"Pose for {gid_k} is not finite: {p_val}"
                    rechecked_frames += 1

            # Restore Home pose
            await ws.send_str(json.dumps({"cmd": "preset", "pose": "home"}))
            await asyncio.sleep(0.5)

            # Check health endpoint nan_detected
            health_req = urllib.request.Request(f"{API_BASE_URL}/health")
            with urllib.request.urlopen(health_req, context=ssl_ctx, timeout=3.0) as h_resp:
                health_data = json.loads(h_resp.read().decode("utf-8"))
                assert health_data.get("nan_detected") is False, "Health endpoint reported nan_detected=True!"
                assert health_data.get("status") == "OK", f"Health status error: {health_data}"
            print(f"  [PASS] 30 s of continuous motion completed across {rechecked_frames} pose frames with 0 NaNs and clean health.")

    return True

def main():
    print("=" * 80)
    print("RUNNING AGIBOT A2 ULTRA 3D WEB PANEL & DIGITAL TWIN ACCEPTANCE TEST")
    print("=" * 80)

    from teleop.utils.server_process import ensure_server_running, stop_spawned_server

    spawned_proc = None
    try:
        spawned_proc, did_spawn = ensure_server_running(timeout_s=20.0)

        t1 = test_http_redirect()
        t2 = asyncio.run(test_websocket_stream())

        if t1 and t2:
            print("\n" + "=" * 80)
            print("[SUCCESS] ALL 3D WEB PANEL SYNC AND ACCEPTANCE TESTS PASSED!")
            print("=" * 80)
            return 0
        else:
            print("\n" + "=" * 80)
            print("[FAIL] Some tests failed.")
            print("=" * 80)
            return 1
    finally:
        if spawned_proc:
            stop_spawned_server(spawned_proc)

if __name__ == "__main__":
    sys.exit(main())
