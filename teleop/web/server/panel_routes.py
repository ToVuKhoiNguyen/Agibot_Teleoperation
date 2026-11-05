"""
teleop/web/server/panel_routes.py
HTTP REST and HTML page request handlers for the teleop control panel.
"""

from aiohttp import web


async def handle_health_impl(server, request):
    with server.status_lock:
        st = server.latest_status.copy()
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
    if hasattr(server, "quest_manager") and server.quest_manager is not None:
        q_st = server.quest_manager.get_status()
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


async def handle_panel_page(html_content: str, request):
    return web.Response(text=html_content, content_type="text/html")


async def handle_vr_page(html_content: str, request):
    return web.Response(
        text=html_content,
        content_type="text/html",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0"
        }
    )


async def handle_post_quest_launch_impl(quest_manager, request):
    if quest_manager is not None:
        ok = quest_manager.launch_quest_browser(force=True)
        return web.json_response({"status": "ok" if ok else "failed"})
    return web.json_response({"status": "error", "message": "Quest ADB manager not available"}, status=503)


async def handle_post_cmd_impl(cmd_queue, request):
    try:
        data = await request.json()
        cmd_queue.put(data)
        return web.json_response({"status": "ok"})
    except Exception as e:
        return web.json_response({"status": "error", "message": str(e)}, status=400)


async def handle_get_status_impl(status_lock, latest_status, request):
    with status_lock:
        return web.json_response(latest_status)

