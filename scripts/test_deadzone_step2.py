import sys, time, json, numpy as np
from main import UnifiedTeleopApp

def measure_deadzone():
    app = UnifiedTeleopApp(backend="sim", render_viewer=False, no_adb=True, no_quest_launch=True)
    # Start headless
    # Test for displacements 0.1, 0.3, 0.6 m
    print("Measuring current A2 behavior...")
    # Initial EE
    ee0 = app.env.get_ee_poses()["left"]["pos"]
    sh0 = np.array([-0.03779, 0.19100, 1.27210])
    dir_vec = np.array([1.0, 0.0, 0.0]) # forward +X
    # Max reach ~ 0.50 m
    # Let reach boundary be sh0 + dir_vec * 0.50
    boundary = sh0 + dir_vec * 0.50
    
    results = {}
    for d_beyond in [0.1, 0.3, 0.6]:
        target_out = boundary + dir_vec * d_beyond
        # Simulate gizmo drag to target_out
        app.cur_target_pos_l = target_out.copy()
        app.gizmo_active_l = True
        app._solve_arm_ik("left")
        
        # Release gizmo
        from teleop.app.ik_dispatcher import freeze_arm_target
        freeze_arm_target(app, "left")
        app.gizmo_active_l = False
        
        # Measure for 3s after release
        plan_q = app.last_arm_cmd_left.copy()
        # In current code: where is gizmo, where is arm?
        gizmo_pos = app.cur_target_pos_l.copy()
        arm_ee = app.env.get_ee_poses()["left"]["pos"]
        dist_release = float(np.linalg.norm(gizmo_pos - arm_ee))
        
        # Now drag back toward boundary at 0.01 m increments
        # Measure deadzone length: distance dragged back until arm plan changes > 0.5 mm
        deadzone_len = 0.0
        arm_moved = False
        steps = int(d_beyond / 0.005) + 20
        curr_p = target_out.copy()
        for i in range(steps):
            curr_p -= dir_vec * 0.005
            app.cur_target_pos_l = curr_p.copy()
            app.gizmo_active_l = True
            app._solve_arm_ik("left")
            new_plan = app.last_arm_cmd_left.copy()
            if not arm_moved:
                if np.max(np.abs(new_plan - plan_q)) > 0.005:
                    arm_moved = True
                    deadzone_len = float(np.linalg.norm(target_out - curr_p))
                    break
        results[d_beyond] = {
            "gizmo_at_release": gizmo_pos.tolist(),
            "arm_ee_at_release": arm_ee.tolist(),
            "dist_release_m": dist_release,
            "deadzone_m": deadzone_len if arm_moved else d_beyond
        }
    print(json.dumps(results, indent=2))

if __name__ == "__main__":
    measure_deadzone()
