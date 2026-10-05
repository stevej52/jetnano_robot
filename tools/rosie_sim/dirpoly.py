# the direction-aware collision monitor: replaces the all-round approach box
cm = p["collision_monitor"]["ros__parameters"]
def box(x0, x1, w=ZW):
    return f"[[{x1}, {w}], [{x1}, -{w}], [{x0}, -{w}], [{x0}, {w}]]"
cm["polygons"] = ["DirectionalStop"]
cm["DirectionalStop"] = {
    "type": "velocity_polygon", "action_type": "stop", "min_points": 4, "visualize": False, "enabled": True,
    "holonomic": False, "polygon_pub_topic": "directional_stop",
    "velocity_polygons": ["fwd_slow", "fwd_fast", "rev_slow", "rev_fast"],
    "fwd_slow": {"points": box(0.12, 0.222 + ZS), "linear_min": 0.0, "linear_max": 0.25, "theta_min": -3.0, "theta_max": 3.0},
    "fwd_fast": {"points": box(0.12, 0.222 + ZF), "linear_min": 0.25, "linear_max": 1.0, "theta_min": -3.0, "theta_max": 3.0},
    "rev_slow": {"points": box(-0.222 - ZS, -0.12), "linear_min": -0.25, "linear_max": 0.0, "theta_min": -3.0, "theta_max": 3.0},
    "rev_fast": {"points": box(-0.222 - ZF, -0.12), "linear_min": -1.0, "linear_max": -0.25, "theta_min": -3.0, "theta_max": 3.0},
}
