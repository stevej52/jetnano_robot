"""Rosie's live ROS graph as an rqt_graph-style diagram: nodes (ovals) -> topics (boxes) -> nodes.

    python3 tools/rosie_graph.py OUT_BASENAME   -> OUT.dot, OUT.svg, OUT.png, OUT.json   (on Rosie: needs
    ROS sourced, ROS_DOMAIN_ID=7 and Graphviz dot; Steve remembered rqt_graph, 2026-10-13)

Hides what rqt_graph hides by default (debug: rosout/parameter_events; tf; topics with no
publisher or no subscriber), plus launch/tf-listener helper nodes, and groups nodes by subsystem.
"""
import json
import re
import subprocess
import sys
import time

import rclpy

HIDE_TOPICS = re.compile(r'^/(rosout|parameter_events|tf|tf_static|clock)$|/transition_event$|/_action/|^/bond$|'
                         r'/describe_parameters|/get_type_description')
HIDE_NODES = re.compile(r'^/(transform_listener_impl_|launch_ros_|_ros2cli|rosie_graph)|_rclcpp_node$|'
                        r'^/(nav2_container|nvblox_container)$')
GROUPS = [
    ('Sensors', '#dbeafe', r'rplidar|scan_filter|bno055|^/camera$|realsense_splitter|csi_cameras|battery_monitor|cliff'),
    ('Odometry and localisation', '#dcfce7', r'visual_slam|ekf|mola|lidar_odom|where_am_i|slam_toolbox|state_publisher|joint_state|map_server|amcl'),
    ('3D map', '#fef9c3', r'nvblox|grid_to_points'),
    ('Navigation (Nav2 + mission)', '#fde68a', r'bt_navigator|planner_server|controller_server|behavior_server|velocity_smoother|'
                                               r'collision_monitor|costmap|lifecycle_manager_navigation|nav_translator|near_cap|'
                                               r'mission|nav_helper|nav_park|battery_home|route|docking|waypoint'),
    ('Safety', '#fecaca', r'safety_gate|safety_monitor|collision_guard|lifecycle_manager_guard|guard_flow|topic_watch|watchdog|'
                          r'sensor_sleep|motion_check'),
    ('Driving and head', '#e9d5ff', r'twist_mux|pca9685|head_pwm|stop_settle|web_teleop|teleop|motion_watch'),
    ('Voice and brain', '#fbcfe8', r'brain|listen|speak|sounds|ears|settle|people|meet'),
]


def group_of(node):
    for name, color, rx in GROUPS:
        if re.search(rx, node):
            return name
    return 'Other'


def main():
    out = sys.argv[1]
    rclpy.init()
    n = rclpy.create_node('rosie_graph')
    end = time.time() + 6.0                     # let discovery fill in
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    pubs, subs, nodes = {}, {}, set()
    for name, ns in n.get_node_names_and_namespaces():
        full = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
        if HIDE_NODES.search(full):
            continue
        try:
            p = n.get_publisher_names_and_types_by_node(name, ns)
            s = n.get_subscriber_names_and_types_by_node(name, ns)
        except Exception:  # noqa: BLE001 - a node that left meanwhile
            continue
        nodes.add(full)
        for t, _ in p:
            if not HIDE_TOPICS.search(t):
                pubs.setdefault(t, set()).add(full)
        for t, _ in s:
            if not HIDE_TOPICS.search(t):
                subs.setdefault(t, set()).add(full)
    n.destroy_node()
    rclpy.shutdown()
    # a topic needs a writer and a reader, and not just one node talking to itself
    topics = sorted(t for t in pubs if t in subs and not (pubs[t] == subs[t] and len(pubs[t]) == 1))
    used = set()
    for t in topics:
        used |= pubs[t] | subs[t]
    json.dump({'nodes': sorted(nodes), 'topics': {t: {'pub': sorted(pubs[t]), 'sub': sorted(subs[t])} for t in topics}},
              open(out + '.json', 'w'), indent=1)
    color = {g: c for g, c, _ in GROUPS}
    color['Other'] = '#e5e7eb'
    lines = ['digraph rosie {', '  rankdir=LR; splines=true; concentrate=true; nodesep=0.08; ranksep=1.1;',
             '  graph [fontname="Helvetica", fontsize=22, labelloc=t,'
             f' label="Rosie - live ROS 2 graph, {time.strftime("%Y-%m-%d %H:%M")}  ({len(used)} nodes, {len(topics)} topics; '
             'hidden: rosout, parameters, tf, topics without a reader or a writer)"];',
             '  node [fontname="Helvetica", fontsize=11]; edge [color="#6b7280", arrowsize=0.6];']
    for nd in sorted(used):
        g = group_of(nd)
        lines.append(f'  "{nd}" [shape=ellipse, style=filled, fillcolor="{color[g]}", penwidth=1.2, fontsize=13, '
                     f'label="{nd.lstrip("/")}"];')
    legend = ''.join(f'<TR><TD BGCOLOR="{color[g]}">{g}</TD></TR>' for g, _, _ in GROUPS)
    lines.append('  legend [shape=plaintext, fontsize=14, label=<<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" '
                 f'CELLPADDING="6"><TR><TD><B>Subsystems</B></TD></TR>{legend}'
                 '<TR><TD>ovals = nodes, boxes = topics</TD></TR></TABLE>>];')
    for t in topics:
        tid = 'T' + t
        lines.append(f'  "{tid}" [shape=box, style="rounded", fontsize=9, color="#9ca3af", fontcolor="#374151", label="{t}"];')
        for p in sorted(pubs[t]):
            lines.append(f'  "{p}" -> "{tid}";')
        for s in sorted(subs[t]):
            lines.append(f'  "{tid}" -> "{s}";')
    lines.append('}')
    open(out + '.dot', 'w').write('\n'.join(lines))
    for fmt in ('svg', 'png'):
        subprocess.run(['dot', f'-T{fmt}', '-Gdpi=60' if fmt == 'png' else '-Gdpi=72', out + '.dot', '-o', f'{out}.{fmt}'], check=True)
    print(f'{len(used)} nodes, {len(topics)} topics -> {out}.svg / .png')


if __name__ == '__main__':
    main()
