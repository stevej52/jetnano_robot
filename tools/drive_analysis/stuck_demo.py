"""nav_helper's picture packet rebuilt from a recording, for a moment she was stuck.

    python stuck_demo.py BAG.mcap HEADING.npz FRAMES_DIR 2026-09-29T23:17:00Z GOAL_X GOAL_Y OUTDIR

Uses stuck_help's renderers exactly as nav_helper does: the house map (SLAM's /map, the
planner's /plan, her EKF track turned into the map frame by the recorded map -> odom, the
goal), the close-up (the /scan and the camera grid laid on the map, turned into her frame)
and a sheet of the camera pictures drive_log saved in that second (there was no pan-tilt
sweep on the recording: front, depth and rear camera instead).
"""
import glob
import math
import os
import sys
from datetime import datetime, timezone

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'jetnano_navigation'))
from jetnano_navigation import stuck_help as sh  # noqa: E402


def main(bag, npz, frames, when, gx, gy, out):
    os.makedirs(out, exist_ok=True)
    t = datetime.strptime(when, '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()
    latest = {}
    dec = DecoderFactory()
    decs = {}
    topics = ['/map', '/nvblox_node/map_grid', '/scan', '/plan']
    with open(bag, 'rb') as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=topics, log_time_order=True):
            if msg.log_time * 1e-9 > t:
                break
            d = decs.get(ch.id) or decs.setdefault(ch.id, dec.decoder_for('cdr', schema))
            latest[ch.topic] = d(msg.data)
    h = np.load(npz)
    ekf, mo = h['ekf'], h['mapodom']
    ekf = ekf[ekf[:, 0] <= t]
    mo = mo[mo[:, 0] <= t][-1]
    odom_pose = (ekf[-1, 1], ekf[-1, 2], ekf[-1, 3])
    m2o = (mo[1], mo[2], mo[3])

    def to_map(p):
        c, s = math.cos(m2o[2]), math.sin(m2o[2])
        return (m2o[0] + c * p[0] - s * p[1], m2o[1] + s * p[0] + c * p[1])
    pose_map = to_map(odom_pose) + (sh.wrap(m2o[2] + odom_pose[2]),)
    # her track: the last 12 m, a point every 5 cm
    track, dist = [], 0.0
    for row in ekf[::-1]:
        p = (row[1], row[2])
        if track:
            step = math.hypot(track[-1][0] - p[0], track[-1][1] - p[1])
            if step < 0.05:
                continue
            dist += step
        track.append(p)
        if dist > 12.0:
            break
    track = track[::-1]
    m = latest['/map']
    grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    info = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
    plan = [(q.pose.position.x, q.pose.position.y) for q in latest['/plan'].poses] if '/plan' in latest else []
    s = latest['/scan']
    scan = sh.scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
    cg = latest['/nvblox_node/map_grid']
    cells = sh.grid_cells(np.array(cg.data, dtype=np.int16).reshape(cg.info.height, cg.info.width),
                          (cg.info.resolution, cg.info.origin.position.x, cg.info.origin.position.y))
    base = grid >= 65
    # camera cells that are not already the house map's walls: what the lidar/map miss
    cam_only = [c for c in cells
                if not base[int((c[1] - info[2]) / info[0]), int((c[0] - info[1]) / info[0])]]
    house = sh.render_house(grid, info, pose_map, [to_map(p) for p in track], plan, (gx, gy), obstacles=cam_only)
    cv2.imwrite(os.path.join(out, 'house_map.png'), house)
    cam = [c for c in sh.to_robot(cam_only, pose_map) if abs(c[0]) < 2.0 and abs(c[1]) < 2.0]
    close = sh.render_close(scan, cam, sh.to_robot(track[-60:], odom_pose))
    cv2.imwrite(os.path.join(out, 'close_up.png'), close)
    stamp = datetime.fromtimestamp(t).strftime('%H%M%S')        # drive_log names frames in local time
    views = []
    for cam_name, label in (('front', 'pan-tilt camera, pointing ahead'), ('d435', 'depth camera, fixed, ahead'),
                            ('rear', 'rear camera, behind her')):
        hits = sorted(glob.glob(os.path.join(frames, cam_name, stamp[:-1] + '*.jpg')))
        near = [f for f in hits if os.path.basename(f)[:6] <= stamp]
        views.append((label, cv2.imread(near[-1]) if near else None))
    cv2.imwrite(os.path.join(out, 'cameras.jpg'), sh.contact_sheet(views), [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f'pose on the map ({pose_map[0]:+.2f}, {pose_map[1]:+.2f}, {math.degrees(pose_map[2]):+.0f} deg); '
          f'{len(track)} track points; {len(plan)} plan points; {len(scan)} lidar hits; {len(cam)} camera cells near')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5]), float(sys.argv[6]), sys.argv[7])
