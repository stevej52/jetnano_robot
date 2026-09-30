#!/usr/bin/env python3
"""Ask Claude, through the brain (brain/look), about a picture packet - exactly as nav_helper's
~/ask does, but with pictures from a directory (stuck_demo.py's). Prints the reply and the
checked advice.     python3 ask_demo.py DIR "situation text"
"""
import json
import os
import sys
import time
import uuid

import rclpy
from jetnano_navigation import stuck_help as sh
from std_msgs.msg import String


def main():
    d, situation = sys.argv[1], sys.argv[2]
    images = [{'path': os.path.join(d, 'house_map.png'), 'label': '1. HOUSE MAP around her (north up, metres)'},
              {'path': os.path.join(d, 'close_up.png'), 'label': '2. CLOSE-UP of what is around her (her front up)'},
              {'path': os.path.join(d, 'cameras.jpg'), 'label': '3. PHOTOS from her cameras, each labelled'}]
    rclpy.init()
    n = rclpy.create_node('ask_demo')
    got = {}
    rid = uuid.uuid4().hex[:12]
    n.create_subscription(String, 'brain/look_answer',
                          lambda m: got.update(json.loads(m.data)) if json.loads(m.data).get('id') == rid else None, 10)
    pub = n.create_publisher(String, 'brain/look', 10)
    end = time.time() + 5
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    m = String()
    m.data = json.dumps({'id': rid, 'system': sh.SYSTEM, 'prompt': sh.PROMPT.format(situation=situation),
                         'images': images, 'effort': 'medium', 'max_tokens': 4000})
    t0 = time.time()
    pub.publish(m)
    while not got and time.time() - t0 < 180:
        rclpy.spin_once(n, timeout_sec=0.2)
    print(f'{time.time() - t0:.1f} s')
    print('RAW:', got.get('text') or got.get('error') or 'no answer')
    print('ADVICE:', json.dumps(sh.parse_advice(got.get('text', '')), indent=1))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
