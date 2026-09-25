import json
import time

from playwright.sync_api import sync_playwright


def status(pg):
    return pg.evaluate("fetch('/status',{cache:'no-store'}).then(r=>r.json())")


def push_forward(pg, cx, cy, travel, seconds):
    pg.mouse.move(cx, cy)
    pg.mouse.down()
    pg.mouse.move(cx, cy - 0.9 * travel, steps=4)
    time.sleep(seconds)
    st = status(pg)
    pg.mouse.up()
    time.sleep(0.9)
    return st


out = {}
with sync_playwright() as p:
    b = p.chromium.launch(channel='chrome', headless=True, args=['--no-sandbox'])
    pg = b.new_page(viewport={'width': 430, 'height': 900})
    errors = []
    pg.on('pageerror', lambda e: errors.append('pageerror: ' + str(e)))
    pg.on('console', lambda m: errors.append('console: ' + m.text) if m.type == 'error' else None)
    pg.goto('http://192.168.1.7:8081/', wait_until='domcontentloaded')
    time.sleep(2.0)
    box = pg.locator('#stick').bounding_box()
    cx, cy, R = box['x'] + box['width'] / 2, box['y'] + box['height'] / 2, box['width'] / 2
    travel = R - 0.34 * R
    out['switch_at_load'] = pg.locator('#guard').inner_text()
    out['forward_guard_on'] = push_forward(pg, cx, cy, travel, 1.2)
    pg.click('#guard')
    time.sleep(1.0)
    out['switch_after_click'] = pg.locator('#guard').inner_text()
    out['forward_guard_off'] = push_forward(pg, cx, cy, travel, 1.2)
    pg.screenshot(path='/tmp/web-guard-off.png')
    pg.click('#guard')
    time.sleep(1.0)
    out['switch_after_second_click'] = pg.locator('#guard').inner_text()
    out['forward_guard_on_again'] = push_forward(pg, cx, cy, travel, 1.2)
    pg.screenshot(path='/tmp/web-guard-on.png')
    out['errors'] = errors
    b.close()
print(json.dumps(out, indent=1))
