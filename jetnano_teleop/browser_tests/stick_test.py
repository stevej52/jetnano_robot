import json
import time

from playwright.sync_api import sync_playwright


def status(pg):
    return pg.evaluate("fetch('/status',{cache:'no-store'}).then(r=>r.json())")


out = {}
with sync_playwright() as p:
    b = p.chromium.launch(channel='chrome', headless=True, args=['--no-sandbox'])
    pg = b.new_page(viewport={'width': 480, 'height': 900})
    errors = []
    pg.on('pageerror', lambda e: errors.append('pageerror: ' + str(e)))
    pg.on('console', lambda m: errors.append('console: ' + m.text) if m.type == 'error' else None)
    pg.goto('http://192.168.1.7:8081/', wait_until='domcontentloaded')
    time.sleep(1.5)
    box = pg.locator('#stick').bounding_box()
    cx, cy, R = box['x'] + box['width'] / 2, box['y'] + box['height'] / 2, box['width'] / 2
    travel = R - 0.34 * R
    # 1. knob to the upper right, outside the circle: expect forward + right turn, clamped
    pg.mouse.move(cx, cy)
    pg.mouse.down()
    pg.mouse.move(cx + 0.5 * R, cy - 0.5 * R, steps=6)
    time.sleep(0.6)
    out['held_upper_right'] = status(pg)
    out['readout'] = pg.locator('#readout').inner_text()
    pg.screenshot(path='/tmp/web-stick-held.png')
    # 2. small push straight forward (40 % of travel)
    pg.mouse.move(cx, cy - 0.4 * travel, steps=4)
    time.sleep(0.5)
    out['held_forward_40pct'] = status(pg)
    # 3. let go
    pg.mouse.up()
    time.sleep(0.9)
    out['after_release'] = status(pg)
    # 4. keyboard
    pg.keyboard.down('ArrowUp')
    pg.keyboard.down('ArrowLeft')
    time.sleep(0.5)
    out['keys_up_left'] = status(pg)
    pg.keyboard.up('ArrowUp')
    pg.keyboard.up('ArrowLeft')
    time.sleep(0.9)
    out['keys_released'] = status(pg)
    # 5. STOP, try to drive, GO
    pg.click('#stop')
    time.sleep(0.4)
    out['after_stop'] = status(pg)
    pg.mouse.move(cx, cy)
    pg.mouse.down()
    pg.mouse.move(cx, cy - 0.5 * R, steps=4)
    time.sleep(0.5)
    out['drive_while_stopped'] = status(pg)
    pg.mouse.up()
    pg.click('#stop')
    time.sleep(0.4)
    out['after_go'] = status(pg)
    pg.screenshot(path='/tmp/web-stick.png')
    out['errors'] = errors
    b.close()
print(json.dumps(out, indent=1))
