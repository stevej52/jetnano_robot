"""ASCII picture of the live global costmap in a map-frame window: python3 gap_probe.py X0 X1 Y0 Y1
'#' lethal (254), 'o' inscribed (253), '+' cost >= 128, '.' cost 1-127, ' ' free, '?' unknown.
Each cell 10 cm. Prints the widest free run per row too (the gap she has to fit through)."""
import sys
import numpy as np
import rclpy
from nav2_msgs.srv import GetCostmap

x0, x1, y0, y1 = [float(v) for v in sys.argv[1:5]]
rclpy.init()
n = rclpy.create_node('gap_probe')
cli = n.create_client(GetCostmap, '/global_costmap/get_costmap')
assert cli.wait_for_service(timeout_sec=5.0), 'no global costmap service'
fut = cli.call_async(GetCostmap.Request())
rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
cm = fut.result().map
md = cm.metadata
grid = np.asarray(cm.data, dtype=np.int16).reshape(md.size_y, md.size_x)
res, ox, oy = md.resolution, md.origin.position.x, md.origin.position.y
print(f'costmap {md.size_x}x{md.size_y} at {res:.2f} m, origin ({ox:.2f}, {oy:.2f})')
sym = lambda c: '?' if c == 255 else '#' if c == 254 else 'o' if c == 253 else '+' if c >= 128 else '.' if c > 0 else ' '  # noqa: E731
ys = np.arange(y1, y0 - 1e-9, -0.1)
xs = np.arange(x0, x1 + 1e-9, 0.1)
print('      ' + ''.join(f'{x:+.0f}'.rjust(10) if abs(x - round(x)) < 0.05 else '' for x in xs))
for y in ys:
    row = ''
    for x in xs:
        i, j = int((x - ox) / res), int((y - oy) / res)
        c = grid[j, i] if 0 <= i < md.size_x and 0 <= j < md.size_y else 255
        row += sym(int(c))
    # widest run of cells below the inscribed cost (she can stand there)
    free = [ch in ' .' for ch in row]
    best = run = 0
    for f in free:
        run = run + 1 if f else 0
        best = max(best, run)
    print(f'{y:+5.1f} |{row}|  widest low-cost run {best * 10:3d} cm')
n.destroy_node()
rclpy.shutdown()
