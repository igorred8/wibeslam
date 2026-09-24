#!/bin/bash
# Аналог yahboom save_map_launch.py: сохраняет текущую карту в
# /root/maps/<имя>.pgm + <имя>.yaml в том же формате (image/mode/
# resolution/origin/negate/occupied_thresh/free_thresh).
# Использование: ./save_map.sh [имя]   (по умолчанию palmslam_map)
set -e
source /opt/ros/humble/setup.bash
NAME=${1:-palmslam_map}
OUT=/root/maps
mkdir -p "$OUT"

MAPFILE="$OUT/$NAME.pgm"
YAMLFILE="$OUT/$NAME.yaml"

echo "Saving map as $NAME -> $OUT ..."
timeout 60 ros2 service call /map_server/map -s nav2_msgs/srv/GetMap >/dev/null 2>&1 || true

# Карту отдаёт cartographer_occupancy_grid_node в /map; пишем через python-yaml + PIL
python3 - "$NAME" "$OUT" <<'PY'
import sys, numpy as np, yaml
name, out = sys.argv[1], sys.argv[2]
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from PIL import Image

rclpy.init()
node = Node('wibeslam_save_map')
got = {}
def cb(m): got['m'] = m
sub = node.create_subscription(OccupancyGrid, '/map', cb, 1)
import time
t0 = time.time()
while 'm' not in got and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.2)
if 'm' not in got:
    print('ERROR: no /map received'); sys.exit(1)
m = got['m']
res = m.info.resolution
arr = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
img = np.where(arr < 0, 205, np.where(arr == -1, 205, 254 - arr * 2)).astype(np.uint8)
Image.fromarray(img[::-1]).save(f'{out}/{name}.pgm')
ox = m.info.origin.position.x
oy = m.info.origin.position.y
with open(f'{out}/{name}.yaml', 'w') as f:
    yaml.dump({'image': f'{name}.pgm', 'mode': 'trinary',
               'resolution': float(res),
               'origin': [float(ox), float(oy), 0.0],
               'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196},
              f, default_flow_style=False)
print(f'Saved {out}/{name}.pgm and {out}/{name}.yaml')
node.destroy_node()
PY
