"""grab_views.py OUTDIR : save one frame each of the camera, the BEV and the lane debug image, and print the lane state/target."""
import sys, os, time, rclpy, numpy as np, cv2
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt8
from geometry_msgs.msg import PointStamped
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
rclpy.init(); n = rclpy.create_node('grab_views'); got = {}
def img(name):
    def cb(m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8': a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        got[name] = a
    return cb
def jpg(m): got['camera'] = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
cam = sys.argv[2] if len(sys.argv) > 2 else '/camera_lane/image_raw'
n.create_subscription(CompressedImage, cam + '/compressed', jpg, 1)
n.create_subscription(Image, '/camera/image_projected', img('bev'), 1)
n.create_subscription(Image, '/detect/image_lane', img('lane'), 1)
n.create_subscription(UInt8, '/detect/lane_state', lambda m: got.__setitem__('state', m.data), 1)
n.create_subscription(PointStamped, '/detect/lane_target', lambda m: got.__setitem__('target', (m.point.x, m.point.y)), 1)
t0 = time.time()
while time.time() - t0 < 12 and not all(k in got for k in ('camera', 'bev', 'lane')): rclpy.spin_once(n, timeout_sec=0.2)
for k in ('camera', 'bev', 'lane'):
    if k in got: cv2.imwrite(f'{out}/{k}.png', got[k]); print(f'{k}: {got[k].shape[1]}x{got[k].shape[0]} -> {out}/{k}.png')
    else: print(f'{k}: no image')
print('lane_state', got.get('state', 'none'), 'target', got.get('target', 'none'))
