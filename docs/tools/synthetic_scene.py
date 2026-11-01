# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""Synthetic table scene for the documentation: feeds the real surface_detector and approach_planner.

Ray-casts an aligned depth image and object mask (replacing the camera and DINO/SAM), back-projects them
with the real utils.project_depth_to_pc_torch, then publishes the SegmentedPointcloud, a costmap built with
the Nav2 inflation law, and the static TFs. All the node outputs are saved to a JSON file.

Usage: synthetic_scene.py <out.json> <scenario>   (use run_synthetic_scene.sh, it also starts the nodes)
The nodes must be restarted for each scenario: a re-sent static TF did not reach them in testing.
"""
import json, math, os, sys, time
import numpy as np, torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav2_msgs.msg import Costmap
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster
from surface_detector_interfaces.msg import SegmentedPointcloud, DetectionResults
from scipy.spatial.transform import Rotation as Rot
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'object_detection'))
import utils

W, H, FX, FY, CX, CY = 640, 480, 386.0, 386.0, 321.5, 238.5
CALIB = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float32)
TABLE_Z, CUP_R, CUP_H = 0.75, 0.04, 0.12
RES, ORIGIN, SIZE = 0.05, (-2.0, -5.0), 200


def cam_pose(robot, pitch_deg, height=1.2):
    x, y, yaw = robot
    p = math.radians(pitch_deg)
    f = np.array([math.cos(p), 0, -math.sin(p)]); r = np.array([0, -1.0, 0]); d = np.cross(f, r)
    R_base_cam = np.stack([r, d, f], 1)                 # optical: x right, y down, z forward
    c, s = math.cos(yaw), math.sin(yaw)
    R_map_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    t_base_cam = np.array([0.05, 0, height])
    return R_base_cam, t_base_cam, R_map_base @ R_base_cam, np.array([x, y, 0]) + R_map_base @ t_base_cam


def render(sc, rng):
    _, _, R, o = cam_pose(sc['robot'], sc['pitch'])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, float)], -1)    # z-depth rays
    r = d @ R.T
    best = np.full((H, W), np.inf); oid = np.zeros((H, W), int)
    t = -o[2] / np.minimum(r[..., 2], -1e-9); t[r[..., 2] >= 0] = np.inf        # floor
    best = np.minimum(best, t)
    for x0, x1, y0, y1 in sc['tables']:                                         # table tops
        t = (TABLE_Z - o[2]) / np.minimum(r[..., 2], -1e-9); t[r[..., 2] >= 0] = np.inf
        px, py = o[0] + t * r[..., 0], o[1] + t * r[..., 1]
        hit = (px > x0) & (px < x1) & (py > y0) & (py < y1) & (t < best)
        best[hit] = t[hit]; oid[hit] = 1
    # cup: lateral surface and top cap
    cx, cy = sc['cup']
    ox, oy = o[0] - cx, o[1] - cy
    a = r[..., 0]**2 + r[..., 1]**2; b = 2 * (ox * r[..., 0] + oy * r[..., 1]); c = ox**2 + oy**2 - CUP_R**2
    disc = b**2 - 4 * a * c
    t = np.where(disc > 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
    z = o[2] + t * r[..., 2]
    hit = (t > 0) & (z > TABLE_Z) & (z < TABLE_Z + CUP_H) & (t < best)
    best[hit] = t[hit]; oid[hit] = 2
    t = (TABLE_Z + CUP_H - o[2]) / np.minimum(r[..., 2], -1e-9); t[r[..., 2] >= 0] = np.inf
    hit = ((o[0] + t * r[..., 0] - cx)**2 + (o[1] + t * r[..., 1] - cy)**2 < CUP_R**2) & (t < best)
    best[hit] = t[hit]; oid[hit] = 2
    depth = best.astype(np.float32)
    depth = np.where(np.isfinite(depth), depth + rng.normal(0, 0.003, depth.shape).astype(np.float32), 0)
    return depth.astype(np.float32), (oid == 2).astype(np.uint8)


def costmap_cells(sc):
    occ = np.zeros((SIZE, SIZE), bool)     # [y, x]
    gx = ORIGIN[0] + (np.arange(SIZE) + 0.5) * RES; gy = ORIGIN[1] + (np.arange(SIZE) + 0.5) * RES
    X, Y = np.meshgrid(gx, gy)
    for x0, x1, y0, y1 in sc['tables'] + sc.get('extra_obstacles', []):
        occ |= (X > x0) & (X < x1) & (Y > y0) & (Y < y1)
    dist = ndimage.distance_transform_edt(~occ) * RES
    ins, k = 0.1, 1.5
    cost = (252 * np.exp(-k * (dist - ins))).astype(np.uint8)
    cost[dist <= ins] = 253
    cost[occ] = 254
    return cost


class Scene(Node):
    def __init__(self):
        super().__init__('scene')
        self.tfb = StaticTransformBroadcaster(self)
        self.seg = self.create_publisher(SegmentedPointcloud, '/object_detector/segmented_pointcloud', 10)
        self.cm = self.create_publisher(Costmap, '/global_costmap/costmap_raw', 10)
        self.r = {}
        self.create_subscription(DetectionResults, '/surface_detector/results', self.on_res, 10)
        self.create_subscription(Marker, '/approach_planner/candidate_goals_marker', lambda m: self.on_mk('cand', m), 10)
        self.create_subscription(Marker, '/approach_planner/filtered_candidate_marker', lambda m: self.on_mk('filt', m), 10)
        self.create_subscription(PointCloud2, '/surface_detector/closer_cluster', lambda m: self.on_pc('cluster', m), 10)
        self.create_subscription(PointCloud2, '/surface_detector/detected_plane', lambda m: self.on_pc('plane', m), 10)
        self.create_subscription(PoseStamped, '/approach_planner/goal_pose', self.on_goal, 10)

    def on_res(self, m):
        self.r['contour'] = [[p.point.x, p.point.y, p.point.z] for p in m.surface_contours]
        self.r['object_base'] = [m.segmented_object.point.x, m.segmented_object.point.y, m.segmented_object.point.z]
        self.r['res_stamp'] = m.segmented_object.header.stamp.sec + 1e-9 * m.segmented_object.header.stamp.nanosec

    def on_mk(self, k, m):
        self.r[k] = [[p.x, p.y] for p in m.points]

    def on_pc(self, k, m):
        pts = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
        off = {f.name: f.offset for f in m.fields}
        xyz = np.stack([np.frombuffer(pts[:, off[c]:off[c] + 4].tobytes(), np.float32) for c in 'xyz'], 1)
        self.r[k + '_n'] = len(xyz); self.r[k] = xyz[::max(1, len(xyz) // 4000)].tolist()

    def on_goal(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        self.r['goal'] = [m.pose.position.x, m.pose.position.y, yaw, m.header.frame_id,
                          m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec]

    def set_tf(self, sc):
        Rbc, tbc, _, _ = cam_pose(sc['robot'], sc['pitch'])
        now = self.get_clock().now().to_msg()
        a = TransformStamped(); a.header.stamp = now; a.header.frame_id = 'map'; a.child_frame_id = 'geometric_unicycle'
        a.transform.translation.x, a.transform.translation.y = sc['robot'][:2]
        q = Rot.from_euler('z', sc['robot'][2]).as_quat()
        a.transform.rotation.x, a.transform.rotation.y, a.transform.rotation.z, a.transform.rotation.w = map(float, q)
        b = TransformStamped(); b.header.stamp = now; b.header.frame_id = 'geometric_unicycle'; b.child_frame_id = 'cam_opt'
        b.transform.translation.x, b.transform.translation.y, b.transform.translation.z = map(float, tbc)
        q = Rot.from_matrix(Rbc).as_quat()
        b.transform.rotation.x, b.transform.rotation.y, b.transform.rotation.z, b.transform.rotation.w = map(float, q)
        self.tfb.sendTransform([a, b])

    def costmap_msg(self, cost):
        m = Costmap(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.metadata.size_x = m.metadata.size_y = SIZE; m.metadata.resolution = RES
        m.metadata.origin.position.x, m.metadata.origin.position.y = ORIGIN
        m.data = cost.flatten().tolist()
        return m


SCENARIOS = {
    'baseline':    dict(robot=(0.6, 0.0, 0.0), pitch=35, cup=(1.85, 0.25),
                        tables=[(1.6, 2.4, -0.7, 0.7), (1.6, 2.4, -1.5, -0.9)]),
    'fov_clipped': dict(robot=(1.3, 0.0, 0.0), pitch=20, cup=(1.85, 0.25),
                        tables=[(1.6, 2.4, -0.7, 0.7), (1.6, 2.4, -1.5, -0.9)]),
    'blocked':     dict(robot=(0.6, 0.0, 0.0), pitch=35, cup=(1.85, 0.45),
                        tables=[(1.6, 2.4, -0.7, 0.7), (1.6, 2.4, -1.5, -0.9)],
                        extra_obstacles=[(1.05, 1.6, -0.75, 0.95)]),
    'straight':    dict(robot=(0.6, 0.25, 0.0), pitch=35, cup=(1.85, 0.25),
                        tables=[(1.6, 2.4, -0.7, 0.7), (1.6, 2.4, -1.5, -0.9)]),
    'two_sided':   dict(robot=(0.6, 0.0, 0.0), pitch=35, cup=(1.9, 0.25),
                        tables=[(1.6, 2.1, -0.7, 0.7)]),
    'too_far':     dict(robot=(0.6, 0.0, 0.0), pitch=30, cup=(2.3, 0.0),
                        tables=[(1.6, 3.2, -0.9, 0.9)]),
}

rclpy.init()
n = Scene()
out = {}
rng = np.random.default_rng(0)
ex = rclpy.executors.SingleThreadedExecutor(); ex.add_node(n)
def spin(s):
    end = time.time() + s
    while time.time() < end:
        ex.spin_once(timeout_sec=0.02)
spin(4.0)
for name in sys.argv[2:]:
    sc = SCENARIOS[name]
    n.r = {}
    n.set_tf(sc); spin(1.0)
    cost = costmap_cells(sc)
    depth, mask = render(sc, rng)
    frames = int(os.environ.get('FRAMES', 5))
    for i in range(frames):
        n.cm.publish(n.costmap_msg(cost)); spin(0.3)
        depth, mask = render(sc, rng)
        stamp = n.get_clock().now().to_msg()
        pc, full = utils.project_depth_to_pc_torch(torch.tensor(depth), torch.zeros(H, W, 3, dtype=torch.uint8),
                                                   CALIB, 'cam_opt', stamp, mask=mask, max_depth=3.0)
        n.seg.publish(SegmentedPointcloud(segmented_object=pc, full_pointcloud=full))
        n.r['n_full'] = full.width; n.r['n_obj'] = pc.width; n.r['last_stamp'] = stamp.sec + 1e-9 * stamp.nanosec
        spin(float(os.environ.get('WAIT', 1.2)))
    r = dict(n.r); r['scene'] = {k: v for k, v in sc.items()}
    r['costmap'] = cost[::1, ::1].tolist()
    r['grid'] = dict(res=RES, origin=ORIGIN, size=SIZE)
    # contour is in the base frame: bring to map
    c, s = math.cos(sc['robot'][2]), math.sin(sc['robot'][2])
    r['contour_map'] = [[sc['robot'][0] + c * p[0] - s * p[1], sc['robot'][1] + s * p[0] + c * p[1]] for p in r.get('contour', [])]
    out[name] = r
    print(name, {k: (len(v) if isinstance(v, list) else v) for k, v in r.items() if k in ('contour', 'cand', 'filt', 'goal', 'n_full', 'n_obj', 'plane_n', 'cluster_n')},
          'goal=', r.get('goal'), 'obj=', r.get('object_base'), flush=True)
json.dump(out, open(sys.argv[1], 'w'))
rclpy.shutdown()
