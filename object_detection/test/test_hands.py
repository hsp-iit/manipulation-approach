# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""Integration test of the hands raise stage of a /reach_object request.

Runs the real detector (DINO/SAM stubbed) with the real YARP client against fake arm controllers, which reply
like the real ones: with the vocabs ok and fail. Isolated from the rest of the system:
  - ROS domain REACH_TEST_DOMAIN_ID, default 86
  - a private YARP name server on 127.0.0.1, with its configuration in a temporary folder
"""
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback

os.environ.setdefault('REACH_TEST_DOMAIN_ID', '86')    # test_reach applies it at import
os.environ['ROS_DOMAIN_ID'] = os.environ['REACH_TEST_DOMAIN_ID']
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
YARP_DIR = tempfile.mkdtemp(prefix='test_hands_yarp_')
os.environ['YARP_CONFIG_HOME'] = YARP_DIR
os.environ['YARP_DATA_HOME'] = YARP_DIR
YARP_SOCKET = os.environ.get('REACH_TEST_YARP_SOCKET', '10322')

import rclpy
import yarp
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, TransformStamped
from surface_detector_interfaces.action import ReachObject
from surface_detector_interfaces.msg import DetectionResults
from tf2_ros import StaticTransformBroadcaster

from test_reach import FakeNav, Harness, HERE, cancel, check, result_of, send, wait

ROOT_Z = 0.8            # height of the fake arm root frame in the base frame
SURFACE_Z = 0.75


class FakeArm:
    """Replies like the thrift service of the arm controllers, with a motion that ends after motion_time"""
    def __init__(self, name):
        self.motion_time = 0.8
        self.refuse = False
        self.commands = []
        self.stops = 0
        self.move_end = 0.0
        self.running = True
        self.port = yarp.Port()
        self.port.open(f'/fake_arm/{name}/rpc:i')
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        while self.running:
            command = yarp.Bottle()
            if not self.port.read(command, True) or command.size() == 0:
                continue
            ok = True
            name = command.get(0).toString()
            if name == 'go_to_position':
                self.commands.append([command.get(i).asFloat64() for i in range(1, 4)])
                self.move_end = time.time() + self.motion_time
                ok = not self.refuse
            elif name == 'is_motion_done':
                ok = time.time() >= self.move_end
            elif name == 'stop':
                self.stops += 1
                self.move_end = 0.0
            reply = yarp.Bottle()
            reply.fromString('[ok]' if ok else '[fail]')
            self.port.reply(reply)

    def close(self):
        self.running = False
        self.port.interrupt()
        self.port.close()


def contour(distance):
    """Square surface edge in front of the base frame, at the given distance"""
    return [(distance, -0.7), (distance + 0.8, -0.7), (distance + 0.8, 0.7), (distance, 0.7), (distance, -0.7)]


def publish_surface(harness, publisher, distance, stamp=None):
    msg = DetectionResults()
    stamp = stamp if stamp is not None else harness.last_stamp
    msg.segmented_object.header.frame_id = 'base'
    msg.segmented_object.header.stamp = stamp
    for x, y in contour(distance):
        point = PointStamped()
        point.header.frame_id = 'base'
        point.header.stamp = stamp
        point.point.x, point.point.y, point.point.z = x, y, SURFACE_Z
        msg.surface_contours.append(point)
    publisher.publish(msg)


def run_tests(h, nav, surface_pub, left, right):
    ok = True
    assert h.det_client.wait_for_server(20.0), 'detector server not up'
    time.sleep(1.0)

    # ---- 1: the goal waits for the hands, targets in the arm frame, success
    left.commands.clear(); right.commands.clear()
    gh = send(h.det_client, h)
    ok &= check('1 goal accepted', gh.accepted)
    rf = gh.get_result_async()
    time.sleep(0.6)
    h.publish_goal(1.0, 0.0)
    time.sleep(0.3)
    ok &= check('1 goal before the surface dropped', len(nav.goals) == 0, nav.goals)
    old = h.get_clock().now().to_msg(); old.sec -= 10
    publish_surface(h, surface_pub, 1.0, stamp=old)
    time.sleep(0.3)
    ok &= check('1 stale surface ignored', not left.commands and not right.commands)
    publish_surface(h, surface_pub, 1.0)
    ok &= check('1 both arms commanded', wait(lambda: left.commands and right.commands))
    z = SURFACE_Z + 0.1 - ROOT_Z
    ok &= check('1 left target in the arm frame', max(abs(a - b) for a, b in zip(left.commands[0], [0.4, 0.2, z])) < 1e-6, left.commands)
    ok &= check('1 right target in the arm frame', max(abs(a - b) for a, b in zip(right.commands[0], [0.4, -0.2, z])) < 1e-6, right.commands)
    time.sleep(0.3)
    h.publish_goal(1.0, 0.0)
    time.sleep(0.2)
    ok &= check('1 goal during the motion dropped', len(nav.goals) == 0, nav.goals)
    publish_surface(h, surface_pub, 0.9)
    time.sleep(0.2)
    ok &= check('1 raised only once', len(left.commands) == 1, left.commands)
    time.sleep(1.0)
    h.publish_goal(1.0, 0.0)
    ok &= check('1 goal forwarded after the motion', wait(lambda: len(nav.goals) == 1), nav.goals)
    time.sleep(0.3)
    nav.succeed_now = True
    res = result_of(rf)
    ok &= check('1 succeeded', res.status == GoalStatus.STATUS_SUCCEEDED and res.result.reached, res.result.error_msg)
    ok &= check('1 arms not stopped', left.stops == 0 and right.stops == 0)

    # ---- 2: too close to raise the hands
    n_goals = len(nav.goals)
    left.commands.clear(); right.commands.clear()
    gh = send(h.det_client, h)
    rf = gh.get_result_async()
    time.sleep(0.6)
    publish_surface(h, surface_pub, 0.3)
    res = result_of(rf)
    ok &= check('2 too close aborts', res.status == GoalStatus.STATUS_ABORTED and 'too close' in res.result.error_msg, res.result.error_msg)
    ok &= check('2 arms not moved', not left.commands and not right.commands)

    # ---- 3: cancel during the motion stops the arms
    stops = left.stops, right.stops
    gh = send(h.det_client, h)
    rf = gh.get_result_async()
    left.motion_time = right.motion_time = 1.5
    time.sleep(0.6)
    publish_surface(h, surface_pub, 1.0)
    wait(lambda: left.commands and right.commands)
    time.sleep(0.1)
    cancel(gh)
    res = result_of(rf)
    ok &= check('3 canceled', res.status == GoalStatus.STATUS_CANCELED, res.status)
    ok &= check('3 arms stopped', wait(lambda: left.stops == stops[0] + 1 and right.stops == stops[1] + 1), (left.stops, right.stops))

    # ---- 4: the arms never finish: abort, no goal to navigation
    left.motion_time = 100.0
    stops = left.stops, right.stops
    gh = send(h.det_client, h)
    rf = gh.get_result_async()
    time.sleep(0.6)
    publish_surface(h, surface_pub, 1.0)
    time.sleep(1.0)
    h.publish_goal(1.0, 0.0)
    res = result_of(rf)
    ok &= check('4 motion timeout aborts', res.status == GoalStatus.STATUS_ABORTED and 'Timeout while raising' in res.result.error_msg, res.result.error_msg)
    ok &= check('4 no goal to navigation', len(nav.goals) == n_goals, nav.goals)
    ok &= check('4 arms stopped', wait(lambda: left.stops == stops[0] + 1 and right.stops == stops[1] + 1), (left.stops, right.stops))
    left.motion_time = 0.1
    left.move_end = 0.0

    # ---- 5: a refused motion aborts, and stops the arm that accepted it
    right.commands.clear()
    stops = right.stops
    left.refuse, right.refuse = False, True
    gh = send(h.det_client, h)
    rf = gh.get_result_async()
    time.sleep(0.6)
    publish_surface(h, surface_pub, 1.0)
    res = result_of(rf)
    ok &= check('5 refused motion aborts', res.status == GoalStatus.STATUS_ABORTED and 'refused' in res.result.error_msg, res.result.error_msg)
    ok &= check('5 left arm stopped', wait(lambda: left.stops > 0 and right.stops == stops + 1), (left.stops, right.stops))
    right.refuse = False

    # ---- 6: a controller that is not running aborts at the start
    left.close()
    time.sleep(0.5)
    gh = send(h.det_client, h)
    res = result_of(gh.get_result_async())
    ok &= check('6 missing controller aborts', res.status == GoalStatus.STATUS_ABORTED, res.result.error_msg)
    ok &= check('6 no goal to navigation', len(nav.goals) == n_goals, nav.goals)
    return ok


def main():
    log = open(os.path.join(YARP_DIR, 'yarpserver.log'), 'w')
    yarpserver = subprocess.Popen(['yarpserver', '--ip', '127.0.0.1', '--socket', YARP_SOCKET, '--write',
                                   '--portdb', os.path.join(YARP_DIR, 'ports.db'),
                                   '--subdb', os.path.join(YARP_DIR, 'subs.db')], stdout=log, stderr=log)
    time.sleep(2.0)
    yarp.Network.init()
    assert yarp.Network.checkNetwork(3.0), 'private YARP name server not up'
    left, right = FakeArm('left'), FakeArm('right')
    detector = subprocess.Popen([sys.executable, os.path.join(HERE, 'run_detector_stub.py')],
                                env=dict(os.environ, STUB_RAISE_HANDS='true'))
    rclpy.init()
    nav = FakeNav()
    h = Harness()
    surface_pub = h.create_publisher(DetectionResults, '/surface_detector/results', 10)
    tf = TransformStamped()
    tf.header.stamp = h.get_clock().now().to_msg()
    tf.header.frame_id = 'fake_root'
    tf.child_frame_id = 'base'
    tf.transform.translation.z = -ROOT_Z
    tf.transform.rotation.w = 1.0
    broadcaster = StaticTransformBroadcaster(h)
    broadcaster.sendTransform(tf)
    exe = MultiThreadedExecutor(num_threads=8)
    exe.add_node(nav)
    exe.add_node(h)
    threading.Thread(target=exe.spin, daemon=True).start()
    try:
        ok = run_tests(h, nav, surface_pub, left, right)
    except Exception:
        traceback.print_exc()
        ok = False
    print('ALL PASSED' if ok else 'SOME FAILED', flush=True)
    detector.send_signal(signal.SIGINT)
    try:
        detector.wait(timeout=5)
    except subprocess.TimeoutExpired:
        detector.kill()
    yarpserver.terminate()
    exe.shutdown()
    rclpy.try_shutdown()
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
