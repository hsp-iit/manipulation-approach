# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""Integration test of detector + coordinator action/navigation logic.

Each node runs in its own process, as in the real pipeline:
  - detector (DINO/SAM stubbed)       -> run_detector_stub.py
  - coordinator                       -> object_detection/coordinator.py
It runs on an isolated ROS domain (REACH_TEST_DOMAIN_ID, default 87) to not interfere with a running pipeline.
  - this harness: fake Nav2 NavigateToPose server, fake camera, fake planner goals, action clients
"""
import os
import signal
import subprocess
import sys
import threading
import time
import traceback

os.environ['ROS_DOMAIN_ID'] = os.environ.get('REACH_TEST_DOMAIN_ID', '87')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from surface_detector_interfaces.action import ReachObject

HERE = os.path.dirname(os.path.abspath(__file__))
OBJECT_DETECTION_DIR = os.path.dirname(HERE)


class FakeNav(Node):
    """NavigateToPose server behaving like Nav2: a new goal aborts the previous one."""
    def __init__(self):
        super().__init__('fake_nav')
        self.lock = threading.Lock()
        self.goals = []          # received poses
        self.cancelled = 0
        self.latest = None
        self.succeed_now = False
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=self.goal_cb,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())

    def goal_cb(self, goal):
        with self.lock:
            self.goals.append((goal.pose.pose.position.x, goal.pose.pose.position.y))
        return GoalResponse.ACCEPT

    def execute(self, gh):
        my_id = bytes(gh.goal_id.uuid)
        with self.lock:
            self.latest = my_id
        fb = NavigateToPose.Feedback()
        while True:
            time.sleep(0.05)
            with self.lock:
                if self.latest != my_id:
                    gh.abort()
                    return NavigateToPose.Result()
                succeed = self.succeed_now
            if gh.is_cancel_requested:
                with self.lock:
                    self.cancelled += 1
                gh.canceled()
                return NavigateToPose.Result()
            if succeed:
                with self.lock:
                    self.succeed_now = False
                gh.succeed()
                return NavigateToPose.Result()
            fb.distance_remaining = 1.5
            gh.publish_feedback(fb)


class Harness(Node):
    def __init__(self):
        super().__init__('harness')
        self.img_pub = self.create_publisher(Image, '/camera/rgbd/img', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/rgbd/depth', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/approach_planner/goal_pose', 10)
        self.det_client = ActionClient(self, ReachObject, '/reach_object',
                                       callback_group=MutuallyExclusiveCallbackGroup())
        self.coord_client = ActionClient(self, ReachObject, '/reach_object_coordinator',
                                         callback_group=MutuallyExclusiveCallbackGroup())
        self.last_stamp = None
        self.feedback = []
        self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()
        img = Image(height=48, width=64, encoding='bgr8', step=64 * 3, data=bytes(48 * 64 * 3))
        depth = Image(height=48, width=64, encoding='32FC1', step=64 * 4,
                      data=np.ones((48, 64), dtype=np.float32).tobytes())
        img.header.stamp = stamp
        depth.header.stamp = stamp
        img.header.frame_id = depth.header.frame_id = 'cam'
        self.img_pub.publish(img)
        self.depth_pub.publish(depth)
        self.last_stamp = stamp

    def publish_goal(self, x, y, stamp=None):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp if stamp is not None else self.last_stamp
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def wait(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def send(client, harness, obj='cup'):
    goal = ReachObject.Goal(object_string=obj)
    f = client.send_goal_async(goal, feedback_callback=lambda m: harness.feedback.append(m.feedback.distance_remaining))
    assert wait(f.done), 'goal response timeout'
    return f.result()


def result_of(rf, timeout=10.0):
    assert wait(rf.done, timeout), 'result timeout'
    return rf.result()


def cancel(gh):
    cf = gh.cancel_goal_async()
    assert wait(cf.done), 'cancel response timeout'
    return cf.result()


def check(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + (f'  ({detail})' if detail else ''), flush=True)
    return cond


def run_tests(h, nav):
    ok = True
    assert h.det_client.wait_for_server(20.0), 'detector server not up'
    assert h.coord_client.wait_for_server(20.0), 'coordinator server not up'
    time.sleep(1.0)

    # ---- 1: success, stale goal dropped, goal hysteresis, preemption ignored, no goals after success
    gh = send(h.det_client, h)
    ok &= check('1 goal accepted', gh.accepted)
    rf = gh.get_result_async()
    time.sleep(0.6)   # let the detector process the first frames of the request
    old = h.get_clock().now().to_msg(); old.sec -= 10
    h.publish_goal(1.0, 0.0, stamp=old)
    time.sleep(0.4)
    ok &= check('1 stale goal dropped', len(nav.goals) == 0, nav.goals)
    h.publish_goal(1.0, 0.0)
    ok &= check('1 first goal sent', wait(lambda: len(nav.goals) == 1), nav.goals)
    time.sleep(0.1); h.publish_goal(3.0, 0.0)       # big move, within min interval
    time.sleep(0.6); h.publish_goal(1.05, 0.0)      # small move after interval
    time.sleep(0.3)
    ok &= check('1 interval + small move filtered', len(nav.goals) == 1, nav.goals)
    h.publish_goal(2.0, 0.0)                        # big move after interval -> preempts
    ok &= check('1 second goal sent', wait(lambda: len(nav.goals) == 2), nav.goals)
    time.sleep(0.5)
    ok &= check('1 request still running after preemption', not rf.done())
    nav.succeed_now = True
    res = result_of(rf)
    ok &= check('1 succeeded', res.status == GoalStatus.STATUS_SUCCEEDED and res.result.reached,
                f'status={res.status} msg={res.result.error_msg!r}')
    ok &= check('1 feedback relayed', len(h.feedback) > 0 and h.feedback[-1] > 0)
    h.publish_goal(5.0, 0.0)
    time.sleep(0.5)
    ok &= check('1 goals after success ignored', len(nav.goals) == 2, nav.goals)

    # ---- 2: cancel while navigating -> navigation cancelled
    gh = send(h.det_client, h)
    rf = gh.get_result_async()
    time.sleep(0.6); h.publish_goal(1.0, 1.0)
    wait(lambda: len(nav.goals) == 3)
    time.sleep(0.4)
    ok &= check('2 cancel accepted', len(cancel(gh).goals_canceling) == 1)
    res = result_of(rf)
    ok &= check('2 canceled', res.status == GoalStatus.STATUS_CANCELED, f'status={res.status}')
    ok &= check('2 navigation cancelled', wait(lambda: nav.cancelled == 1), nav.cancelled)

    # ---- 3: concurrent request rejected
    gh1 = send(h.det_client, h)
    gh2 = send(h.det_client, h)
    ok &= check('3 second concurrent request rejected', gh1.accepted and not gh2.accepted,
                f'{gh1.accepted=} {gh2.accepted=}')
    rf = gh1.get_result_async()
    cancel(gh1)
    res = result_of(rf)
    ok &= check('3 first request canceled', res.status == GoalStatus.STATUS_CANCELED, f'status={res.status}')

    # ---- 4: timeout when navigation never starts (stub uses a 3 s timeout)
    gh = send(h.det_client, h)
    res = result_of(gh.get_result_async())
    ok &= check('4 timeout aborts', res.status == GoalStatus.STATUS_ABORTED and 'Timeout' in res.result.error_msg,
                res.result.error_msg)

    # ---- 5: coordinator success
    n_goals = len(nav.goals)
    gh = send(h.coord_client, h)
    ok &= check('5 coordinator goal accepted', gh.accepted)
    rf = gh.get_result_async()
    time.sleep(0.8); h.publish_goal(-1.0, 0.0)
    wait(lambda: len(nav.goals) == n_goals + 1)
    time.sleep(0.4)
    nav.succeed_now = True
    res = result_of(rf)
    ok &= check('5 coordinator succeeded', res.status == GoalStatus.STATUS_SUCCEEDED and res.result.reached,
                f'status={res.status} msg={res.result.error_msg!r}')

    # ---- 6: coordinator cancel propagates to detector and navigation
    n_cancel = nav.cancelled
    gh = send(h.coord_client, h)
    rf = gh.get_result_async()
    time.sleep(0.8); h.publish_goal(-2.0, 0.0)
    wait(lambda: len(nav.goals) == n_goals + 2)
    time.sleep(0.4)
    ok &= check('6 coordinator cancel accepted', len(cancel(gh).goals_canceling) == 1)
    res = result_of(rf)
    ok &= check('6 coordinator canceled', res.status == GoalStatus.STATUS_CANCELED, f'status={res.status}')
    ok &= check('6 navigation cancelled', wait(lambda: nav.cancelled == n_cancel + 1), nav.cancelled)
    gh = send(h.det_client, h)
    ok &= check('6 detector accepts a new request afterwards', gh.accepted)
    rf = gh.get_result_async()
    cancel(gh)
    result_of(rf)
    return ok


def main():
    procs = [
        subprocess.Popen([sys.executable, os.path.join(HERE, 'run_detector_stub.py')]),
        subprocess.Popen([sys.executable, os.path.join(OBJECT_DETECTION_DIR, 'coordinator.py')]),
    ]
    rclpy.init()
    nav = FakeNav()
    h = Harness()
    exe = MultiThreadedExecutor(num_threads=8)
    exe.add_node(nav)
    exe.add_node(h)
    threading.Thread(target=exe.spin, daemon=True).start()
    try:
        ok = run_tests(h, nav)
    except Exception:
        traceback.print_exc()
        ok = False
    print('ALL PASSED' if ok else 'SOME FAILED', flush=True)
    for p in procs:
        p.send_signal(signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    exe.shutdown()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
