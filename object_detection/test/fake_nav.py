# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause
"""NavigateToPose server for dry runs on the robot: logs the approach goals and never moves anything.

A goal stays active until it is preempted by a newer one or cancelled, so a /reach_object request keeps
running until you cancel it. Next to a real Nav2, remap it and point the detector to it, so that no goal
reaches the real navigation:
    python3 fake_nav.py --ros-args -r navigate_to_pose:=approach_dry_run
    python3 detector.py --ros-args -p navigate_to_pose_action:=approach_dry_run
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class FakeNav(Node):
    def __init__(self):
        super().__init__('fake_nav')
        self.lock = threading.Lock()
        self.latest = None
        self.server = ActionServer(self, NavigateToPose, 'navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=self.log_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())
        self.get_logger().info('Dry run: goals are logged, the robot does not move')

    def log_goal(self, goal):
        pose = goal.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f'Goal in {goal.pose.header.frame_id}: X: {pose.position.x:.3f} '
                               f'Y: {pose.position.y:.3f} yaw: {math.degrees(yaw):.1f} deg')
        return rclpy.action.GoalResponse.ACCEPT

    def execute(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self.lock:
            self.latest = goal_id
        while rclpy.ok():
            time.sleep(0.1)
            with self.lock:
                preempted = self.latest != goal_id
            if preempted:
                goal_handle.abort()
                break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal cancelled')
                break
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
