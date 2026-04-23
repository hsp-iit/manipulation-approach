# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

import asyncio

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from surface_detector_interfaces.action import ReachObject


class ReachCoordinator(Node):
    """Minimal orchestration layer that forwards `ReachObject` goals to detector.

    This node exposes its own action server and acts as a proxy/client toward the
    detector action server. It forwards request, feedback and result, and handles
    cancellation propagation.
    """

    def __init__(self):
        super().__init__("reach_coordinator")

        # Runtime parameters for endpoint names and server wait timeout.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("coordinator_action_name", "/reach_object_coordinator"),
                ("detector_action_name", "/reach_object"),
                ("detector_server_wait_timeout", 5.0),
            ],
        )

        self.coordinator_action_name = self.get_parameter("coordinator_action_name").value
        self.detector_action_name = self.get_parameter("detector_action_name").value
        self.detector_server_wait_timeout = float(
            self.get_parameter("detector_server_wait_timeout").value
        )

        # Action client used to send goals to the detector node.
        self.detector_client = ActionClient(
            self,
            ReachObject,
            self.detector_action_name,
        )

        # Public action server used by external callers.
        self.action_server = ActionServer(
            self,
            ReachObject,
            self.coordinator_action_name,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info(
            "Coordinator ready. Serving %s and forwarding to detector action %s"
            % (self.coordinator_action_name, self.detector_action_name)
        )

    def _forward_feedback(
        self,
        outer_goal_handle: ServerGoalHandle,
        feedback_msg,
    ) -> None:
        # Translate detector feedback into coordinator feedback and relay it.
        outer_feedback = ReachObject.Feedback()
        outer_feedback.distance_remaining = feedback_msg.feedback.distance_remaining
        outer_goal_handle.publish_feedback(outer_feedback)

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Forward one incoming coordinated goal to the detector action server."""

        self.get_logger().info(
            f"Received coordinated request for object: {goal_handle.request.object_string}"
        )

        # Ensure the downstream detector action server is available.
        if not self.detector_client.wait_for_server(timeout_sec=self.detector_server_wait_timeout):
            result = ReachObject.Result()
            result.reached = False
            result.error_msg = (
                f"Detector action server {self.detector_action_name} not available"
            )
            goal_handle.abort()
            return result

        # Build downstream goal from upstream request.
        detector_goal = ReachObject.Goal()
        detector_goal.object_string = goal_handle.request.object_string

        # Send goal asynchronously and forward feedback in real-time.
        detector_goal_handle = await self.detector_client.send_goal_async(
            detector_goal,
            feedback_callback=lambda msg: self._forward_feedback(goal_handle, msg),
        )

        # Downstream server may reject the goal.
        if not detector_goal_handle.accepted:
            result = ReachObject.Result()
            result.reached = False
            result.error_msg = "Detector rejected goal"
            goal_handle.abort()
            return result

        # Wait for downstream result while checking user cancellation requests.
        detector_result_future = detector_goal_handle.get_result_async()

        while not detector_result_future.done():
            await asyncio.sleep(0.1)
            if goal_handle.is_cancel_requested:
                # Propagate cancellation to downstream goal and terminate upstream.
                await detector_goal_handle.cancel_goal_async()
                result = ReachObject.Result()
                result.reached = False
                result.error_msg = "Coordinator goal canceled"
                goal_handle.canceled()
                return result

        # Map downstream terminal status into upstream terminal status.
        detector_wrapped_result = await detector_result_future
        detector_result = detector_wrapped_result.result
        detector_status = detector_wrapped_result.status

        if detector_status == GoalStatus.STATUS_SUCCEEDED and detector_result.reached:
            goal_handle.succeed()
            return detector_result

        if detector_status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            goal_handle.canceled()
            return detector_result

        goal_handle.abort()
        return detector_result



def main():
    rclpy.init()
    node = ReachCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
