# SPDX-FileCopyrightText: 2026 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
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

        # Downstream detector goal handles, indexed by the upstream goal id.
        self.detector_goal_handles = {}

        self.cb_grp = ReentrantCallbackGroup()

        # Action client used to send goals to the detector node.
        # Not in the reentrant group: rclpy action clients can process the same
        # goal response twice when run concurrently.
        self.detector_client = ActionClient(
            self,
            ReachObject,
            self.detector_action_name,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Public action server used by external callers.
        self.action_server = ActionServer(
            self,
            ReachObject,
            self.coordinator_action_name,
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_grp,
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

    def _abort(self, goal_handle: ServerGoalHandle, error_msg: str):
        result = ReachObject.Result()
        result.reached = False
        result.error_msg = error_msg
        goal_handle.abort()
        return result

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        # Propagate the cancellation to the detector: the upstream goal is
        # terminated in execute_callback once the detector goal ends.
        detector_goal_handle = self.detector_goal_handles.get(bytes(goal_handle.goal_id.uuid))
        if detector_goal_handle is not None:
            detector_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Forward one incoming coordinated goal to the detector action server."""

        self.get_logger().info(
            f"Received coordinated request for object: {goal_handle.request.object_string}"
        )

        # Ensure the downstream detector action server is available.
        if not self.detector_client.wait_for_server(timeout_sec=self.detector_server_wait_timeout):
            return self._abort(
                goal_handle,
                f"Detector action server {self.detector_action_name} not available",
            )

        if goal_handle.is_cancel_requested:
            result = ReachObject.Result()
            result.reached = False
            result.error_msg = "Coordinator goal canceled"
            goal_handle.canceled()
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
            return self._abort(goal_handle, "Detector rejected goal")

        goal_key = bytes(goal_handle.goal_id.uuid)
        self.detector_goal_handles[goal_key] = detector_goal_handle
        try:
            # A cancellation may have arrived while the goal was being sent.
            if goal_handle.is_cancel_requested:
                detector_goal_handle.cancel_goal_async()
            # Wait for downstream result (cancellation is forwarded by cancel_callback).
            detector_wrapped_result = await detector_goal_handle.get_result_async()
        finally:
            del self.detector_goal_handles[goal_key]

        # Map downstream terminal status into upstream terminal status.
        detector_result = detector_wrapped_result.result
        detector_status = detector_wrapped_result.status

        if detector_status == GoalStatus.STATUS_SUCCEEDED and detector_result.reached:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
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
