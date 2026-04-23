import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ld = launch.LaunchDescription()
    surface_detector_dir = launch.substitutions.LaunchConfiguration(
        'surface_detector',
        default=os.path.join(
            get_package_share_directory('surface_detection'),
            'param',
            'surface_detector.yaml'))

    surface_detector = launch_ros.actions.LifecycleNode(
            name = 'surface_detector',
            namespace='',
            package='surface_detection',
            executable='surface_detector',
            output='screen',
            parameters=[surface_detector_dir]
        )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(surface_detector),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=surface_detector,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(surface_detector),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(from_inactive_to_active)
    ld.add_action(surface_detector)
    ld.add_action(to_inactive)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        ld
    ])
