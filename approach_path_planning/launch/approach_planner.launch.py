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
    #approach_planner_dir = launch.substitutions.LaunchConfiguration(
    #    'approach_path_planning',
    #    default=os.path.join(
    #        get_package_share_directory('approach_path_planning'),
    #        'param',
    #        'approach_path_planning.yaml'))

    approach_planner = launch_ros.actions.LifecycleNode(
            name = 'approach_planner',
            namespace='',
            package='approach_path_planning',
            executable='approach_planner',
            output='screen',
            #parameters=[approach_planner_dir]
        )

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(approach_planner),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=approach_planner,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(approach_planner),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(from_inactive_to_active)
    ld.add_action(approach_planner)
    ld.add_action(to_inactive)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        ld
    ])
