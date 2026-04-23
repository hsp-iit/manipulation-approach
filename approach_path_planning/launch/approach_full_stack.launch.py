import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_repo_root = os.path.expanduser("~/ws_ros2/src/manipulation-approach")

    repo_root = LaunchConfiguration("repo_root")
    detector_script = LaunchConfiguration("detector_script")
    coordinator_script = LaunchConfiguration("coordinator_script")

    detector = ExecuteProcess(
        cmd=["python3", detector_script],
        output="screen",
    )

    coordinator = ExecuteProcess(
        cmd=["python3", coordinator_script],
        output="screen",
    )

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("approach_path_planning"),
                    "launch",
                    "approach_pipeline.launch.py",
                )
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "repo_root",
                default_value=default_repo_root,
                description="Absolute path to repository root",
            ),
            DeclareLaunchArgument(
                "detector_script",
                default_value=[repo_root, "/object_detection/detector.py"],
                description="Absolute path to detector node script",
            ),
            DeclareLaunchArgument(
                "coordinator_script",
                default_value=[repo_root, "/object_detection/coordinator.py"],
                description="Absolute path to coordinator node script",
            ),
            detector,
            coordinator,
            pipeline,
        ]
    )
