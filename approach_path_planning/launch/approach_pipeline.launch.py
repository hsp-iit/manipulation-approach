import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    surface_det = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('surface_detection'), 'launch'),
            '/surface_detector.launch.py'])
        )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('approach_path_planning'), 'launch'),
            '/approach_planner.launch.py'])
        )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('approach_path_planning'), 'launch'),
            '/rviz.launch.py'])
        )
    

    return LaunchDescription([
        surface_det,
        planner,
        rviz
    ])