import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("exclude_tf_frames", default_value="['odom']", description="exclude tf frames"))
    rosbag_play_launch_dir = os.path.join(get_package_share_directory('utils'), 'launch')
    rosbag_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rosbag_play_launch_dir, 'rosbag_play.launch.py')),
        launch_arguments={"exclude_tf_frames":LaunchConfiguration("exclude_tf_frames")}.items()
    )
    ld.add_action(rosbag_launch)
    return ld