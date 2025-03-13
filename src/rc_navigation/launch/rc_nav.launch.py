import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
def generate_launch_description():
    ld=LaunchDescription()
    param_file_path=os.path.join(get_package_share_directory('rc_navigation'),'config','nav2_params.yaml')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True',description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('use_composition',default_value='False',description='Use lifecycle nodes'))
    ld.add_action(DeclareLaunchArgument('params_file',default_value=param_file_path,description='Full path to the ROS2 parameters file to use'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'))
    #======调用原始launch文件，传入参数======
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file')
                          }.items()
    )
    ld.add_action(nav2_bringup_launch)
    return ld