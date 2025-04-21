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
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('use_composition',default_value='False',description='Use lifecycle nodes'))
    ld.add_action(DeclareLaunchArgument('params_file',default_value=param_file_path,description='Full path to the ROS2 parameters file to use'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'))
    #======调用原始launch文件，传入参数======
    my_packager_share_dir = get_package_share_directory('rc_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file')
                          }.items()
    )
    #======================================启动地图服务
    map_path=os.path.join(my_packager_share_dir,'map','court_map.yaml')
    # map_path=os.path.join(my_packager_share_dir,'map','empty_map.yaml')
    ld.add_action(DeclareLaunchArgument('map', default_value=map_path, description='Full path to map yaml file to load'))
    map_server_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_packager_share_dir,'/launch','/map_server.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'map': LaunchConfiguration('map')}.items()
    )
    ld.add_action(nav2_bringup_launch)
    ld.add_action(map_server_launch)
    return ld