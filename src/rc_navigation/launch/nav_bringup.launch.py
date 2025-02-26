import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#传入参数列表将多个yaml合并成一个
def merge_yaml_files(file_paths, output_file):
    merged_data = {}
    for file_path in file_paths:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            if data:
                merged_data.update(data)
    with open(output_file, 'w') as file:
        yaml.safe_dump(merged_data, file)

def generate_launch_description():
    os.environ['RCUTILS_LOGGING_SEVERITY'] = 'WARN'  # 设置日志级别为 WARN
    my_navigation2_dir = get_package_share_directory('rc_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(my_navigation2_dir,'map','map.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(my_navigation2_dir,'config','my_nav2.yaml'))
    
    #=============================3.合并多个yaml文件===================================================
    navYamlPaths=[ 
                    os.path.join(my_navigation2_dir,'config','amcl.yaml'),
                    os.path.join(my_navigation2_dir,'config','bt_navigator.yaml'),
                    os.path.join(my_navigation2_dir,'config','controller_server.yaml'),
                    os.path.join(my_navigation2_dir,'config','costmaps.yaml'),
                    os.path.join(my_navigation2_dir,'config','planner_server.yaml'),
                  ]
    merged_nav2_param_path = os.path.join(my_navigation2_dir,'config','mergenav2.yaml')
    merge_yaml_files(navYamlPaths,merged_nav2_param_path)
    # rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')
    rviz_config_dir=os.path.join(my_navigation2_dir,'rviz','nav2_stvl.rviz')
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': merged_nav2_param_path,
                'log_level': 'WARN'  # 设置日志级别为 WARN
                }.items(),
        # launch_arguments=[map_yaml_path,nav2_param_path]
        )
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],  # 设置日志级别为 WARN
            ros_arguments=['--log-level', 'warn'],
            output='screen')
    ld=LaunchDescription()
    ld.add_action(nav2_bringup_launch)
    # ld.add_action(rviz_node)
    return ld