#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 声明YAML文件路径参数（允许命令行覆盖）
    config_path_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('rc_navigation'),
            'config',  
            'nav_decide.yaml'
        ]),
        description='全局参数配置文件路径'
    )
    
    # 2. 节点定义（从YAML加载参数）
    nodes = [
        # RandomPointGenerator节点
        Node(
            package='my_algorithm',
            executable='random_generate.py',
            name='random_generate',
            # 关键改造：优先加载YAML参数
            parameters=[LaunchConfiguration('config_file')]
        ),
        # ObstacleExtractor节点
        Node(
            package='change_laser',
            executable='obstacle_extractor_node',
            name='obstacle_extractor',
            parameters=[LaunchConfiguration('config_file')]
        ),
        # OptimalPointSelector节点
        Node(
            package='my_algorithm',
            executable='logic.py',
            name='optimal_point_selector',
            parameters=[LaunchConfiguration('config_file')]
        ),
        # OptimalGoalNavigator节点
        Node(
            package='my_algorithm',
            executable='nav_behave.py',
            name='optimal_goal_navigator',
            parameters=[LaunchConfiguration('config_file')]
        )
    ]
    
    return LaunchDescription([
        config_path_arg,
        LogInfo(msg="启动导航决策系统..."),
        *nodes,
        LogInfo(msg=f"参数文件: {LaunchConfiguration('config_file')}"),
        LogInfo(msg="所有节点已启动，系统运行中")
    ])