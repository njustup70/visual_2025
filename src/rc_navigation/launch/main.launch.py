from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 声明仿真模式参数
    sim_mode_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='是否启用仿真模式 (true:仿真, false:实际部署)',
        choices=['true', 'false']
    )
    
    # 实际部署的launch文件组合
    real_world_launch_files = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/rc_nav.launch.py'  # 实际部署导航
            ]),
            condition=UnlessCondition(LaunchConfiguration('sim'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/nav_decide.launch.py'  # 实际部署决策
            ]),
            condition=UnlessCondition(LaunchConfiguration('sim'))
        )
    ]
    
    # 仿真模式的launch文件组合
    sim_launch_files = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/rc_nav_sim.launch.py'  # 仿真导航
            ]),
            condition=IfCondition(LaunchConfiguration('sim'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                '/nav_decide_sim.launch.py'  # 仿真决策
            ]),
            condition=IfCondition(LaunchConfiguration('sim'))
        )
    ]

    return LaunchDescription([
        sim_mode_arg,
        *real_world_launch_files,
        *sim_launch_files
    ])