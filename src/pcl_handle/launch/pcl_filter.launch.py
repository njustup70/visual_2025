import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    # Get the launch file directory
    pcl_handle_dir = get_package_share_directory('pcl_handle')
    # Create the launch configuration variables
    pcl_handle_params = os.path.join(pcl_handle_dir, 'config', 'filter.json')
    # Declare the launch options
    return LaunchDescription([
        Node(
            package='pcl_handle',
            executable='PclHandle_node',
            # name='pcl_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{"config_path": pcl_handle_params,
                         "pub_xyzitr": False,
                         "subscribe_topic": "/livox/lidar/raw",
                         "publish_topic": "/livox/lidar",
                         "param_debug": False,
                         "pipeline_id": 0,
                         }],
            
        )
    ])