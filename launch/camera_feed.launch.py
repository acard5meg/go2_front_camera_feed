from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('go2_camera')
    
    # Path to the parameters file
    # params_file = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    
    # Create the node
    go2_camera_node = Node(
        package='go2_camera',
        executable='camera_node',
        name='camera_node',
        output='screen'
        # parameters=[params_file]
    )
    
    return LaunchDescription([
        go2_camera_node
    ])