import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to other launch files
    bringup_dir = get_package_share_directory('aura_bringup')
    camera_ai_launch = os.path.join(bringup_dir, 'launch', 'camera_ai_launch.py')

    # Include the camera + AI system
    camera_ai = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_ai_launch)
    )

    # Add Pixhawk / MAVLink telemetry node (you can enable later)
    pixhawk_node = Node(
        package='aura_pixhawk',
        executable='pixhawk_node',
        name='pixhawk_interface',
        output='screen',
        # parameters=[{'connection_url': '/dev/ttyAMA0', 'baud_rate': 921600}],
    )

    # Telemetry bridge (optional future node for data uplink)
    

    """ telemetry_node = Node(
        package='aura_telemetry',
        executable='telemetry_node',
        name='aura_telemetry',
        output='screen',
    )"""

    return LaunchDescription([
        camera_ai,
        pixhawk_node,
        #telemetry_node,
    ])

