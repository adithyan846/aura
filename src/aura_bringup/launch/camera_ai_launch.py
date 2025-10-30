import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define parameters for the camera
    frame_duration = [200000, 200000] # 5 FPS

    # 1. Camera Node Configuration
    rpi_camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        # Name the camera node consistently
        name='rpi_camera_node', 
        output='screen',
        parameters=[
            {'width': 640},
            {'height': 480},
            {'FrameDurationLimits': frame_duration},
            {'format': "XRGB8888"},
        ],
        # Remap the camera's default image topic to the project's standard '/visuals/image_raw'
        # The node's default topic 'image_raw' resolves internally to '/rpi_camera_node/image_raw'
        # We remap the full path to a more abstract 'visuals' topic for downstream nodes (like aura_ai)
        remappings=[
            ('/rpi_camera_node/image_raw', '/visuals/image_raw'),
        ]
    )

    # 2. AI Detection Node Configuration
    # This node subscribes to /visuals/image_raw and publishes /detection
    aura_ai_node = Node(
        package='aura_ai',
        executable='ai_node',
        name='aura_ai_processor',
        output='screen',
        # The aura_ai_node already subscribes to '/visuals/image_raw' internally, 
        # so no remapping is explicitly needed for its input if we follow the convention set above.
        # Its output topic is /detection (which is what we want)
    )

    return LaunchDescription([
        rpi_camera_node,
        aura_ai_node,
        # You would also add your Pixhawk node here to complete the onboard system
        # Node(
        #     package='aura_pixhawk',
        #     executable='pixhawk_node',
        #     name='flight_controller_interface',
        # ),
    ])
