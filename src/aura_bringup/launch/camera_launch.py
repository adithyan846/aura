import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 5 FPS = 200000 microseconds frame duration
    frame_duration = [200000, 200000]
    
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='rpi_camera_node',
            output='screen',
            parameters=[
                {'width': 640},
                {'height': 480},
                # Sets the fixed frame rate (5 FPS)
                {'FrameDurationLimits': frame_duration}, 
                # Optional: Silence warnings by setting defaults
                {'format': "XRGB8888"}, 
            ]
        ),
        # You can add other nodes from aura_ws here, like image processors
        # Node(
        #     package='aura_vision',
        #     executable='detector_node',
        #     name='image_processor',
        # ),
    ])
