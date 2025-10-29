#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import subprocess
import tempfile
import os
import time
import threading

class RPiCameraNode(Node):
    def __init__(self):
        super().__init__('rpicam_camera_node')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.status_pub = self.create_publisher(String, 'camera/status', 10)
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('frame_rate', 5.0)  # Lower FPS for reliability
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('quality', 80)
        
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.quality = self.get_parameter('quality').value
        
        self.get_logger().info(f"RPiCamera Node Started - Target: {self.width}x{self.height} @ {self.frame_rate} FPS")
        
        # Camera state
        self.camera_working = False
        self.camera_initialized = False
        self.frame_count = 0
        
        # Test camera in a separate thread
        self.init_thread = threading.Thread(target=self.initialize_camera)
        self.init_thread.start()
        
        # Timer for status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
    def test_rpicam_hello(self):
        """Test if rpicam-hello works"""
        self.get_logger().info("Testing rpicam-hello...")
        
        try:
            # Quick test - run rpicam-hello for 2 seconds
            process = subprocess.Popen(
                ['timeout', '2', 'rpicam-hello', '--nopreview'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            stdout, stderr = process.communicate()
            
            if process.returncode in [0, 124]:  # Success or timeout
                self.get_logger().info("✓ rpicam-hello test PASSED")
                
                # Extract camera info from stderr
                for line in stderr.split('\n'):
                    if 'fps' in line.lower() or 'Registered camera' in line:
                        self.get_logger().info(f"Camera info: {line.strip()}")
                return True
            else:
                self.get_logger().error(f"✗ rpicam-hello test FAILED: {stderr}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"rpicam-hello test exception: {e}")
            return False
    
    def capture_with_rpicam_jpeg(self):
        """Capture a single frame using rpicam-jpeg"""
        temp_file = None
        try:
            # Create temporary file for the capture
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp_file:
                temp_filename = tmp_file.name
                temp_file = temp_filename
            
            # Capture image with rpicam-jpeg
            result = subprocess.run([
                'rpicam-jpeg',
                '-o', temp_filename,
                '--width', str(self.width),
                '--height', str(self.height),
                '--quality', str(self.quality),
                '--nopreview',
                '--immediate'  # Capture immediately without preview delay
            ], capture_output=True, text=True, timeout=10.0)
            
            if result.returncode == 0 and os.path.exists(temp_filename):
                # Read the captured image
                frame = cv2.imread(temp_filename)
                os.unlink(temp_filename)
                temp_file = None
                
                if frame is not None:
                    return True, frame
                else:
                    self.get_logger().warn("Failed to read captured image file")
                    return False, None
            else:
                self.get_logger().warn(f"rpicam-jpeg capture failed: {result.stderr}")
                if os.path.exists(temp_filename):
                    os.unlink(temp_filename)
                return False, None
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn("rpicam-jpeg capture timeout")
            if temp_file and os.path.exists(temp_file):
                os.unlink(temp_file)
            return False, None
        except Exception as e:
            self.get_logger().error(f"Capture error: {e}")
            if temp_file and os.path.exists(temp_file):
                os.unlink(temp_file)
            return False, None
    
    def capture_with_rpicam_still(self):
        """Alternative capture using rpicam-still"""
        temp_file = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp_file:
                temp_filename = tmp_file.name
                temp_file = temp_filename
            
            # Try rpicam-still as alternative
            result = subprocess.run([
                'rpicam-still',
                '-o', temp_filename,
                '--width', str(self.width),
                '--height', str(self.height),
                '--quality', str(self.quality),
                '--nopreview',
                '--immediate',
                '--timeout', '1'  # 1ms timeout for immediate capture
            ], capture_output=True, text=True, timeout=10.0)
            
            if result.returncode == 0 and os.path.exists(temp_filename):
                frame = cv2.imread(temp_filename)
                os.unlink(temp_filename)
                temp_file = None
                
                if frame is not None:
                    return True, frame
                else:
                    return False, None
            else:
                if os.path.exists(temp_filename):
                    os.unlink(temp_filename)
                return False, None
                
        except Exception as e:
            self.get_logger().error(f"rpicam-still capture error: {e}")
            if temp_file and os.path.exists(temp_file):
                os.unlink(temp_file)
            return False, None
    
    def initialize_camera(self):
        """Initialize the camera using rpicam-hello method"""
        self.get_logger().info("Initializing camera with rpicam method...")
        
        # Test if rpicam-hello works
        if not self.test_rpicam_hello():
            self.get_logger().error("Camera initialization failed - rpicam-hello not working")
            self.camera_working = False
            return
        
        self.camera_working = True
        self.camera_initialized = True
        self.get_logger().info("✓ Camera initialized successfully with rpicam method")
        
        # Start image capture timer
        timer_period = 1.0 / self.frame_rate
        self.capture_timer = self.create_timer(timer_period, self.capture_and_publish)
        
        self.get_logger().info("Started image capture loop")
    
    def capture_and_publish(self):
        """Capture and publish a single frame"""
        if not self.camera_working:
            return
        
        # Try rpicam-jpeg first
        success, frame = self.capture_with_rpicam_jpeg()
        
        # If that fails, try rpicam-still
        if not success:
            success, frame = self.capture_with_rpicam_still()
        
        if success and frame is not None:
            self.frame_count += 1
            self.publish_image(frame)
            
            # Log periodically
            if self.frame_count % 10 == 0:
                self.get_logger().info(f"Published {self.frame_count} frames")
        else:
            self.get_logger().warn("Failed to capture frame")
    
    def publish_image(self, frame):
        """Publish the captured image"""
        try:
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "rpicam_frame"
            
            # Publish image
            self.image_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")
    
    def publish_status(self):
        """Publish camera status"""
        status_msg = String()
        if self.camera_working:
            status_msg.data = f"RPICAM_ACTIVE: {self.width}x{self.height} @ {self.frame_rate}fps - Frames: {self.frame_count}"
        else:
            status_msg.data = "RPICAM_ERROR: Camera not functioning"
        
        self.status_pub.publish(status_msg)
    
    def get_available_rpicam_commands(self):
        """Check which rpicam commands are available"""
        available_commands = []
        for cmd in ['rpicam-hello', 'rpicam-jpeg', 'rpicam-still', 'rpicam-vid']:
            try:
                result = subprocess.run(['which', cmd], capture_output=True, text=True)
                if result.returncode == 0:
                    available_commands.append(cmd)
                    self.get_logger().info(f"Found: {cmd}")
            except:
                pass
        return available_commands
        
    def destroy_node(self):
        self.get_logger().info("Shutting down RPiCamera Node")
        self.get_logger().info(f"Total frames published: {self.frame_count}")
        super().destroy_node()

def main():
    rclpy.init()
    node = RPiCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
