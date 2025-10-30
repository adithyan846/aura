#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from aura_interfaces.msg import Detection, Telemetry
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class AiNode(Node):
    def __init__(self):
        super().__init__('ai_node')
        self.get_logger().info('AI Node starting. Loading YOLOv11n model...')
        
        # --- Configuration Parameters ---
        model_path = '/home/akari/aura/automation/yolo/best_ncnn_model'
        
        # Model parameters
        self.confidence_threshold = 0.25  # Adjust based on your needs
        self.iou_threshold = 0.45
        
        # --- AI Model Initialization ---
        try:
            self.model = YOLO(model_path, task='detect')  # Explicitly set task
            self.get_logger().info(f'YOLOv11n model loaded successfully. Classes: {len(self.model.names)}')
            self.get_logger().info(f'Available classes: {self.model.names}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise RuntimeError("YOLO model loading failed.")

        # Bridge to convert ROS Images to OpenCV
        self.bridge = CvBridge()
        
        # Store latest telemetry data
        self.current_telemetry = Telemetry()
        self.current_telemetry.latitude = 0.0
        self.current_telemetry.longitude = 0.0
        
        # Performance tracking
        self.frame_count = 0
        self.detection_count = 0
        self.last_log_time = self.get_clock().now()
        
        # --- Subscribers ---
        self.image_subscriber = self.create_subscription(
            Image,
            '/visuals/image_raw',
            self.image_callback,
            1)
        
        self.telemetry_subscriber = self.create_subscription(
            Telemetry,
            '/telemetry',
            self.telemetry_callback,
            10)
            
        # --- Publishers ---
        self.detection_publisher = self.create_publisher(
            Detection,
            '/detection',
            10)
        
        self.get_logger().info('AI Node initialized and ready to subscribe to /visuals/image_raw.')

    def telemetry_callback(self, msg):
        """Store latest telemetry data"""
        self.current_telemetry = msg

    def image_callback(self, msg):
        """Process incoming image and run object detection"""
        self.frame_count += 1
        
        # 1. Convert ROS Image message to OpenCV image
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # 2. Run YOLOv11n Inference
        try:
            results = self.model(
                cv_frame, 
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False,
                max_det=10
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return
        
        # 3. Process results
        detections_found = False
        
        for r in results:
            if r.boxes is not None and len(r.boxes) > 0:
                for i, box in enumerate(r.boxes):
                    # Extract bounding box coordinates
                    x_center, y_center, w, h = box.xywh[0].cpu().numpy()
                    
                    # Get prediction data
                    label_id = int(box.cls[0].cpu().numpy())
                    confidence = float(box.conf[0].cpu().numpy())
                    label = self.model.names.get(label_id, "person")  # Default to 'person'
                    
                    # Create and populate Detection message (without header)
                    detection_msg = Detection()
                    
                    # Populate detection data
                    detection_msg.label = label
                    detection_msg.confidence = confidence
                    detection_msg.x = float(x_center)
                    detection_msg.y = float(y_center) 
                    detection_msg.width = float(w)
                    detection_msg.height = float(h)
                    
                    # Populate telemetry data
                    detection_msg.latitude = self.current_telemetry.latitude
                    detection_msg.longitude = self.current_telemetry.longitude
                    
                    # Publish detection
                    self.detection_publisher.publish(detection_msg)
                    self.detection_count += 1
                    detections_found = True
                    
                    # Log first detection in frame
                    if i == 0:
                        self.get_logger().info(
                            f'Detected {label} (conf: {confidence:.3f}) at [{x_center:.1f}, {y_center:.1f}]',
                            throttle_duration_sec=2.0
                        )
        
        # Periodic status logging
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time).nanoseconds > 5e9:  # Every 5 seconds
            self.get_logger().info(
                f'Status: {self.frame_count} frames processed, '
                f'{self.detection_count} total detections'
            )
            self.last_log_time = current_time
        
        # Debug logging for no detections (less frequent)
        if not detections_found:
            self.get_logger().debug(
                f'Frame {self.frame_count}: No human detections', 
                throttle_duration_sec=10.0
            )

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info(
            f'AI Node shutting down. Processed {self.frame_count} frames, '
            f'made {self.detection_count} detections.'
        )
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        ai_node = AiNode()
        rclpy.spin(ai_node)
    except RuntimeError as e:
        print(f"Failed to start AI Node: {e}")
    except KeyboardInterrupt:
        print("AI Node stopped by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'ai_node' in locals():
            ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
