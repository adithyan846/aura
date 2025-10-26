#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import sys

# Import your custom message
from aura_interfaces.msg import Telemetry

class PixhawkNode(Node):
    def __init__(self):
        super().__init__('pixhawk_node')

        # --- Parameters ---
        self.declare_parameter('connection_string', '/dev/ttyAMA0') # Default for RPi 5 hardware UART
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('poll_rate_hz', 20.0) # Rate to poll for MAVLink msgs
        self.declare_parameter('stream_rate_hz', 10.0) # Rate to request MAVLink streams
        
        conn_string = self.get_parameter('connection_string').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        poll_rate = self.get_parameter('poll_rate_hz').get_parameter_value().double_value
        self.stream_rate = self.get_parameter('stream_rate_hz').get_parameter_value().double_value

        self.get_logger().info(f"Connecting to Pixhawk on {conn_string} at {baud} baud...")

        # --- MAVLink Connection ---
        try:
            self.master = mavutil.mavlink_connection(conn_string, baud=baud)
            self.master.wait_heartbeat()
            self.get_logger().info("Heartbeat received! MAVLink connection established.")
        except Exception as e:
            self.get_logger().fatal(f"Failed to connect to Pixhawk: {e}")
            self.get_logger().fatal("Shutting down node. Check connection, permissions, and parameters.")
            sys.exit(1) # Exit the script

        # --- Request Data Streams ---
        self.request_data_streams()

        # --- ROS 2 Publisher ---
        self.telemetry_pub = self.create_publisher(Telemetry, '/telemetry', 10)

        # --- ROS 2 Timer ---
        self.poll_timer = self.create_timer(1.0 / poll_rate, self.poll_telemetry_callback)

        # --- Local Telemetry Cache ---
        # We cache the latest data from MAVLink messages
        self.latest_telemetry = Telemetry()
        self.last_pos_msg_time = 0
        self.last_bat_msg_time = 0
        self.last_hud_msg_time = 0


    def request_data_streams(self):
        """Requests MAVLink data streams from the Pixhawk."""
        self.get_logger().info(f"Requesting data streams at {self.stream_rate} Hz...")
        
        # Interval in microseconds (1,000,000 / rate_hz)
        interval_us = int(1_000_000 / self.stream_rate)
        
        # MAVLink message IDs
        msg_ids_to_stream = {
            'GLOBAL_POSITION_INT': 33,
            'SYS_STATUS': 1,
            'VFR_HUD': 74,
            'ATTITUDE': 30,         # <-- NEW: Orientation data
            'SCALED_IMU2': 117,     # <-- NEW: Acceleration data
        }

        for msg_name, msg_id in msg_ids_to_stream.items():
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,      # confirmation
                    msg_id, # param1: The MAVLink message ID
                    interval_us, # param2: Interval in microseconds
                    0, 0, 0, 0, 0 # params 3-7 (unused)
                )
                self.get_logger().info(f"Requested {msg_name} (ID {msg_id}) at {interval_us} us interval.")
                time.sleep(0.1) # Short delay between requests
            except Exception as e:
                self.get_logger().error(f"Failed to request stream for {msg_name}: {e}")

    def poll_telemetry_callback(self):
        """
        Polls for MAVLink messages and publishes a consolidated ROS 2 Telemetry message.
        """
        # ... (start of function remains the same) ...

        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            
            msg_type = msg.get_type()

            # --- Update local cache based on message type ---
            if msg_type == 'GLOBAL_POSITION_INT':
                # ... (existing code for lat, lon, alt, heading) ...
                self.latest_telemetry.heading = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0
                self.last_pos_msg_time = time.time()
                
            elif msg_type == 'SYS_STATUS':
                # ... (existing code for battery) ...
                self.last_bat_msg_time = time.time()

            elif msg_type == 'VFR_HUD':
                # ... (existing code for velocity) ...
                self.last_hud_msg_time = time.time()

            # --- NEW: ATTITUDE (Roll/Pitch/Yaw) ---
            elif msg_type == 'ATTITUDE':
                # MAVLink uses radians for these fields
                # We'll keep them as radians here or convert them to degrees for the ROS message
                self.latest_telemetry.roll = msg.roll
                self.latest_telemetry.pitch = msg.pitch
                self.latest_telemetry.yaw = msg.yaw
            
            # --- NEW: SCALED_IMU2 (Acceleration in milli-g) ---
            elif msg_type == 'SCALED_IMU2':
                # MAVLink SCALED_IMU values are in milli-g (10^-3 g)
                # We convert them to m/s^2 (1g ≈ 9.80665 m/s^2)
                G_MS2 = 9.80665
                
                self.latest_telemetry.accel_x = (msg.xacc / 1000.0) * G_MS2
                self.latest_telemetry.accel_y = (msg.yacc / 1000.0) * G_MS2
                self.latest_telemetry.accel_z = (msg.zacc / 1000.0) * G_MS2
        
        # --- Publish the consolidated message (check all key data points) ---
        # NOTE: You'll want to add checks for ATTITUDE and SCALED_IMU if you need to guarantee they are present before publishing
        # For simplicity, we'll keep the existing checks for now and let the new fields default to 0.0 if not yet received.
#        if self.last_pos_msg_time > 0 and self.last_bat_msg_time > 0 and self.last_hud_msg_time > 0:
        self.telemetry_pub.publish(self.latest_telemetry)
#        else:
            # Still waiting for initial GPS/Battery/HUD data
 #           pass


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
