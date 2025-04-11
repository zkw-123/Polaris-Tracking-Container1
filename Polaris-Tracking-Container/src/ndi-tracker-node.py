#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import numpy as np
from sksurgerynditracker.nditracker import NDITracker
import os
import datetime
import json
import threading

class NDITrackerNode(Node):
    """
    ROS 2 Node for publishing NDI tracker data as tf2 transforms.
    Records transforms to log files and supports inter-container communication.
    """
    
    def __init__(self):
        super().__init__('ndi_tracker_node')
        
        # Declare parameters
        self.declare_parameter('tracker_type', 'spectra')  # Changed default to spectra
        self.declare_parameter('rom_files_dir', '/opt/ndi/rom_files')
        self.declare_parameter('use_dummy', True)
        self.declare_parameter('serial_port', '')  # For Polaris Spectra
        self.declare_parameter('update_rate', 30.0)  # Hz
        self.declare_parameter('log_dir', '/ros2_ws/logs')
        self.declare_parameter('log_enabled', True)
        
        # Get parameters
        self.tracker_type = self.get_parameter('tracker_type').value
        self.rom_files_dir = self.get_parameter('rom_files_dir').value
        self.use_dummy = self.get_parameter('use_dummy').value
        self.serial_port = self.get_parameter('serial_port').value
        update_rate = self.get_parameter('update_rate').value
        self.log_dir = self.get_parameter('log_dir').value
        self.log_enabled = self.get_parameter('log_enabled').value
        
        # Create log directory if it doesn't exist
        if self.log_enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file_path = os.path.join(self.log_dir, f"log_{timestamp}.json")
            self.get_logger().info(f"Logging transforms to: {self.log_file_path}")
            self.log_file_lock = threading.Lock()
            
        # For tracking data publishing to other containers
        self.transform_publisher = self.create_publisher(String, 'ndi_transforms', 10)
        
        # Service for receiving commands from other containers
        self.cmd_service = self.create_service(
            Trigger, 'ndi_command', self.handle_command)
        
        # Initialize tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize tracker settings
        self.get_logger().info(f"Initializing {self.tracker_type} tracker...")
        
        # Find ROM files in the specified directory
        rom_files = []
        if not self.use_dummy:
            if os.path.exists(self.rom_files_dir):
                rom_files = [os.path.join(self.rom_files_dir, f) 
                            for f in os.listdir(self.rom_files_dir) 
                            if f.endswith('.rom')]
                self.get_logger().info(f"Found ROM files: {rom_files}")
            else:
                self.get_logger().warn(f"ROM files directory {self.rom_files_dir} does not exist")
        
        # Configure tracker based on type
        if self.use_dummy:
            self.settings = {"tracker type": "dummy"}
            if rom_files:
                self.settings["romfiles"] = rom_files
            self.get_logger().info("Using dummy tracker")
        elif self.tracker_type == "spectra":
            self.settings = {
                "tracker type": "polaris",  # Both Polaris Spectra and Vicra use "polaris" as tracker type
                "romfiles": rom_files
            }
            # Add serial port if specified
            if self.serial_port:
                self.settings["serial port"] = self.serial_port
                
            self.get_logger().info(f"Using Polaris Spectra tracker with {len(rom_files)} ROM files")
        else:
            self.get_logger().error(f"Unsupported tracker type: {self.tracker_type}. Only 'spectra' or 'dummy' are supported.")
            return
        
        try:
            # Initialize tracker
            self.tracker = NDITracker(self.settings)
            
            # Get tool descriptions for logging
            port_handles, descriptions = self.tracker.get_tool_descriptions()
            for i, (handle, desc) in enumerate(zip(port_handles, descriptions)):
                self.get_logger().info(f"Tool {i}: Handle {handle}, Description: {desc}")
            
            # Start tracking
            self.tracker.start_tracking()
            self.get_logger().info("Tracking started")
            
            # Create timer for updating transforms
            self.timer = self.create_timer(1.0 / update_rate, self.update_transforms_callback)
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize tracker: {str(e)}")
            if hasattr(self, 'tracker'):
                self.tracker.close()
    
    def handle_command(self, request, response):
        """
        Service callback to handle commands from other containers.
        Currently supports:
        - "start_logging": Start logging transforms
        - "stop_logging": Stop logging transforms
        - "get_status": Get tracker status
        """
        cmd = request.data if hasattr(request, 'data') else ""
        
        if cmd == "start_logging" or cmd == "":
            if not self.log_enabled:
                self.log_enabled = True
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                self.log_file_path = os.path.join(self.log_dir, f"log_{timestamp}.json")
                self.get_logger().info(f"Started logging transforms to: {self.log_file_path}")
                response.success = True
                response.message = f"Started logging to {self.log_file_path}"
            else:
                response.success = True
                response.message = f"Already logging to {self.log_file_path}"
        
        elif cmd == "stop_logging":
            if self.log_enabled:
                self.log_enabled = False
                self.get_logger().info("Stopped logging transforms")
                response.success = True
                response.message = "Stopped logging transforms"
            else:
                response.success = True
                response.message = "Logging was already disabled"
        
        elif cmd == "get_status":
            status = {
                "tracker_type": "Polaris Spectra" if self.tracker_type == "spectra" else "Dummy",
                "logging_enabled": self.log_enabled,
                "log_file": self.log_file_path if hasattr(self, 'log_file_path') and self.log_enabled else None
            }
            response.success = True
            response.message = json.dumps(status)
        
        else:
            response.success = False
            response.message = f"Unknown command: {cmd}"
        
        return response

    def update_transforms_callback(self):
        """
        Timer callback to update and publish transforms from the NDI tracker.
        Also logs the transforms to a file and publishes them for inter-container communication.
        """
        try:
            # Get a frame of tracking data
            port_handles, timestamps, framenumbers, tracking, tracking_quality = self.tracker.get_frame()
            
            # Current ROS time for the transform header
            now = self.get_clock().now().to_msg()
            timestamp_str = datetime.datetime.now().isoformat()
            
            # Data for logging and inter-container communication
            log_data = {
                "timestamp": timestamp_str,
                "transforms": []
            }
            
            # Publish each tool's transform
            for i, (handle, transform, quality) in enumerate(zip(port_handles, tracking, tracking_quality)):
                # Only process if tracking quality is good (not NaN)
                if not np.isnan(quality):
                    # Create transform message
                    t = TransformStamped()
                    t.header.stamp = now
                    t.header.frame_id = 'ndi_world'
                    t.child_frame_id = f'tool_{handle}'
                    
                    transform_data = {
                        "tool_id": handle,
                        "quality": float(quality)
                    }
                    
                    # Extract rotation and translation from 4x4 matrix
                    if transform.shape == (4, 4):  # Regular matrix format
                        rotation_matrix = transform[0:3, 0:3]
                        translation = transform[0:3, 3]
                        
                        # Convert rotation matrix to quaternion
                        q = self.rotation_matrix_to_quaternion(rotation_matrix)
                        
                        # Set translation
                        t.transform.translation.x = float(translation[0])
                        t.transform.translation.y = float(translation[1])
                        t.transform.translation.z = float(translation[2])
                        
                        # Set rotation
                        t.transform.rotation.x = float(q[0])
                        t.transform.rotation.y = float(q[1])
                        t.transform.rotation.z = float(q[2])
                        t.transform.rotation.w = float(q[3])
                        
                        # Add matrix data for logging
                        transform_data["matrix"] = transform.tolist()
                        transform_data["translation"] = translation.tolist()
                        transform_data["quaternion"] = q
                        
                    elif transform.shape == (1, 7):  # Quaternion format
                        # Format is [qw, qx, qy, qz, x, y, z]
                        t.transform.rotation.w = float(transform[0, 0])
                        t.transform.rotation.x = float(transform[0, 1])
                        t.transform.rotation.y = float(transform[0, 2])
                        t.transform.rotation.z = float(transform[0, 3])
                        t.transform.translation.x = float(transform[0, 4])
                        t.transform.translation.y = float(transform[0, 5])
                        t.transform.translation.z = float(transform[0, 6])
                        
                        # Add data for logging
                        transform_data["quaternion"] = [
                            float(transform[0, 0]),  # w
                            float(transform[0, 1]),  # x
                            float(transform[0, 2]),  # y
                            float(transform[0, 3])   # z
                        ]
                        transform_data["translation"] = [
                            float(transform[0, 4]),  # x
                            float(transform[0, 5]),  # y
                            float(transform[0, 6])   # z
                        ]
                    
                    # Publish the transform via TF
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Add to log data
                    log_data["transforms"].append(transform_data)
            
            # Only log and publish if we have transforms to report
            if log_data["transforms"]:
                # Write to log file if enabled
                if self.log_enabled:
                    with self.log_file_lock:
                        with open(self.log_file_path, 'a') as f:
                            f.write(json.dumps(log_data) + '\n')
                
                # Publish transform data for inter-container communication
                self.transform_publisher.publish(String(data=json.dumps(log_data)))
                    
        except Exception as e:
            self.get_logger().error(f"Error getting tracking data: {str(e)}")
    
    def rotation_matrix_to_quaternion(self, R):
        """
        Convert a rotation matrix to quaternion [x, y, z, w]
        """
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
            
        return [qx, qy, qz, qw]
    
    def destroy_node(self):
        """
        Clean up when the node is destroyed.
        """
        if hasattr(self, 'tracker'):
            try:
                self.get_logger().info("Stopping tracking...")
                self.tracker.stop_tracking()
                self.tracker.close()
            except Exception as e:
                self.get_logger().error(f"Error closing tracker: {str(e)}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NDITrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
