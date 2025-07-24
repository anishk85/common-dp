FILE: scripts/camera_node.py
================================================================================
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from camera_info_manager import CameraInfoManager

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_type', 'usb_camera')  # 'pi_camera' or 'usb_camera'
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('auto_focus', True)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.auto_focus = self.get_parameter('auto_focus').get_parameter_value().bool_value
        
        # Initialize camera
        self.bridge = CvBridge()
        self.cap = None
        self.setup_camera()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # Camera info manager
        self.camera_info_manager = CameraInfoManager(self, self.camera_name)
        
        # Timer for publishing
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info(f'Camera node started: {self.camera_name} ({self.camera_type})')
        
    def setup_camera(self):
        """Initialize camera based on type"""
        try:
            if self.camera_type == 'pi_camera':
                self.setup_pi_camera()
            else:
                self.setup_usb_camera()
                
            if self.cap is not None and self.cap.isOpened():
                self.get_logger().info(f'Camera {self.camera_id} initialized successfully')
            else:
                self.get_logger().error(f'Failed to initialize camera {self.camera_id}')
                
        except Exception as e:
            self.get_logger().error(f'Camera setup error: {str(e)}')
            
    def setup_pi_camera(self):
        """Setup Raspberry Pi camera"""
        try:
            # For Pi camera, use libcamera-based approach or picamera2
            # This is a simplified OpenCV approach
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            
            if self.cap.isOpened():
                # Set resolution
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # Pi camera specific settings
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                if self.auto_focus:
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
                else:
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                    
        except Exception as e:
            self.get_logger().error(f'Pi camera setup error: {str(e)}')
            
    def setup_usb_camera(self):
        """Setup USB camera"""
        try:
            if isinstance(self.camera_id, str):
                # Device path provided
                self.cap = cv2.VideoCapture(self.camera_id)
            else:
                # Camera index provided
                self.cap = cv2.VideoCapture(self.camera_id)
                
            if self.cap.isOpened():
                # Set resolution
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # USB camera settings
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                
                if not self.auto_focus:
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                    
        except Exception as e:
            self.get_logger().error(f'USB camera setup error: {str(e)}')
            
    def publish_frame(self):
        """Capture and publish camera frame"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn('Camera not available')
            return
            
        try:
            ret, frame = self.cap.read()
            
            if ret and frame is not None:
                # Create timestamp
                timestamp = self.get_clock().now().to_msg()
                
                # Convert to ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header.stamp = timestamp
                image_msg.header.frame_id = self.frame_id
                
                # Publish image
                self.image_pub.publish(image_msg)
                
                # Create and publish camera info
                camera_info_msg = self.create_camera_info_msg()
                camera_info_msg.header.stamp = timestamp
                camera_info_msg.header.frame_id = self.frame_id
                self.camera_info_pub.publish(camera_info_msg)
                
            else:
                self.get_logger().warn('Failed to capture frame')
                
        except Exception as e:
            self.get_logger().error(f'Frame capture error: {str(e)}')
            
    def create_camera_info_msg(self):
        """Create camera info message with calibration data"""
        camera_info_msg = CameraInfo()
        
        # Try to load calibration from camera_info_manager
        if self.camera_info_manager.isCalibrated():
            camera_info_msg = self.camera_info_manager.getCameraInfo()
        else:
            # Use default values
            camera_info_msg.width = self.width
            camera_info_msg.height = self.height
            
            # Default camera matrix (approximate)
            fx = fy = self.width  # Rough estimate
            cx = self.width / 2.0
            cy = self.height / 2.0
            
            camera_info_msg.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ]
            
            camera_info_msg.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            camera_info_msg.distortion_model = "plumb_bob"
            
        return camera_info_msg
        
    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
