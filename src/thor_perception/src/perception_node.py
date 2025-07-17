#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribers - Try both camera topics
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Try the generic one first
            self.image_callback,
            10
        )
        
        self.thor_image_sub = self.create_subscription(
            Image,
            '/thor_arm/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.processed_image_pub = self.create_publisher(
            Image,
            '/perception/processed_image',
            10
        )
        
        self.detected_objects_pub = self.create_publisher(
            PoseStamped,
            '/perception/detected_objects',
            10
        )
        
        # Storage
        self.latest_rgb_image = None
        self.image_received = False
        
        # Timer to check for images
        self.check_timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info("🔍 Perception node initialized")
        self.get_logger().info("📷 Listening to camera topics...")
    
    def check_status(self):
        """Check if we're receiving images"""
        if not self.image_received:
            self.get_logger().warn("⚠️ No camera images received yet...")
            # List available topics
            try:
                import subprocess
                result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True)
                topics = result.stdout.split('\n')
                camera_topics = [t for t in topics if 'image' in t and 'camera' in t]
                self.get_logger().info(f"📷 Available camera topics: {camera_topics}")
            except:
                pass
    
    def image_callback(self, msg):
        """Process RGB images"""
        try:
            if not self.image_received:
                self.image_received = True
                self.get_logger().info(f"📷 Receiving images from: {msg.header.frame_id}")
                self.check_timer.cancel()
            
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb_image = cv_image
            
            # Object detection
            processed_image, detected_objects = self.detect_objects(cv_image)
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
            
            # Publish detected objects
            if detected_objects:
                self.publish_detected_objects(detected_objects, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")
    
    def detect_objects(self, image):
        """Simple object detection using color segmentation"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Color ranges for different objects
        color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'green': [(50, 100, 100), (80, 255, 255)],
            'yellow': [(20, 100, 100), (40, 255, 255)]
        }
        
        result_image = image.copy()
        detected_objects = []
        
        for color_name, (lower, upper) in color_ranges.items():
            # Create mask for this color
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process contours
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Filter small objects
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center point
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Draw bounding box
                    cv2.rectangle(result_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(result_image, f"{color_name}", (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Add to detected objects
                    detected_objects.append({
                        'name': f"{color_name}_object",
                        'center': (center_x, center_y),
                        'area': area
                    })
        
        return result_image, detected_objects
    
    def publish_detected_objects(self, detected_objects, header):
        """Publish detected objects as poses"""
        for obj in detected_objects:
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = "camera_link"
            
            # Simple position estimation
            pose_msg.pose.position.x = 0.5  # Forward from camera
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            self.detected_objects_pub.publish(pose_msg)
            self.get_logger().info(f"🎯 Detected {obj['name']} at center {obj['center']}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()