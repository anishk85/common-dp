#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # FIXED: Correct camera topics to match URDF configuration
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Matches gazebo output from URDF
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # Matches gazebo depth output from URDF
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',  # Matches gazebo info output from URDF
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.processed_image_pub = self.create_publisher(
            Image,
            '/perception/processed_image',
            10
        )
        
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/perception/pointcloud',
            10
        )
        
        self.detected_objects_pub = self.create_publisher(
            PoseStamped,
            '/perception/detected_objects',
            10
        )
        
        # Storage
        self.camera_info = None
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.image_count = 0
        self.depth_count = 0
        
        # Status check timer
        self.status_timer = self.create_timer(2.0, self.check_status)
        
        # Detection timer
        self.detection_timer = self.create_timer(0.5, self.detect_and_publish)
        
        self.get_logger().info("🔍 Enhanced Perception Node initialized")
        self.get_logger().info("📷 Listening for camera data...")
        self.get_logger().info("📷 Expected topics: /camera/image_raw, /camera/depth/image_raw, /camera/camera_info")
    def check_status(self):
        """Check status of data reception"""
        self.get_logger().info(f"📊 Status: RGB={self.image_count}, Depth={self.depth_count}, CameraInfo={'✅' if self.camera_info else '❌'}")
        
        if self.image_count == 0:
            self.get_logger().warn("⚠️ No RGB images received - check topic /camera/image_raw")
        if self.depth_count == 0:
            self.get_logger().warn("⚠️ No depth images received - check topic /camera/depth/image_raw")
    
    def camera_info_callback(self, msg):
        """Store camera info"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f"📊 Camera info received: {msg.width}x{msg.height}")
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.get_logger().info(f"📊 Camera intrinsics: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    
    def image_callback(self, msg):
        """Store RGB image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb_image = cv_image
            self.image_count += 1
            
            if self.image_count == 1:
                self.get_logger().info(f"📷 First RGB image received: {cv_image.shape}")
                self.get_logger().info(f"📷 Frame ID: {msg.header.frame_id}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing RGB image: {e}")
    
    def depth_callback(self, msg):
        """Store depth image"""
        try:
            # Try different encodings
            if msg.encoding == "32FC1":
                cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            elif msg.encoding == "16UC1":
                cv_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
                cv_depth = cv_depth.astype(np.float32) / 1000.0  # Convert to meters
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                
            self.latest_depth_image = cv_depth
            self.depth_count += 1
            
            if self.depth_count == 1:
                self.get_logger().info(f"📊 First depth image received: {cv_depth.shape}")
                self.get_logger().info(f"📊 Depth encoding: {msg.encoding}")
                self.get_logger().info(f"📊 Depth range: {np.min(cv_depth):.3f} to {np.max(cv_depth):.3f}m")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing depth image: {e}")
    
    def detect_and_publish(self):
        """Detect objects and publish results"""
        if self.latest_rgb_image is None:
            return
            
        try:
            # Object detection
            processed_image, detected_objects = self.detect_objects_advanced(self.latest_rgb_image)
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header.stamp = self.get_clock().now().to_msg()
            processed_msg.header.frame_id = "camera_optical_frame"
            self.processed_image_pub.publish(processed_msg)
            
            # Publish detected objects
            if detected_objects:
                for obj in detected_objects:
                    self.publish_object_pose(obj, processed_msg.header)
            
            # Generate point cloud
            if self.latest_depth_image is not None and self.camera_info is not None:
                self.generate_pointcloud(self.latest_rgb_image, self.latest_depth_image, processed_msg.header)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in detection: {e}")
    
    def detect_objects_advanced(self, image):
        """Advanced object detection with robot filtering"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define regions of interest (exclude robot parts)
        height, width = image.shape[:2]
        
        # Create mask to exclude robot (lower part of image)
        robot_mask = np.zeros((height, width), dtype=np.uint8)
        robot_mask[int(height*0.8):, :] = 255  # Exclude bottom 20%
        
        # Enhanced color ranges for better detection
        color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'red2': [(170, 100, 100), (180, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'green': [(40, 100, 100), (80, 255, 255)],
            'yellow': [(20, 100, 100), (40, 255, 255)],
            'orange': [(10, 100, 100), (25, 255, 255)],
            'purple': [(130, 100, 100), (160, 255, 255)]
        }
        
        result_image = image.copy()
        detected_objects = []
        
        for color_name, (lower, upper) in color_ranges.items():
            # Create color mask
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Apply robot mask (exclude robot parts)
            mask = cv2.bitwise_and(mask, cv2.bitwise_not(robot_mask))
            
            # Morphological operations
            kernel = np.ones((7,7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area for objects
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Filter by aspect ratio and position
                    aspect_ratio = w / h
                    if 0.3 < aspect_ratio < 3.0 and y < height * 0.7:  # Upper part of image
                        
                        center_x = x + w // 2
                        center_y = y + h // 2
                        
                        # Draw detection
                        cv2.rectangle(result_image, (x-5, y-5), (x+w+5, y+h+5), (0, 255, 0), 3)
                        cv2.putText(result_image, f"{color_name.replace('2', '')}", 
                                   (x, y-15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.putText(result_image, f"A:{int(area)}", 
                                   (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        detected_objects.append({
                            'name': f"{color_name.replace('2', '')}_object",
                            'center': (center_x, center_y),
                            'area': area,
                            'bbox': (x, y, w, h),
                            'position_3d': self.estimate_3d_position(center_x, center_y)
                        })
        
        # Add status info to image
        cv2.putText(result_image, f"Objects: {len(detected_objects)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(result_image, f"RGB: {self.image_count} | Depth: {self.depth_count}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        if detected_objects:
            self.get_logger().info(f"🎯 Detected {len(detected_objects)} objects")
        
        return result_image, detected_objects
    
    def estimate_3d_position(self, pixel_x, pixel_y):
        """Estimate 3D position from pixel coordinates"""
        if self.latest_depth_image is not None and self.camera_info is not None:
            try:
                # Ensure pixel coordinates are within image bounds
                height, width = self.latest_depth_image.shape
                pixel_x = max(0, min(pixel_x, width-1))
                pixel_y = max(0, min(pixel_y, height-1))
                
                depth = self.latest_depth_image[pixel_y, pixel_x]
                
                if depth > 0.1 and depth < 10.0:  # Valid depth range
                    # Convert pixel to 3D point using camera intrinsics
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx = self.camera_info.k[2]
                    cy = self.camera_info.k[5]
                    
                    x = (pixel_x - cx) * depth / fx
                    y = (pixel_y - cy) * depth / fy
                    z = depth
                    
                    return (x, y, z)
            except Exception as e:
                self.get_logger().debug(f"3D estimation error: {e}")
        
        # Fallback estimation
        return (0.5, 0.0, 0.3)
    
    def publish_object_pose(self, obj, header):
        """Publish object pose"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        
        pos_3d = obj['position_3d']
        pose_msg.pose.position.x = pos_3d[0]
        pose_msg.pose.position.y = pos_3d[1]
        pose_msg.pose.position.z = pos_3d[2]
        pose_msg.pose.orientation.w = 1.0
        
        self.detected_objects_pub.publish(pose_msg)
    
    def generate_pointcloud(self, rgb_image, depth_image, header):
        """Generate colored point cloud"""
        if self.camera_info is None:
            return
        
        # Camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        height, width = depth_image.shape
        points = []
        
        # Downsample for performance
        for v in range(0, height, 4):  # Every 4th pixel
            for u in range(0, width, 4):
                z = depth_image[v, u]
                
                if 0.1 < z < 8.0:  # Valid depth range
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # Get RGB color
                    b, g, r = rgb_image[v, u]
                    rgb_packed = struct.unpack('f', struct.pack('I', 
                                              (int(r) << 16) | (int(g) << 8) | int(b)))[0]
                    
                    points.append([x, y, z, rgb_packed])
        
        if len(points) > 0:
            pc2_msg = self.create_pointcloud2_msg(points, header)
            self.pointcloud_pub.publish(pc2_msg)
    
    def create_pointcloud2_msg(self, points, header):
        """Create PointCloud2 message"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        pc2_msg.data = struct.pack('f' * len(points) * 4, 
                                   *[item for point in points for item in point])
        
        return pc2_msg

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