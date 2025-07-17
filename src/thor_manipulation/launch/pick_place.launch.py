#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from moveit_msgs.msg import DisplayRobotState
from moveit_msgs.srv import GetPositionIK
import moveit_commander
import sys
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

class ThorPickPlace(Node):
    def __init__(self):
        super().__init__('thor_pick_place')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("thor_arm")
        
        # Set planning parameters
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)
        
        # Publishers/Subscribers
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Object detection
        self.object_detected = False
        self.object_pose = None
        
        # Predefined poses
        self.home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.place_position = [0.3, 0.3, 0.2]  # Adjust as needed
        
        self.get_logger().info("Thor Pick and Place initialized")
        
    def image_callback(self, msg):
        """Process camera image to detect objects"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_object(cv_image)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def detect_object(self, image):
        """Simple object detection using color thresholding"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color range for object (adjust for your object)
        lower_color = np.array([0, 100, 100])  # Red lower bound
        upper_color = np.array([10, 255, 255])  # Red upper bound
        
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                # Get centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Convert pixel coordinates to world coordinates
                    self.pixel_to_world(cx, cy)
                    
    def pixel_to_world(self, x, y):
        """Convert pixel coordinates to world coordinates"""
        # Simple projection (you need camera calibration for accuracy)
        # Assuming camera is at (0.5, 0, 1.5) looking down
        # This is a simplified conversion - calibrate your camera properly
        
        world_x = (x - 320) * 0.001  # Scale factor
        world_y = (y - 240) * 0.001
        world_z = 0.05  # Object height
        
        self.object_pose = Pose()
        self.object_pose.position.x = world_x
        self.object_pose.position.y = world_y
        self.object_pose.position.z = world_z
        self.object_pose.orientation.w = 1.0
        
        self.object_detected = True
        self.get_logger().info(f"Object detected at: {world_x:.3f}, {world_y:.3f}, {world_z:.3f}")
        
    def go_to_pose(self, pose_target):
        """Move to target pose"""
        self.group.set_pose_target(pose_target)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return plan
        
    def go_to_joint_state(self, joint_goal):
        """Move to joint positions"""
        self.group.go(joint_goal, wait=True)
        self.group.stop()
        
    def control_electromagnet(self, state):
        """Control electromagnet on/off"""
        msg = Bool()
        msg.data = state
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"Electromagnet: {'ON' if state else 'OFF'}")
        
    def pick_and_place(self):
        """Main pick and place sequence"""
        if not self.object_detected:
            self.get_logger().warn("No object detected")
            return False
            
        try:
            # 1. Go to home position
            self.get_logger().info("Going to home position")
            self.go_to_joint_state(self.home_pose)
            
            # 2. Move to pre-grasp position (above object)
            pre_grasp_pose = PoseStamped()
            pre_grasp_pose.header.frame_id = "base_link"
            pre_grasp_pose.pose = self.object_pose
            pre_grasp_pose.pose.position.z += 0.1  # 10cm above object
            
            self.get_logger().info("Moving to pre-grasp position")
            if not self.go_to_pose(pre_grasp_pose.pose):
                self.get_logger().error("Failed to reach pre-grasp position")
                return False
                
            # 3. Move to grasp position
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = "base_link"
            grasp_pose.pose = self.object_pose
            grasp_pose.pose.position.z += 0.02  # Just above object
            
            self.get_logger().info("Moving to grasp position")
            if not self.go_to_pose(grasp_pose.pose):
                self.get_logger().error("Failed to reach grasp position")
                return False
                
            # 4. Activate electromagnet
            self.control_electromagnet(True)
            self.create_timer(1.0, lambda: None)  # Wait 1 second
            
            # 5. Lift object
            lift_pose = grasp_pose.pose
            lift_pose.position.z += 0.1
            
            self.get_logger().info("Lifting object")
            if not self.go_to_pose(lift_pose):
                self.get_logger().error("Failed to lift object")
                return False
                
            # 6. Move to place position
            place_pose = PoseStamped()
            place_pose.header.frame_id = "base_link"
            place_pose.pose.position.x = self.place_position[0]
            place_pose.pose.position.y = self.place_position[1]
            place_pose.pose.position.z = self.place_position[2]
            place_pose.pose.orientation.w = 1.0
            
            self.get_logger().info("Moving to place position")
            if not self.go_to_pose(place_pose.pose):
                self.get_logger().error("Failed to reach place position")
                return False
                
            # 7. Lower object
            lower_pose = place_pose.pose
            lower_pose.position.z -= 0.05
            
            self.get_logger().info("Lowering object")
            if not self.go_to_pose(lower_pose):
                self.get_logger().error("Failed to lower object")
                return False
                
            # 8. Deactivate electromagnet
            self.control_electromagnet(False)
            self.create_timer(1.0, lambda: None)  # Wait 1 second
            
            # 9. Return to home
            self.get_logger().info("Returning to home")
            self.go_to_joint_state(self.home_pose)
            
            self.get_logger().info("Pick and place completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Pick and place failed: {e}")
            self.control_electromagnet(False)  # Safety: turn off electromagnet
            return False
            
    def run(self):
        """Main execution loop"""
        self.get_logger().info("Waiting for object detection...")
        
        # Wait for object detection
        while not self.object_detected and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.object_detected:
            self.pick_and_place()

def main():
    rclpy.init()
    node = ThorPickPlace()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()