#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
import numpy as np
from tf_transformations import quaternion_from_euler
from enum import Enum, auto
import cv2
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import message_filters

class PickPlaceState(Enum):
    IDLE = auto()
    DETECTING = auto()
    MOVING_TO_APPROACH = auto()
    MOVING_TO_PICK = auto()
    PICKING = auto()
    LIFTING = auto()
    MOVING_TO_PLACE = auto()
    PLACING = auto()
    RETREATING = auto()
    RETURNING_HOME = auto()

class EnhancedPickPlaceNode(Node):
    def __init__(self):
        super().__init__('enhanced_pick_place_node')
        
        # State management
        self.state = PickPlaceState.IDLE
        self.object_pose = None
        self.detected_objects = []
        
        # Computer vision setup
        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')  # You can use yolov8s.pt for better accuracy
        
        # Camera parameters
        self.camera_info = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/thor_arm/pick_place_status', 10)
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        self.detection_viz_pub = self.create_publisher(Image, '/detection_visualization', 10)
        
        # Subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info')
        self.command_sub = self.create_subscription(
            String, '/user_command', self.command_callback, 10)
        
        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], 10, 0.1)
        self.sync.registerCallback(self.camera_callback)
        
        # Robot configuration
        self.arm_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.planning_frame = "base_link"
        self.camera_frame = "camera_link"
        
        # Predefined poses
        self.home_pose = self.create_pose(0.2, 0.0, 0.4, 0, np.pi/2, 0)
        self.place_pose = self.create_pose(-0.2, 0.3, 0.1, 0, np.pi/2, 0)
        
        # Detection parameters
        self.detection_active = False
        self.target_classes = ['bottle', 'cup', 'bowl', 'apple', 'orange', 'banana']  # COCO classes
        self.min_confidence = 0.5
        
        self.get_logger().info("🤖 Enhanced Pick and Place Node with YOLO initialized")
        self.get_logger().info("Waiting for MoveGroup action server...")
        
        # Wait for MoveGroup
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ Connected to MoveGroup action server")
        
        # Start in home position
        self.move_to_home()
        
        # Print available commands
        self.print_commands()

    def print_commands(self):
        """Print available commands"""
        commands = """
        🎮 Available Commands:
        - detect: Start object detection
        - pick: Pick detected object
        - place: Place object at predefined location
        - home: Return to home position
        - stop: Stop detection
        - status: Show current status
        """
        self.get_logger().info(commands)

    def create_pose(self, x, y, z, roll, pitch, yaw):
        """Create a Pose with given position and orientation"""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        
        return pose

    def camera_callback(self, image_msg, camera_info_msg):
        """Process camera data"""
        if not self.detection_active:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Store camera info
            if self.camera_info is None:
                self.camera_info = camera_info_msg
                self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
                self.dist_coeffs = np.array(camera_info_msg.d)
            
            # Run YOLO detection
            results = self.yolo_model(cv_image)
            
            # Process detections
            detections = self.process_detections(results[0], cv_image)
            
            if detections:
                # Convert best detection to 3D pose
                best_detection = max(detections, key=lambda x: x['confidence'])
                object_pose = self.pixel_to_3d_pose(best_detection, image_msg.header)
                
                if object_pose:
                    self.object_pose = object_pose
                    self.publish_status(f"OBJECT_DETECTED: {best_detection['class']}")
                    self.get_logger().info(f"📦 Detected {best_detection['class']} at "
                                         f"({object_pose.pose.position.x:.3f}, "
                                         f"{object_pose.pose.position.y:.3f}, "
                                         f"{object_pose.pose.position.z:.3f})")
            
            # Publish visualization
            self.publish_detection_viz(cv_image, detections)
            
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def process_detections(self, results, image):
        """Process YOLO detection results"""
        detections = []
        
        if results.boxes is not None:
            for box in results.boxes:
                class_id = int(box.cls[0])
                class_name = self.yolo_model.names[class_id]
                confidence = float(box.conf[0])
                
                # Filter by confidence and target classes
                if confidence > self.min_confidence and class_name in self.target_classes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    detection = {
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': [x1, y1, x2, y2],
                        'center': [center_x, center_y],
                        'width': x2 - x1,
                        'height': y2 - y1
                    }
                    detections.append(detection)
        
        return detections

    def pixel_to_3d_pose(self, detection, header):
        """Convert pixel coordinates to 3D pose"""
        try:
            # Use camera matrix to convert pixel to ray
            center_x, center_y = detection['center']
            
            # Normalize pixel coordinates
            x_norm = (center_x - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
            y_norm = (center_y - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
            
            # Assume object is on table (z = 0.05m above base_link)
            # This is a simplified approach - you can improve with depth estimation
            assumed_z = 0.05
            
            # Create pose in camera frame
            camera_pose = PoseStamped()
            camera_pose.header = header
            camera_pose.header.frame_id = self.camera_frame
            
            # Estimate 3D position (simplified - assumes known height)
            # You may need to adjust these values based on your camera setup
            camera_pose.pose.position.x = 0.5  # Assume 50cm in front of camera
            camera_pose.pose.position.y = x_norm * camera_pose.pose.position.x
            camera_pose.pose.position.z = -y_norm * camera_pose.pose.position.x
            
            # Standard orientation for pickup
            q = quaternion_from_euler(0, 0, 0)
            camera_pose.pose.orientation.x = q[0]
            camera_pose.pose.orientation.y = q[1]
            camera_pose.pose.orientation.z = q[2]
            camera_pose.pose.orientation.w = q[3]
            
            # Transform to base_link frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.planning_frame, self.camera_frame, rclpy.time.Time())
                base_pose = do_transform_pose(camera_pose, transform)
                
                # Ensure z is above table
                base_pose.pose.position.z = max(base_pose.pose.position.z, 0.05)
                
                return base_pose
                
            except Exception as e:
                self.get_logger().error(f"TF transform error: {e}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Pixel to 3D conversion error: {e}")
            return None

    def publish_detection_viz(self, image, detections):
        """Publish detection visualization"""
        viz_image = image.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = [int(x) for x in detection['bbox']]
            class_name = detection['class']
            confidence = detection['confidence']
            
            # Draw bounding box
            cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(viz_image, label, (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw center point
            center_x, center_y = detection['center']
            cv2.circle(viz_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Convert back to ROS message
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "bgr8")
            self.detection_viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().error(f"Visualization publishing error: {e}")

    def command_callback(self, msg):
        """Handle incoming commands"""
        command = msg.data.lower().strip()
        
        if command == "detect":
            self.start_detection()
        elif command == "stop":
            self.stop_detection()
        elif command == "pick" and self.object_pose is not None:
            self.start_pick_sequence()
        elif command == "pick":
            self.get_logger().warn("⚠️ No object detected. Run 'detect' first.")
        elif command == "place":
            self.move_to_place()
        elif command == "home":
            self.move_to_home()
        elif command == "status":
            self.print_status()
        else:
            self.get_logger().warn(f"❌ Unknown command: {command}")
            self.print_commands()

    def start_detection(self):
        """Start object detection"""
        self.detection_active = True
        self.state = PickPlaceState.DETECTING
        self.get_logger().info("🔍 Starting camera-based object detection...")
        self.publish_status("DETECTING")

    def stop_detection(self):
        """Stop object detection"""
        self.detection_active = False
        self.get_logger().info("🛑 Stopping object detection")
        self.publish_status("DETECTION_STOPPED")

    def print_status(self):
        """Print current status"""
        status = f"""
        📊 System Status:
        - State: {self.state.name}
        - Detection Active: {self.detection_active}
        - Object Detected: {'Yes' if self.object_pose else 'No'}
        - Camera Info: {'Available' if self.camera_info else 'Not Available'}
        """
        self.get_logger().info(status)

    def start_pick_sequence(self):
        """Start the pick and place sequence"""
        if self.object_pose is None:
            self.get_logger().error("❌ No object pose available")
            return
            
        self.detection_active = False  # Stop detection during manipulation
        self.get_logger().info("🚀 Starting pick sequence...")
        self.state = PickPlaceState.MOVING_TO_APPROACH
        
        # Create approach pose (above object)
        approach_pose = Pose()
        approach_pose.position.x = self.object_pose.pose.position.x
        approach_pose.position.y = self.object_pose.pose.position.y
        approach_pose.position.z = self.object_pose.pose.position.z + 0.15  # 15cm above
        
        # Point downward
        q = quaternion_from_euler(0, np.pi, 0)
        approach_pose.orientation.x = q[0]
        approach_pose.orientation.y = q[1]
        approach_pose.orientation.z = q[2]
        approach_pose.orientation.w = q[3]
        
        self.move_to_pose(approach_pose, "Moving to approach position")

    def move_to_home(self):
        """Move to home position"""
        self.state = PickPlaceState.RETURNING_HOME
        self.move_to_pose(self.home_pose, "Moving to home position")

    def move_to_place(self):
        """Move to place position"""
        self.state = PickPlaceState.MOVING_TO_PLACE
        self.move_to_pose(self.place_pose, "Moving to place position")

    def move_to_pose(self, target_pose, description):
        """Move robot to target pose"""
        self.get_logger().info(f"🎯 {description}...")
        self.publish_status(description)
        
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        
        # Create bounding volume
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.02, 0.02, 0.02]  # 2cm tolerance
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.planning_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.3
        orientation_constraint.absolute_y_axis_tolerance = 0.3
        orientation_constraint.absolute_z_axis_tolerance = 0.3
        orientation_constraint.weight = 1.0
        
        # Combine constraints
        pose_constraint = Constraints()
        pose_constraint.position_constraints.append(position_constraint)
        pose_constraint.orientation_constraints.append(orientation_constraint)
        
        goal.request.goal_constraints.append(pose_constraint)
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.move_goal_response_callback)

    def move_goal_response_callback(self, future):
        """Handle move goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by MoveGroup")
            self.state = PickPlaceState.IDLE
            return
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_callback)

    def move_result_callback(self, future):
        """Handle move result and continue sequence"""
        result = future.result().result
        success = result.error_code.val == result.error_code.SUCCESS
        
        if success:
            self.get_logger().info("✅ Movement completed successfully")
            self.handle_movement_success()
        else:
            self.get_logger().error(f"❌ Movement failed: {result.error_code.val}")
            self.state = PickPlaceState.IDLE
            self.publish_status("MOVEMENT_FAILED")

    def handle_movement_success(self):
        """Handle successful movement and continue sequence"""
        if self.state == PickPlaceState.MOVING_TO_APPROACH:
            # Move to pick position
            pick_pose = Pose()
            pick_pose.position.x = self.object_pose.pose.position.x
            pick_pose.position.y = self.object_pose.pose.position.y
            pick_pose.position.z = self.object_pose.pose.position.z + 0.02  # Just above object
            
            q = quaternion_from_euler(0, np.pi, 0)
            pick_pose.orientation.x = q[0]
            pick_pose.orientation.y = q[1]
            pick_pose.orientation.z = q[2]
            pick_pose.orientation.w = q[3]
            
            self.state = PickPlaceState.MOVING_TO_PICK
            self.move_to_pose(pick_pose, "Moving to pick position")
            
        elif self.state == PickPlaceState.MOVING_TO_PICK:
            # Activate electromagnet
            self.state = PickPlaceState.PICKING
            self.activate_electromagnet(True)
            
            # Wait then lift
            self.create_timer(1.0, self.lift_object_timer)
            
        elif self.state == PickPlaceState.LIFTING:
            # Move to place
            self.state = PickPlaceState.MOVING_TO_PLACE
            self.move_to_pose(self.place_pose, "Moving to place position")
            
        elif self.state == PickPlaceState.MOVING_TO_PLACE:
            # Place object
            self.state = PickPlaceState.PLACING
            place_down_pose = Pose()
            place_down_pose.position.x = self.place_pose.position.x
            place_down_pose.position.y = self.place_pose.position.y
            place_down_pose.position.z = 0.05  # Lower to table
            place_down_pose.orientation = self.place_pose.orientation
            
            self.move_to_pose(place_down_pose, "Placing object")
            
        elif self.state == PickPlaceState.PLACING:
            # Deactivate electromagnet
            self.activate_electromagnet(False)
            
            # Wait then retreat
            self.create_timer(1.0, self.retreat_timer)
            
        elif self.state == PickPlaceState.RETREATING:
            # Return home
            self.move_to_home()
            
        elif self.state == PickPlaceState.RETURNING_HOME:
            # Task complete
            self.state = PickPlaceState.IDLE
            self.object_pose = None  # Reset for next detection
            self.get_logger().info("🎉 Pick and place sequence completed!")
            self.publish_status("TASK_COMPLETED")

    def lift_object_timer(self):
        """Timer callback to lift object"""
        lift_pose = Pose()
        lift_pose.position.x = self.object_pose.pose.position.x
        lift_pose.position.y = self.object_pose.pose.position.y
        lift_pose.position.z = self.object_pose.pose.position.z + 0.2  # Lift 20cm
        
        q = quaternion_from_euler(0, np.pi, 0)
        lift_pose.orientation.x = q[0]
        lift_pose.orientation.y = q[1]
        lift_pose.orientation.z = q[2]
        lift_pose.orientation.w = q[3]
        
        self.state = PickPlaceState.LIFTING
        self.move_to_pose(lift_pose, "Lifting object")

    def retreat_timer(self):
        """Timer callback to retreat after placing"""
        retreat_pose = Pose()
        retreat_pose.position.x = self.place_pose.position.x
        retreat_pose.position.y = self.place_pose.position.y
        retreat_pose.position.z = self.place_pose.position.z + 0.1  # Retreat up
        retreat_pose.orientation = self.place_pose.orientation
        
        self.state = PickPlaceState.RETREATING
        self.move_to_pose(retreat_pose, "Retreating")

    def activate_electromagnet(self, activate):
        """Control electromagnet"""
        msg = Bool()
        msg.data = activate
        self.electromagnet_pub.publish(msg)
        
        action = "activated" if activate else "deactivated"
        self.get_logger().info(f"🧲 Electromagnet {action}")

    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = EnhancedPickPlaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()