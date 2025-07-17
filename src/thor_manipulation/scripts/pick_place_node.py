#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
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

class PickPlaceState(Enum):
    IDLE = auto()
    DETECTING = auto()
    PLANNING = auto()
    MOVING_TO_APPROACH = auto()
    MOVING_TO_PICK = auto()
    PICKING = auto()
    LIFTING = auto()
    MOVING_TO_PLACE = auto()
    PLACING = auto()
    RETREATING = auto()
    RETURNING_HOME = auto()

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        
        # State management
        self.state = PickPlaceState.IDLE
        self.object_pose = None
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/thor_arm/pick_place_status', 10)
        self.command_pub = self.create_publisher(String, '/thor_arm/pick_place_command', 10)
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        
        # Subscribers
        self.object_detection_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_detected_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/thor_arm/pick_place_command', self.command_callback, 10)
        
        # Robot configuration
        self.arm_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.planning_frame = "base_link"
        
        # Predefined poses
        self.home_pose = self.create_pose(0.2, 0.0, 0.4, 0, np.pi/2, 0)
        self.ready_pose = self.create_pose(0.3, 0.0, 0.2, 0, np.pi/2, 0)
        self.place_pose = self.create_pose(-0.2, 0.3, 0.1, 0, np.pi/2, 0)
        
        self.get_logger().info("🤖 Pick and Place Node initialized")
        self.get_logger().info("Waiting for MoveGroup action server...")
        
        # Wait for MoveGroup
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ Connected to MoveGroup action server")
        
        # Start in home position
        self.move_to_home()

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

    def command_callback(self, msg):
        """Handle incoming commands"""
        command = msg.data.upper()
        
        if command == "HOME":
            self.move_to_home()
        elif command == "DETECT":
            self.start_detection()
        elif command == "PICK" and self.object_pose is not None:
            self.start_pick_sequence()
        elif command == "PICK":
            self.get_logger().warn("⚠️ No object detected. Run DETECT first.")
        elif command == "PLACE":
            self.move_to_place()
        else:
            self.get_logger().warn(f"❌ Unknown command or invalid state: {command}")

    def object_detected_callback(self, msg):
        """Store detected object pose"""
        self.object_pose = msg.pose
        self.get_logger().info(f"📦 Object detected at: "
                              f"x={msg.pose.position.x:.3f}, "
                              f"y={msg.pose.position.y:.3f}, "
                              f"z={msg.pose.position.z:.3f}")
        
        # Publish status
        self.publish_status("OBJECT_DETECTED")

    def start_detection(self):
        """Trigger object detection"""
        self.state = PickPlaceState.DETECTING
        self.get_logger().info("🔍 Starting object detection...")
        
        # Send detection command
        detection_msg = String()
        detection_msg.data = "DETECT"
        self.command_pub.publish(detection_msg)
        
        self.publish_status("DETECTING")

    def start_pick_sequence(self):
        """Start the pick and place sequence"""
        if self.object_pose is None:
            self.get_logger().error("❌ No object pose available")
            return
            
        self.get_logger().info("🚀 Starting pick sequence...")
        self.state = PickPlaceState.MOVING_TO_APPROACH
        
        # Create approach pose (above object)
        approach_pose = Pose()
        approach_pose.position.x = self.object_pose.position.x
        approach_pose.position.y = self.object_pose.position.y
        approach_pose.position.z = self.object_pose.position.z + 0.15  # 15cm above
        
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
            pick_pose.position.x = self.object_pose.position.x
            pick_pose.position.y = self.object_pose.position.y
            pick_pose.position.z = self.object_pose.position.z + 0.02  # Just above object
            
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
            self.get_logger().info("🎉 Pick and place sequence completed!")
            self.publish_status("TASK_COMPLETED")

    def lift_object_timer(self):
        """Timer callback to lift object"""
        lift_pose = Pose()
        lift_pose.position.x = self.object_pose.position.x
        lift_pose.position.y = self.object_pose.position.y
        lift_pose.position.z = self.object_pose.position.z + 0.2  # Lift 20cm
        
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
    
    node = PickPlaceNode()
    
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