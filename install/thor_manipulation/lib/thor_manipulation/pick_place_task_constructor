#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
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

class TaskState(Enum):
    IDLE = auto()
    PLANNING = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAILED = auto()

class SimplifiedTaskConstructor(Node):
    def __init__(self):
        super().__init__('simplified_task_constructor')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/task_command', self.execute_task_callback, 10)
        self.object_detection_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_detected_callback, 10)
        
        # Task parameters
        self.arm_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.planning_frame = "base_link"
        
        # Object properties
        self.object_pose = None
        self.current_state = TaskState.IDLE
        
        # Predefined poses
        self.home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Adjust for your robot
        self.ready_joints = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]  # Adjust for your robot
        
        # Place location
        self.place_position = Point(x=-0.2, y=0.3, z=0.1)
        
        self.get_logger().info("🤖 Simplified Task Constructor initialized")
        self.get_logger().info("Available commands: 'pick_place', 'home', 'ready'")
        
        # Wait for MoveGroup action server
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ Connected to MoveGroup action server")

    def object_detected_callback(self, msg):
        """Store detected object pose"""
        self.object_pose = msg.pose
        self.get_logger().info(f"📦 Object detected at: x={msg.pose.position.x:.3f}, "
                              f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")

    def execute_task_callback(self, msg):
        """Execute task based on command"""
        command = msg.data.lower()
        
        if self.current_state != TaskState.IDLE:
            self.get_logger().warn(f"⚠️ Task in progress (state: {self.current_state.name})")
            return
        
        if command == "pick_place":
            if self.object_pose is None:
                self.get_logger().warn("⚠️ No object detected. Trigger detection first.")
                return
            self.execute_pick_place_sequence()
        elif command == "home":
            self.move_to_joint_goal(self.home_joints, "Moving to home position")
        elif command == "ready":
            self.move_to_joint_goal(self.ready_joints, "Moving to ready position")
        else:
            self.get_logger().warn(f"❌ Unknown command: {command}")

    def execute_pick_place_sequence(self):
        """Execute complete pick and place sequence"""
        self.get_logger().info("🚀 Starting pick and place sequence...")
        self.current_state = TaskState.PLANNING
        self.publish_status("Starting pick and place sequence")
        
        # Step 1: Move to ready position
        self.move_to_joint_goal(self.ready_joints, "Moving to ready position", 
                               callback=self.on_ready_complete)

    def on_ready_complete(self, success):
        """Callback after moving to ready position"""
        if not success:
            self.handle_failure("Failed to reach ready position")
            return
        
        # Step 2: Move to approach pose
        approach_pose = self.create_approach_pose()
        self.move_to_pose_goal(approach_pose, "Moving to approach pose", 
                              callback=self.on_approach_complete)

    def on_approach_complete(self, success):
        """Callback after approaching object"""
        if not success:
            self.handle_failure("Failed to reach approach pose")
            return
        
        # Step 3: Move to grasp pose
        grasp_pose = self.create_grasp_pose()
        self.move_to_pose_goal(grasp_pose, "Moving to grasp pose", 
                              callback=self.on_grasp_complete)

    def on_grasp_complete(self, success):
        """Callback after reaching grasp pose"""
        if not success:
            self.handle_failure("Failed to reach grasp pose")
            return
        
        # Step 4: Activate electromagnet
        self.get_logger().info("🧲 Activating electromagnet...")
        self.activate_electromagnet(True)
        
        # Wait a moment for electromagnet to engage
        self.create_timer(1.0, self.lift_object_timer_callback)

    def lift_object_timer_callback(self):
        """Timer callback to lift object after electromagnet activation"""
        # Step 5: Lift object
        lift_pose = self.create_lift_pose()
        self.move_to_pose_goal(lift_pose, "Lifting object", 
                              callback=self.on_lift_complete)

    def on_lift_complete(self, success):
        """Callback after lifting object"""
        if not success:
            self.handle_failure("Failed to lift object")
            return
        
        # Step 6: Move to place position
        place_pose = self.create_place_pose()
        self.move_to_pose_goal(place_pose, "Moving to place position", 
                              callback=self.on_place_move_complete)

    def on_place_move_complete(self, success):
        """Callback after moving to place position"""
        if not success:
            self.handle_failure("Failed to reach place position")
            return
        
        # Step 7: Lower to place pose
        lower_pose = self.create_lower_pose()
        self.move_to_pose_goal(lower_pose, "Lowering object", 
                              callback=self.on_lower_complete)

    def on_lower_complete(self, success):
        """Callback after lowering object"""
        if not success:
            self.handle_failure("Failed to lower object")
            return
        
        # Step 8: Deactivate electromagnet
        self.get_logger().info("🔓 Deactivating electromagnet...")
        self.activate_electromagnet(False)
        
        # Wait a moment then retreat
        self.create_timer(1.0, self.retreat_timer_callback)

    def retreat_timer_callback(self):
        """Timer callback to retreat after releasing object"""
        # Step 9: Retreat upward
        retreat_pose = self.create_retreat_pose()
        self.move_to_pose_goal(retreat_pose, "Retreating", 
                              callback=self.on_retreat_complete)

    def on_retreat_complete(self, success):
        """Callback after retreating"""
        if not success:
            self.get_logger().warn("⚠️ Retreat failed, but task mostly complete")
        
        # Step 10: Return home
        self.move_to_joint_goal(self.home_joints, "Returning home", 
                               callback=self.on_task_complete)

    def on_task_complete(self, success):
        """Final callback when task is complete"""
        if success:
            self.get_logger().info("🎉 Pick and place task completed successfully!")
            self.publish_status("Task completed successfully")
            self.current_state = TaskState.COMPLETED
        else:
            self.get_logger().warn("⚠️ Task completed with issues during return home")
            self.publish_status("Task completed with minor issues")
            self.current_state = TaskState.COMPLETED
        
        # Reset state after a delay
        self.create_timer(3.0, self.reset_state_timer_callback)

    def reset_state_timer_callback(self):
        """Reset state to idle after task completion"""
        self.current_state = TaskState.IDLE
        self.get_logger().info("Ready for next task")

    def handle_failure(self, error_msg):
        """Handle task failure"""
        self.get_logger().error(f"❌ {error_msg}")
        self.publish_status(f"Task failed: {error_msg}")
        self.current_state = TaskState.FAILED
        
        # Try to return home safely
        self.move_to_joint_goal(self.home_joints, "Emergency return home")
        
        # Reset state after delay
        self.create_timer(5.0, self.reset_state_timer_callback)

    def move_to_joint_goal(self, joint_values, description, callback=None):
        """Move to joint goal"""
        self.get_logger().info(f"🎯 {description}...")
        self.publish_status(description)
        
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Set joint goal
        joint_constraint = Constraints()
        # Add joint constraints here (simplified for brevity)
        goal.request.goal_constraints.append(joint_constraint)
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.move_goal_response_callback(f, callback))

    def move_to_pose_goal(self, target_pose, description, callback=None):
        """Move to pose goal"""
        self.get_logger().info(f"🎯 {description}...")
        self.publish_status(description)
        
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # Create position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.constraint_region.primitive_poses.append(target_pose)
        
        # Create bounding volume (small box around target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.weight = 1.0
        
        # Create orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.planning_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0
        
        # Combine constraints
        pose_constraint = Constraints()
        pose_constraint.position_constraints.append(position_constraint)
        pose_constraint.orientation_constraints.append(orientation_constraint)
        
        goal.request.goal_constraints.append(pose_constraint)
        
        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.move_goal_response_callback(f, callback))

    def move_goal_response_callback(self, future, callback=None):
        """Handle move goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by MoveGroup")
            if callback:
                callback(False)
            return
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.move_result_callback(f, callback))

    def move_result_callback(self, future, callback=None):
        """Handle move result"""
        result = future.result().result
        success = result.error_code.val == result.error_code.SUCCESS
        
        if success:
            self.get_logger().info("✅ Movement completed successfully")
        else:
            self.get_logger().error(f"❌ Movement failed: {result.error_code.val}")
        
        if callback:
            callback(success)

    def create_approach_pose(self):
        """Create approach pose above object"""
        pose = Pose()
        pose.position.x = self.object_pose.position.x
        pose.position.y = self.object_pose.position.y
        pose.position.z = self.object_pose.position.z + 0.15  # 15cm above
        
        # Point downward
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def create_grasp_pose(self):
        """Create grasp pose at object"""
        pose = Pose()
        pose.position.x = self.object_pose.position.x
        pose.position.y = self.object_pose.position.y
        pose.position.z = self.object_pose.position.z + 0.02  # Just above object
        
        # Point downward
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def create_lift_pose(self):
        """Create lift pose"""
        pose = Pose()
        pose.position.x = self.object_pose.position.x
        pose.position.y = self.object_pose.position.y
        pose.position.z = self.object_pose.position.z + 0.2  # Lift 20cm
        
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def create_place_pose(self):
        """Create place pose"""
        pose = Pose()
        pose.position = self.place_position
        pose.position.z += 0.1  # Above place location
        
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def create_lower_pose(self):
        """Create lower pose for placing"""
        pose = Pose()
        pose.position = self.place_position
        pose.position.z += 0.02  # Just above place surface
        
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def create_retreat_pose(self):
        """Create retreat pose after placing"""
        pose = Pose()
        pose.position = self.place_position
        pose.position.z += 0.15  # Retreat upward
        
        q = quaternion_from_euler(0, np.pi, 0)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def activate_electromagnet(self, activate):
        """Control electromagnet"""
        msg = Bool()
        msg.data = activate
        self.electromagnet_pub.publish(msg)
        action = "activated" if activate else "deactivated"
        self.get_logger().info(f"🧲 Electromagnet {action}")

    def publish_status(self, status):
        """Publish task status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = SimplifiedTaskConstructor()
    
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