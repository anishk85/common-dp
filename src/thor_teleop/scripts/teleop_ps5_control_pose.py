#!/usr/bin/env python3
# ========================================================================
# File: thor_teleop/scripts/teleop_pose_control_ps5.py
# ========================================================================
# This node provides PS5 controller teleoperation for the end-effector's
# Cartesian pose (position and orientation).
#
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import math
import time

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

# --- PS5 Controller Button/Axis Mapping ---
# Based on standard PS5 controller mapping in ROS2
PS5_BUTTONS = {
    'CROSS': 0,        # X button
    'CIRCLE': 1,       # O button  
    'SQUARE': 2,       # Square button
    'TRIANGLE': 3,     # Triangle button
    'L1': 4,           # L1 shoulder button
    'R1': 5,           # R1 shoulder button
    'L2': 6,           # L2 trigger button
    'R2': 7,           # R2 trigger button
    'SHARE': 8,        # Share button
    'OPTIONS': 9,      # Options button
    'PS': 10,          # PS button
    'L3': 11,          # Left stick click
    'R3': 12           # Right stick click
}

PS5_AXES = {
    'LEFT_STICK_X': 0,     # Left stick horizontal
    'LEFT_STICK_Y': 1,     # Left stick vertical  
    'RIGHT_STICK_X': 2,    # Right stick horizontal
    'RIGHT_STICK_Y': 3,    # Right stick vertical
    'L2_TRIGGER': 4,       # L2 analog trigger
    'R2_TRIGGER': 5,       # R2 analog trigger
    'DPAD_X': 6,           # D-pad horizontal
    'DPAD_Y': 7            # D-pad vertical
}

class TeleopPoseControlPS5(Node):
    def __init__(self):
        super().__init__('teleop_pose_control_ps5')

        # --- Parameters ---
        self.move_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.base_frame = "world"
        
        # Control parameters
        self.position_scale = 0.01    # m per joystick unit
        self.orientation_scale = 0.05 # rad per joystick unit
        self.deadzone = 0.1          # Joystick deadzone
        self.control_rate = 10.0     # Hz for continuous control

        # --- State ---
        self.current_pose = None
        self.electromagnet_state = False
        self.motion_in_progress = False
        self.state_lock = threading.Lock()
        self.last_joy_msg = None
        self.last_button_states = {}
        
        # Control mode flags
        self.position_mode = True    # True = position control, False = orientation control

        # --- ROS Communications ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        # Joy subscriber
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Control timer for continuous movement
        self.control_timer = self.create_timer(1.0/self.control_rate, self.control_update)
        
        self.get_logger().info("🎮 Thor PS5 Controller Pose Teleop Started.")
        self.print_instructions()

        # Initialize MoveIt connection
        self.init_moveit()

    def print_instructions(self):
        """Prints the PS5 controller instructions."""
        print("\n" + "="*60)
        print("      Thor PS5 Controller Pose Teleop Control")
        print("="*60)
        print("  🎮 CONTROL MODES:")
        print("    L1: Position Control Mode (XYZ translation)")
        print("    R1: Orientation Control Mode (Roll/Pitch/Yaw)")
        print("\n  📍 POSITION CONTROL (Hold L1):")
        print("    Left Stick:  X (left/right), Y (forward/back)")
        print("    Right Stick: Z (up/down)")
        print("\n  🔄 ORIENTATION CONTROL (Hold R1):")
        print("    Left Stick:  Roll, Pitch")
        print("    Right Stick: Yaw")
        print("\n  🧲 END EFFECTOR:")
        print("    Cross (X):   Toggle Electromagnet")
        print("\n  ⚙️  SETTINGS:")
        print("    Triangle:    Get Current Pose")
        print("    Square:      Emergency Stop")
        print("\n  Status indicators in terminal...")
        print("="*60 + "\n")

    def init_moveit(self):
        """Initialize MoveIt connection and get initial pose."""
        self.get_logger().info("Waiting for MoveIt server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ MoveIt server is ready.")
        
        # Get initial robot pose
        self.get_current_pose()

    def joy_callback(self, msg):
        """Callback for joystick messages."""
        self.last_joy_msg = msg
        
        # Handle button presses (detect rising edge)
        self.handle_button_presses(msg)

    def handle_button_presses(self, joy_msg):
        """Handle discrete button press events."""
        for button_name, button_idx in PS5_BUTTONS.items():
            if button_idx < len(joy_msg.buttons):
                current_state = joy_msg.buttons[button_idx]
                last_state = self.last_button_states.get(button_name, 0)
                
                # Rising edge detection (button pressed)
                if current_state and not last_state:
                    if button_name == 'CROSS':
                        self.toggle_electromagnet()
                    elif button_name == 'TRIANGLE':
                        self.get_current_pose()
                    elif button_name == 'SQUARE':
                        self.emergency_stop()
                
                self.last_button_states[button_name] = current_state

    def control_update(self):
        """Main control loop called at regular intervals."""
        if self.last_joy_msg is None or self.current_pose is None:
            return
            
        with self.state_lock:
            if self.motion_in_progress:
                return
                
        joy = self.last_joy_msg
        
        # Check control mode buttons
        l1_pressed = joy.buttons[PS5_BUTTONS['L1']] if PS5_BUTTONS['L1'] < len(joy.buttons) else False
        r1_pressed = joy.buttons[PS5_BUTTONS['R1']] if PS5_BUTTONS['R1'] < len(joy.buttons) else False
        
        # Get joystick values with deadzone
        left_x = self.apply_deadzone(joy.axes[PS5_AXES['LEFT_STICK_X']])
        left_y = self.apply_deadzone(joy.axes[PS5_AXES['LEFT_STICK_Y']])
        right_x = self.apply_deadzone(joy.axes[PS5_AXES['RIGHT_STICK_X']])
        right_y = self.apply_deadzone(joy.axes[PS5_AXES['RIGHT_STICK_Y']])
        
        # Check if any control input is active
        any_input = abs(left_x) > 0 or abs(left_y) > 0 or abs(right_x) > 0 or abs(right_y) > 0
        
        if not any_input or (not l1_pressed and not r1_pressed):
            return
            
        # Create a copy of current pose for modification
        target_pose = PoseStamped()
        target_pose.header = self.current_pose.header
        target_pose.pose.position.x = self.current_pose.pose.position.x
        target_pose.pose.position.y = self.current_pose.pose.position.y
        target_pose.pose.position.z = self.current_pose.pose.position.z
        target_pose.pose.orientation = self.current_pose.pose.orientation
        
        pose_changed = False
        
        if l1_pressed:  # Position control mode
            if abs(left_x) > 0:
                target_pose.pose.position.y += left_x * self.position_scale
                pose_changed = True
            if abs(left_y) > 0:
                target_pose.pose.position.x += left_y * self.position_scale
                pose_changed = True
            if abs(right_y) > 0:
                target_pose.pose.position.z += right_y * self.position_scale
                pose_changed = True
                
        elif r1_pressed:  # Orientation control mode
            delta_roll = left_y * self.orientation_scale
            delta_pitch = left_x * self.orientation_scale  
            delta_yaw = right_x * self.orientation_scale
            
            if abs(delta_roll) > 0 or abs(delta_pitch) > 0 or abs(delta_yaw) > 0:
                # Create incremental quaternion
                delta_q = quaternion_from_euler(delta_roll, delta_pitch, delta_yaw)
                
                # Apply to current orientation
                current_q = [
                    target_pose.pose.orientation.x,
                    target_pose.pose.orientation.y,
                    target_pose.pose.orientation.z,
                    target_pose.pose.orientation.w
                ]
                new_q = quaternion_multiply(current_q, delta_q)
                
                target_pose.pose.orientation.x = new_q[0]
                target_pose.pose.orientation.y = new_q[1]
                target_pose.pose.orientation.z = new_q[2]
                target_pose.pose.orientation.w = new_q[3]
                pose_changed = True
        
        if pose_changed:
            with self.state_lock:
                self.motion_in_progress = True
            
            mode = "POSITION" if l1_pressed else "ORIENTATION"
            p = target_pose.pose.position
            self.get_logger().info(f"🎮 {mode} → P({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
            
            self.move_to_pose(target_pose)

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range to full range
        if value > 0:
            return (value - self.deadzone) / (1.0 - self.deadzone)
        else:
            return (value + self.deadzone) / (1.0 - self.deadzone)

    def get_current_pose(self):
        """Retrieves the current end-effector pose from TF2."""
        self.get_logger().info("📍 Getting current end-effector pose...")
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_link, rclpy.time.Time()
            )
            
            pose = Pose()
            pose.position.x = transform_stamped.transform.translation.x
            pose.position.y = transform_stamped.transform.translation.y
            pose.position.z = transform_stamped.transform.translation.z
            pose.orientation = transform_stamped.transform.rotation
            
            self.current_pose = PoseStamped(header=transform_stamped.header, pose=pose)
            
            p = self.current_pose.pose.position
            self.get_logger().info(f"✅ Pose: P({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not get transform: {e}")

    def toggle_electromagnet(self):
        """Toggles the electromagnet state."""
        self.electromagnet_state = not self.electromagnet_state
        msg = Bool(data=self.electromagnet_state)
        self.electromagnet_pub.publish(msg)
        state_str = "ON" if self.electromagnet_state else "OFF"
        self.get_logger().info(f"🧲 Electromagnet → {state_str}")

    def emergency_stop(self):
        """Emergency stop - cancel current motion."""
        self.get_logger().warn("🛑 EMERGENCY STOP - Cancelling current motion")
        # Cancel any pending goals
        if hasattr(self.move_group_client, '_goal_handles'):
            for goal_handle in self.move_group_client._goal_handles:
                goal_handle.cancel_goal_async()
        
        with self.state_lock:
            self.motion_in_progress = False

    def move_to_pose(self, pose_stamped):
        """Sends a goal to move to a target pose."""
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.num_planning_attempts = 3  # Reduced for faster response
        request.allowed_planning_time = 2.0  # Reduced planning time
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])  # 5mm tolerance
        )
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        request.goal_constraints.append(constraints)
        goal_msg.request = request
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        """Callback for when a motion goal is accepted/rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveIt')
            with self.state_lock:
                self.motion_in_progress = False
        else:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for the final result of the motion."""
        result = future.result().result
        
        # Always re-acquire pose after motion attempt
        self.get_current_pose()
        
        with self.state_lock:
            self.motion_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPoseControlPS5()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down PS5 teleop controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()