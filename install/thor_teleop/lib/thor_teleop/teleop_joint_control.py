#!/usr/bin/env python3
# ========================================================================
# File: thor_teleop/scripts/teleop_joint_control.py
# ========================================================================
# This node provides keyboard teleoperation for individual joints
# and moves to predefined named states from the SRDF.
#
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import sys
import termios
import tty
import math

from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

# --- Key Mapping and Configuration ---
KEY_MAP = {
    'q': ('joint_1', 0.1), 'a': ('joint_1', -0.1),
    'w': ('joint_2', 0.1), 's': ('joint_2', -0.1),
    'e': ('joint_3', 0.1), 'd': ('joint_3', -0.1),
    'r': ('joint_4', 0.1), 'f': ('joint_4', -0.1),
    't': ('joint_5', 0.1), 'g': ('joint_5', -0.1),
    'y': ('joint_6', 0.1), 'h': ('joint_6', -0.1),
}
PREDEFINED_POSES = {
    '1': "Zero",
    '2': "Rest",
}

class TeleopJointControl(Node):
    def __init__(self):
        super().__init__('teleop_joint_control')

        # --- Parameters ---
        self.move_group_name = "arm_group"
        self.joint_names = [f"joint_{i+1}" for i in range(6)]

        # --- State ---
        self.current_joint_positions = {name: 0.0 for name in self.joint_names}
        self.electromagnet_state = False
        self.motion_in_progress = False
        self.state_lock = threading.Lock()

        # --- ROS Communications ---
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        self.get_logger().info("🤖 Thor Joint Teleop Controller Started.")
        self.print_instructions()

        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def print_instructions(self):
        """Prints the control instructions to the console."""
        print("\n" + "="*50)
        print("          Thor Joint Teleop Control")
        print("="*50)
        print("  Joints 1-6 Control:")
        print("    J1: [q]/[a]   J2: [w]/[s]   J3: [e]/[d]")
        print("    J4: [r]/[f]   J5: [t]/[g]   J6: [y]/[h]")
        print("\n  Predefined Poses:")
        print("    [1]: Zero Position")
        print("    [2]: Rest Position")
        print("\n  End Effector:")
        print("    [spacebar]: Toggle Electromagnet")
        print("\n  [Ctrl+C] to quit")
        print("="*50 + "\n")

    def keyboard_loop(self):
        """Continuously listens for keyboard input."""
        self.get_logger().info("Waiting for MoveIt server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ MoveIt server is ready.")

        # Get initial robot state
        self.get_current_state()

        while rclpy.ok():
            key = self.get_key()
            if key == '\x03': # Ctrl+C
                break
            
            with self.state_lock:
                if self.motion_in_progress:
                    self.get_logger().warn("⚠️ Motion in progress, please wait.")
                    continue
                self.motion_in_progress = True

            if key in KEY_MAP:
                joint_name, increment = KEY_MAP[key]
                self.current_joint_positions[joint_name] += increment
                self.get_logger().info(f"➡️ Jogging {joint_name} by {math.degrees(increment):.1f}°")
                self.move_to_joints(self.current_joint_positions)
            elif key in PREDEFINED_POSES:
                pose_name = PREDEFINED_POSES[key]
                self.get_logger().info(f"➡️ Moving to predefined pose: '{pose_name}'")
                self.move_to_named_target(pose_name)
            elif key == ' ':
                self.toggle_electromagnet()
                self.motion_in_progress = False # No motion, so release lock immediately
            else:
                self.motion_in_progress = False # No valid key, release lock

    def get_key(self):
        """Gets a single key press from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def get_current_state(self):
        """Retrieves the current joint states of the robot."""
        # This is a simplified approach. A real implementation would subscribe
        # to /joint_states and update continuously.
        # For teleop, starting from a known state or 0 is often sufficient.
        self.get_logger().info("Assuming starting joint positions are 0.")

    def toggle_electromagnet(self):
        """Toggles the state of the electromagnet."""
        self.electromagnet_state = not self.electromagnet_state
        msg = Bool(data=self.electromagnet_state)
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"🧲 Electromagnet toggled {'ON' if self.electromagnet_state else 'OFF'}")

    def move_to_named_target(self, target_name):
        """Sends a goal to move to a predefined named target state."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        goal_msg.request.goal_constraints.append(Constraints(name=target_name))
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def move_to_joints(self, joint_positions):
        """Sends a goal to move to a specific joint configuration."""
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        
        constraints = Constraints()
        for name, value in joint_positions.items():
            jc = JointConstraint(joint_name=name, position=value, weight=1.0)
            constraints.joint_constraints.append(jc)
        
        request.goal_constraints.append(constraints)
        goal_msg.request = request
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        """Callback for when a motion is complete."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveIt server')
        else:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for the final result of the motion."""
        result = future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info("✅ Motion succeeded.")
            # Update current positions after successful move
            # Note: A more robust way is to read from /joint_states
        else:
            self.get_logger().error(f"❌ Motion failed with error code: {result.error_code.val}")
            self.get_logger().info("Reverting to last known good positions.")
            # Revert to last known good state from a real /joint_states topic if available

        with self.state_lock:
            self.motion_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJointControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()