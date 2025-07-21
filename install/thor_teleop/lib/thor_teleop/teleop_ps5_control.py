#!/usr/bin/env python3
# ========================================================================
# File: thor_teleop/scripts/teleop_ps5_control.py
# ========================================================================
# This node maps PS5 controller inputs to joint movements for the thor_arm.
# It supports analog stick control, button-based jogging, named poses, and electromagnet toggling.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class PS5JointTeleop(Node):
    def __init__(self):
        super().__init__('ps5_joint_teleop')

        # --- Move Group Setup ---
        self.move_group_name = "arm_group"
        self.joint_names = [f"joint_{i+1}" for i in range(6)]
        self.current_joint_positions = {name: 0.0 for name in self.joint_names}
        self.motion_in_progress = False
        self.electromagnet_state = False

        # --- ROS Interfaces ---
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info("🎮 PS5 Joint Teleop Node Started - Waiting for MoveIt...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ MoveIt server is ready.")

    def joy_callback(self, msg):
        """Callback for processing joystick input."""
        if self.motion_in_progress:
            return

        increment = 0.1
        analog_threshold = 0.05
        updates = {}

        # --- Analog Stick Mapping ---
        if abs(msg.axes[0]) > analog_threshold:
            updates['joint_1'] = -msg.axes[0] * increment
        if abs(msg.axes[1]) > analog_threshold:
            updates['joint_2'] = msg.axes[1] * increment
        if abs(msg.axes[3]) > analog_threshold:
            updates['joint_3'] = -msg.axes[3] * increment
        if abs(msg.axes[4]) > analog_threshold:
            updates['joint_4'] = msg.axes[4] * increment

        # --- Button-Based Mapping ---
        if msg.buttons[4]:  # L1
            updates['joint_5'] = increment
        elif msg.buttons[5]:  # R1
            updates['joint_5'] = -increment

        if msg.buttons[6]:  # L2
            updates['joint_6'] = increment
        elif msg.buttons[7]:  # R2
            updates['joint_6'] = -increment

        # --- Apply Joint Updates ---
        if updates:
            for joint, delta in updates.items():
                self.current_joint_positions[joint] += delta
                self.get_logger().info(f"🔧 Jogging {joint}: {delta:.3f} rad")
            self.motion_in_progress = True
            self.move_to_joints(self.current_joint_positions)

        # --- Electromagnet Toggle ---
        if msg.buttons[0]:  # X button
            self.toggle_electromagnet()

        # --- Predefined Poses ---
        if msg.buttons[3]:  # Triangle
            self.move_to_named_target("Zero")
        elif msg.buttons[1]:  # Circle
            self.move_to_named_target("Rest")

    def move_to_named_target(self, target_name):
        """Moves to a predefined pose (from SRDF)."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.move_group_name
        goal_msg.request.goal_constraints.append(Constraints(name=target_name))
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def move_to_joints(self, joint_positions):
        """Moves to a specified joint configuration."""
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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Motion goal rejected')
            self.motion_in_progress = False
        else:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("✅ Motion succeeded.")
        else:
            self.get_logger().error(f"❌ Motion failed with code: {result.error_code.val}")
        self.motion_in_progress = False

    def toggle_electromagnet(self):
        """Toggle electromagnet ON/OFF."""
        self.electromagnet_state = not self.electromagnet_state
        msg = Bool(data=self.electromagnet_state)
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"🧲 Electromagnet {'ON' if msg.data else 'OFF'}")

def main(args=None):
    rclpy.init(args=args)
    node = PS5JointTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
