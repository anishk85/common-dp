#!/usr/bin/env python3
# ========================================================================
# File: thor_teleop/scripts/teleop_pose_control.py
# ========================================================================
# This node provides keyboard teleoperation for the end-effector's
# Cartesian pose (position and orientation).
#
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import threading
import sys
import termios
import tty
import math
import time

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

# --- Key Mapping and Configuration ---
KEY_MAP = {
    'w': ('x', 0.02), 's': ('x', -0.02),  # Forward/Backward
    'a': ('y', 0.02), 'd': ('y', -0.02),  # Left/Right
    'r': ('z', 0.02), 'f': ('z', -0.02),  # Up/Down
    'q': ('roll', 0.1), 'e': ('roll', -0.1),
    't': ('pitch', 0.1), 'g': ('pitch', -0.1),
    'y': ('yaw', 0.1), 'h': ('yaw', -0.1),
}

class TeleopPoseControl(Node):
    def __init__(self):
        super().__init__('teleop_pose_control')

        # --- Parameters ---
        self.move_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.base_frame = "world"

        # --- State ---
        self.current_pose = None
        self.electromagnet_state = False
        self.motion_in_progress = False
        self.state_lock = threading.Lock()

        # --- ROS Communications ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        self.get_logger().info("🤖 Thor Pose (Cartesian) Teleop Controller Started.")
        self.print_instructions()

        # Start keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def print_instructions(self):
        """Prints the control instructions to the console."""
        print("\n" + "="*50)
        print("        Thor Pose (Cartesian) Teleop Control")
        print("="*50)
        print("  Position (XYZ):")
        print("    Fwd/Bwd [w]/[s]   Left/Right [a]/[d]   Up/Down [r]/[f]")
        print("\n  Orientation (Roll, Pitch, Yaw):")
        print("    Roll: [q]/[e]   Pitch: [t]/[g]   Yaw: [y]/[h]")
        print("\n  End Effector:")
        print("    [spacebar]: Toggle Electromagnet")
        print("\n  [Ctrl+C] to quit")
        print("="*50 + "\n")

    def keyboard_loop(self):
        """Continuously listens for keyboard input."""
        self.get_logger().info("Waiting for MoveIt server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("✅ MoveIt server is ready.")

        # Get initial robot pose
        self.get_current_pose()

        while rclpy.ok():
            key = self.get_key()
            if key == '\x03': # Ctrl+C
                break
            
            with self.state_lock:
                if self.motion_in_progress:
                    self.get_logger().warn("⚠️ Motion in progress, please wait.")
                    continue
                if self.current_pose is None:
                    self.get_logger().warn("⚠️ Current pose not yet available.")
                    continue
                self.motion_in_progress = True

            if key in KEY_MAP:
                axis, increment = KEY_MAP[key]
                self.update_pose(axis, increment)
                self.move_to_pose(self.current_pose)
            elif key == ' ':
                self.toggle_electromagnet()
                self.motion_in_progress = False # No motion, release lock
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

    def get_current_pose(self):
        """Retrieves the current end-effector pose from TF2."""
        self.get_logger().info("Getting current end-effector pose...")
        # Clear the current pose to ensure we get a fresh one
        self.current_pose = None 
        while self.current_pose is None and rclpy.ok():
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
                
                self.get_logger().info(f"✅ Pose acquired: "
                                       f"P({self.current_pose.pose.position.x:.2f}, "
                                       f"{self.current_pose.pose.position.y:.2f}, "
                                       f"{self.current_pose.pose.position.z:.2f})")
            except Exception as e:
                self.get_logger().warn(f"Could not get transform, retrying: {e}")
                time.sleep(1.0)

    def update_pose(self, axis, increment):
        """Updates the target pose based on the key press."""
        p = self.current_pose.pose.position
        o = self.current_pose.pose.orientation
        
        if axis == 'x': p.x += increment
        elif axis == 'y': p.y += increment
        elif axis == 'z': p.z += increment
        elif axis in ['roll', 'pitch', 'yaw']:
            delta_q = {
                'roll': quaternion_from_euler(increment, 0, 0),
                'pitch': quaternion_from_euler(0, increment, 0),
                'yaw': quaternion_from_euler(0, 0, increment),
            }[axis]
            
            current_q = [o.x, o.y, o.z, o.w]
            new_q = quaternion_multiply(current_q, delta_q)
            o.x, o.y, o.z, o.w = new_q[0], new_q[1], new_q[2], new_q[3]
        
        self.get_logger().info(f"➡️ New Target: P({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")

    def toggle_electromagnet(self):
        """Toggles the state of the electromagnet."""
        self.electromagnet_state = not self.electromagnet_state
        msg = Bool(data=self.electromagnet_state)
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"🧲 Electromagnet toggled {'ON' if self.electromagnet_state else 'OFF'}")

    def move_to_pose(self, pose_stamped):
        """Sends a goal to move to a target pose."""
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # 1cm tolerance
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.2
        orient_constraint.absolute_y_axis_tolerance = 0.2
        orient_constraint.absolute_z_axis_tolerance = 0.2
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        request.goal_constraints.append(constraints)
        goal_msg.request = request
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def goal_done_callback(self, future):
        """Callback for when a motion is complete."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveIt server')
            with self.state_lock:
                self.motion_in_progress = False
        else:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for the final result of the motion."""
        result = future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info("✅ Motion succeeded.")
        else:
            self.get_logger().error(f"❌ Motion failed with error code: {result.error_code.val}")
        
        # *** FIX: Always re-acquire the pose from TF after a move attempt ***
        # This ensures the next jog starts from the robot's true position.
        self.get_logger().info("Re-acquiring current pose from TF to re-sync.")
        self.get_current_pose()

        with self.state_lock:
            self.motion_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = TeleopPoseControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
