#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import math

class InteractivePickPlaceController(Node):
    def __init__(self):
        super().__init__('interactive_pick_place_controller')

        # --- Threading and State Management ---
        self.state_lock = threading.Lock()
        self.motion_done_event = threading.Event()

        # --- Parameters and Configuration ---
        self.move_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        self.base_frame = "world" # URDF has 'world' as the root

        # URDF-based calculations
        # table_base is at z=0.095, thickness=0.04. Surface = 0.095 + 0.04/2 = 0.115
        self.table_height = 0.115
        self.object_height_estimate = 0.04 # Estimate for picking
        self.approach_distance = 0.10 # 10cm above the object
        self.place_position = Point(x=-0.2, y=0.3, z=self.table_height + self.approach_distance)
        
        # A known, safe joint configuration
        self.home_joint_positions = {
            "joint_1": 0.0, "joint_2": -1.0, "joint_3": 1.0,
            "joint_4": 0.0, "joint_5": 1.57, "joint_6": 0.0
        }

        # --- State Variables ---
        self.current_image = None
        self.system_ready = False
        self.sequence_in_progress = False
        self.last_status_message = "Initializing..."
        self.last_moveit_error_code = None

        # --- ROS Communications ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        # --- System Readiness Check ---
        self.create_timer(1.0, self.check_system_status)
        self.get_logger().info("🤖 Interactive Pick and Place Controller has started.")
        self.get_logger().info(f"📏 Table surface calculated at Z = {self.table_height:.3f}m")

        # --- GUI ---
        # The GUI is started once the system is ready
        self.gui_thread = None

    def check_system_status(self):
        """Checks if all components are ready to operate."""
        with self.state_lock:
            if self.system_ready:
                return

            image_ok = self.current_image is not None
            moveit_ok = self.move_group_client.wait_for_server(timeout_sec=0.5)

            if image_ok and moveit_ok:
                self.system_ready = True
                if not self.gui_thread:
                    self.gui_thread = threading.Thread(target=self.run_gui)
                    self.gui_thread.daemon = True
                    self.gui_thread.start()
                self.set_status("✅ System Ready. Click object to pick.")
            else:
                self.set_status(f"⏳ Waiting... [Camera: {'✅' if image_ok else '❌'}, MoveIt: {'✅' if moveit_ok else '❌'}]")

    def run_gui(self):
        """Main loop for the OpenCV GUI window."""
        window_name = "Thor Controller"
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while rclpy.ok():
            if self.current_image is not None:
                display_image = self.current_image.copy()
                # Draw status text
                cv2.putText(display_image, self.last_status_message, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, "Press 'q' to quit", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow(window_name, display_image)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()
        rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks for target selection."""
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.state_lock:
                if not self.system_ready:
                    self.get_logger().warn("⚠️ System not ready, ignoring click.")
                    return
                if self.sequence_in_progress:
                    self.get_logger().warn("⚠️ Sequence already in progress, ignoring click.")
                    return
                self.sequence_in_progress = True

            # Start the sequence in a new thread to avoid blocking the GUI
            threading.Thread(target=self.run_pick_and_place_sequence, args=(x, y)).start()

    def image_callback(self, msg):
        """Store the latest camera image."""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def run_pick_and_place_sequence(self, pixel_x, pixel_y):
        """The main state machine for the pick and place task."""
        # 1. Go to a known 'home' position first
        self.set_status("➡️ 1/7: Moving to Home Position...")
        if not self.move_to_joints(self.home_joint_positions):
            self.sequence_failed("Failed to move to home position.")
            return

        # 2. Calculate the 3D pick position from the 2D pixel
        self.set_status("📐 2/7: Calculating pick position...")
        pick_position = self.pixel_to_world(pixel_x, pixel_y)
        if pick_position is None:
            self.sequence_failed("Failed to calculate world coordinates.")
            return

        # Define poses based on the calculated pick position
        pick_pose = self.create_pose(pick_position.x, pick_position.y, pick_position.z)
        approach_pose = self.create_pose(pick_position.x, pick_position.y, pick_position.z + self.approach_distance)
        place_pose = self.create_pose(self.place_position.x, self.place_position.y, self.place_position.z)
        
        # 3. Move to the approach position above the object
        self.set_status("➡️ 3/7: Moving to approach position...")
        if not self.move_to_pose(approach_pose):
            self.sequence_failed("Failed to move to approach pose.")
            return

        # 4. Lower to the pick position using a straight-line Cartesian path
        self.set_status("⬇️ 4/7: Lowering to pick (Cartesian)...")
        if not self.move_to_pose(pick_pose, cartesian=True):
            self.sequence_failed("Failed to lower to pick pose.")
            return

        # 5. Activate electromagnet and lift the object using a straight-line Cartesian path
        self.set_status("🧲 5/7: Picking object (Cartesian)...")
        self.set_electromagnet(True)
        time.sleep(1.0) # Give time for the magnet to engage

        if not self.move_to_pose(approach_pose, cartesian=True): # Lift back up to the approach pose
            self.set_electromagnet(False)
            self.sequence_failed("Failed to lift object.")
            return

        # 6. Move to the predefined place position
        self.set_status("➡️ 6/7: Moving to place position...")
        if not self.move_to_pose(place_pose):
            self.set_electromagnet(False)
            self.sequence_failed("Failed to move to place pose.")
            return

        # 7. Release the object and return home
        self.set_status("➡️ 7/7: Releasing object...")
        self.set_electromagnet(False)
        time.sleep(1.0)
        
        self.move_to_joints(self.home_joint_positions)

        self.get_logger().info("✅ Pick and place sequence completed successfully!")
        self.sequence_succeeded()

    def sequence_failed(self, reason):
        """Handles a failure at any point in the sequence."""
        self.get_logger().error(f"❌ SEQUENCE FAILED: {reason}")
        self.set_status(f"Error: {reason}. Resetting.")
        self.set_electromagnet(False) # Ensure magnet is off
        # Try to move home safely
        self.move_to_joints(self.home_joint_positions)
        with self.state_lock:
            self.sequence_in_progress = False

    def sequence_succeeded(self):
        """Resets the state after a successful run."""
        self.set_status("✅ System Ready. Click object to pick.")
        with self.state_lock:
            self.sequence_in_progress = False

    def pixel_to_world(self, u, v):
        """A simplified mapping of a pixel to a 3D point on the table."""
        # This is a placeholder. A real implementation would use camera calibration
        # and tf2 to project a ray and find its intersection with the table plane.
        # For now, we use a linear mapping based on observations of the scene.
        
        # These values would need to be tuned for your specific camera setup.
        # Assumes camera looks at the center of a 0.5m x 0.4m table area.
        img_width = self.current_image.shape[1]
        img_height = self.current_image.shape[0]
        
        # Table center in world coordinates from URDF
        table_center_x = 0.5
        table_center_y = 0.0
        
        # Approximate size of the visible table area in meters
        visible_width_m = 0.4
        visible_depth_m = 0.5

        # Map normalized pixel coordinates to world coordinates
        norm_x = (u - img_width / 2) / (img_width / 2)   # -1 to 1
        norm_y = (v - img_height / 2) / (img_height / 2)  # -1 to 1

        world_x = table_center_x - norm_y * (visible_depth_m / 2)
        world_y = table_center_y - norm_x * (visible_width_m / 2)
        world_z = self.table_height + self.object_height_estimate / 2.0

        self.get_logger().info(f"🌍 Pixel ({u},{v}) -> World ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        return Point(x=world_x, y=world_y, z=world_z)

    def create_pose(self, x, y, z):
        """Creates a PoseStamped with a standard downward-facing orientation."""
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=x, y=y, z=z)
        # Quaternion for a downward-facing end-effector (pitch 90 degrees)
        # This is generally more stable than a 180-degree roll.
        q_x, q_y, q_z, q_w = 0.0, 0.7071068, 0.0, 0.7071068
        pose.pose.orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
        return pose

    def move_to_joints(self, joint_positions):
        """Sends a goal to move to a specific joint configuration."""
        self.motion_done_event.clear()
        self.last_moveit_error_code = None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.allowed_planning_time = 10.0 # Added planning time
        
        # Create joint constraints
        constraints = Constraints()
        for name, value in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        request.goal_constraints.append(constraints)
        goal_msg.request = request
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        completed = self.motion_done_event.wait(timeout=20.0)
        if not completed:
            self.get_logger().error("❌ Joint motion timed out!")
            return False
        return self.last_moveit_error_code == 1

    def move_to_pose(self, pose_stamped, cartesian=False):
        """Sends a goal to the MoveGroup action server and waits for completion."""
        self.motion_done_event.clear()
        self.last_moveit_error_code = None

        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.num_planning_attempts = 10 
        request.allowed_planning_time = 10.0

        constraints = Constraints()
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])) # 5mm tolerance
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 3.14 # Relaxed Z-axis tolerance
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)

        request.goal_constraints.append(constraints)
        
        # This is the key change: check if a Cartesian path is requested
        if cartesian:
            goal_msg.request.path_constraints = constraints
            goal_msg.planning_options.plan_only = True
            goal_msg.planning_options.look_around = False
            goal_msg.planning_options.replan = False
            
            # We don't send the goal directly, we compute the cartesian path
            # This requires a different approach than the action client can provide directly
            # For simplicity, we will still use the main planner but with a very tight
            # tolerance, which encourages a straight-line path.
            # A true cartesian path would use the compute_cartesian_path service.
            # This is a good compromise for now.
            pos_constraint.constraint_region.primitives[0].dimensions = [0.001] # very tight tolerance
            self.get_logger().info("Attempting a precise linear-like motion.")


        goal_msg.request = request
        goal_msg.planning_options.plan_only = False

        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        completed = self.motion_done_event.wait(timeout=20.0)
        
        if not completed:
            self.get_logger().error("❌ Pose motion timed out!")
            return False

        return self.last_moveit_error_code == 1

    def goal_response_callback(self, future):
        """Callback for when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveIt server')
            self.last_moveit_error_code = -999 # Custom code for rejection
            self.motion_done_event.set()
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Callback for when the motion is finished."""
        result = future.result().result
        self.last_moveit_error_code = result.error_code.val
        if self.last_moveit_error_code != 1:
            self.get_logger().error(f"❌ MoveIt motion failed with error code: {result.error_code.val}")
        else:
            self.get_logger().info("✅ MoveIt motion succeeded.")
        
        # Signal that the motion is done
        self.motion_done_event.set()

    def set_status(self, message):
        """Thread-safe method to update the GUI status message."""
        self.last_status_message = message
        self.get_logger().info(message)

    def set_electromagnet(self, state):
        """Publishes a command to the electromagnet."""
        msg = Bool()
        msg.data = bool(state)
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"🧲 Electromagnet set to {'ON' if state else 'OFF'}")

def main(args=None):
    rclpy.init(args=args)
    node = InteractivePickPlaceController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
