#!/usr/bin/env python3
# ========================================================================
# File: thor_learning/scripts/data_recorder.py
# ========================================================================
# This script is the core of the data collection for LeRobot.
# It provides joystick teleoperation for the Thor arm's end-effector,
# subscribes to multiple camera feeds and robot state topics,
# and saves synchronized "episodes" into an HDF5 dataset file.
#
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import h5py
import numpy as np
from datetime import datetime

# ROS Messages
from sensor_msgs.msg import Joy, Image, JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.action import MoveGroup

# TF for pose transformations
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

# CV Bridge for image conversion
from cv_bridge import CvBridge

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # --- Parameters and Configuration ---
        self.declare_parameter("move_group_name", "arm_group")
        self.declare_parameter("end_effector_link", "electromagnet_plate")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("dataset_path", f"thor_dataset_{datetime.now().strftime('%Y%m%d_%H%M')}.hdf5")
        
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.dataset_path = self.get_parameter("dataset_path").value

        # --- State Management ---
        self.state_lock = threading.Lock()
        self.is_recording = False
        self.motion_in_progress = False
        self.episode_data = {}
        self.episode_count = 0

        # --- Data Storage ---
        self.latest_joy_msg = None
        self.latest_top_image = None
        self.latest_base_image = None
        self.latest_joint_states = None
        self.latest_ee_pose = None
        self.target_pose = None # The pose we are commanding the robot to go to

        # --- ROS Communications ---
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.top_cam_sub = self.create_subscription(Image, '/camera_top/image_raw', self.top_cam_callback, 10)
        self.base_cam_sub = self.create_subscription(Image, '/camera_base/image_raw', self.base_cam_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- Timers ---
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10 Hz control loop
        self.record_timer = self.create_timer(0.1, self.record_step)  # 10 Hz recording loop

        self.get_logger().info("🤖 Thor Data Recorder Initialized.")
        self.get_logger().info(f"💾 Dataset will be saved to: {self.dataset_path}")
        self.print_instructions()

    def print_instructions(self):
        """Prints the control instructions to the console."""
        self.get_logger().info("\n" + "="*50 +
            "\n          🎮 Thor Joystick Teleop & Recorder 🔴" +
            "\n" + "="*50 +
            "\n  Movement (Left Stick):" +
            "\n    Fwd/Bwd [Y-Axis]   Left/Right [X-Axis]" +
            "\n  Movement (Right Stick):" +
            "\n    Up/Down [Y-Axis]" +
            "\n  Orientation (D-Pad):" +
            "\n    Yaw: [Left/Right]   Pitch: [Up/Down]" +
            "\n  Orientation (Shoulder Buttons):" +
            "\n    Roll: [L1]/[R1]" +
            "\n\n  Actions:" +
            "\n    [Start/Options Button]: Start/Stop Recording Episode" +
            "\n    [R2/RT Trigger]: Toggle Electromagnet ON/OFF" +
            "\n" + "="*50)

    # --- Callback Functions ---
    def joy_callback(self, msg):
        with self.state_lock:
            self.latest_joy_msg = msg

    def top_cam_callback(self, msg):
        with self.state_lock:
            self.latest_top_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def base_cam_callback(self, msg):
        with self.state_lock:
            self.latest_base_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def joint_state_callback(self, msg):
        with self.state_lock:
            self.latest_joint_states = msg

    # --- Main Loops ---
    def control_loop(self):
        """Processes joystick input and sends commands to the robot."""
        with self.state_lock:
            if self.latest_joy_msg is None:
                return
            
            # Initialize target pose from current pose if it's the first run
            if self.target_pose is None:
                self.get_current_ee_pose()
                if self.latest_ee_pose:
                    self.target_pose = self.latest_ee_pose
                else:
                    self.get_logger().warn("Waiting for initial end-effector pose...")
                    return

            # --- Handle Recording Button ---
            # PS5 Start is button 9, Xbox is button 7
            if self.latest_joy_msg.buttons[9] == 1 or self.latest_joy_msg.buttons[7] == 1:
                if not hasattr(self, 'record_button_pressed') or not self.record_button_pressed:
                    self.toggle_recording()
                self.record_button_pressed = True
            else:
                self.record_button_pressed = False

            # --- Handle Electromagnet Button ---
            # R2/RT is axis 5, value goes from 1.0 (released) to -1.0 (pressed)
            if self.latest_joy_msg.axes[5] < -0.9:
                 if not hasattr(self, 'magnet_button_pressed') or not self.magnet_button_pressed:
                    self.toggle_electromagnet()
                 self.magnet_button_pressed = True
            else:
                self.magnet_button_pressed = False

            # --- Update Target Pose based on Joystick ---
            # Position (X, Y, Z)
            self.target_pose.pose.position.x -= self.latest_joy_msg.axes[1] * 0.01  # Left Stick Y (Fwd/Bwd)
            self.target_pose.pose.position.y -= self.latest_joy_msg.axes[0] * 0.01  # Left Stick X (L/R)
            self.target_pose.pose.position.z += self.latest_joy_msg.axes[4] * 0.01  # Right Stick Y (Up/Down)

            # Orientation (Roll, Pitch, Yaw)
            roll_inc = (self.latest_joy_msg.buttons[5] - self.latest_joy_msg.buttons[4]) * 0.05 # R1 - L1
            pitch_inc = self.latest_joy_msg.axes[7] * 0.05 # D-Pad Up/Down
            yaw_inc = -self.latest_joy_msg.axes[6] * 0.05   # D-Pad Left/Right

            o = self.target_pose.pose.orientation
            current_q = [o.x, o.y, o.z, o.w]
            delta_q = quaternion_from_euler(roll_inc, pitch_inc, yaw_inc)
            new_q = quaternion_multiply(current_q, delta_q)
            o.x, o.y, o.z, o.w = new_q[0], new_q[1], new_q[2], new_q[3]
            
            # --- Send Goal to MoveIt ---
            if not self.motion_in_progress:
                self.motion_in_progress = True
                self.move_to_pose(self.target_pose)

    def record_step(self):
        """If recording, captures the current state and saves it to the dataset."""
        with self.state_lock:
            if not self.is_recording:
                return
            
            # Ensure we have all data before saving a step
            if not all([self.latest_top_image is not None, 
                        self.latest_base_image is not None, 
                        self.latest_joint_states is not None,
                        self.latest_ee_pose is not None,
                        self.latest_joy_msg is not None]):
                self.get_logger().warn("Skipping record step, missing data.")
                return

            # Append data to the current episode's buffer
            self.episode_data["observation/top_image"].append(self.latest_top_image)
            self.episode_data["observation/base_image"].append(self.latest_base_image)
            self.episode_data["observation/joint_states"].append(self.latest_joint_states.position)
            
            pose = self.latest_ee_pose.pose
            quat = pose.orientation
            r, p, y = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            pose_array = [pose.position.x, pose.position.y, pose.position.z, r, p, y]
            self.episode_data["observation/ee_pose"].append(pose_array)

            # Action is the commanded delta from the joystick
            action = [
                -self.latest_joy_msg.axes[1], # X vel
                -self.latest_joy_msg.axes[0], # Y vel
                self.latest_joy_msg.axes[4],  # Z vel
                (self.latest_joy_msg.buttons[5] - self.latest_joy_msg.buttons[4]), # Roll vel
                self.latest_joy_msg.axes[7],  # Pitch vel
                -self.latest_joy_msg.axes[6],  # Yaw vel
                1.0 if self.latest_joy_msg.axes[5] < -0.9 else -1.0 # Gripper state
            ]
            self.episode_data["action"].append(action)
            self.get_logger().info(f"🔴 Rec Step {len(self.episode_data['action'])}", throttle_duration_sec=1.0)

    # --- Helper Functions ---
    def get_current_ee_pose(self):
        """Retrieves the current end-effector pose from TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_link, rclpy.time.Time()
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            self.latest_ee_pose = PoseStamped(header=transform.header, pose=pose)
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}", throttle_duration_sec=1.0)

    def toggle_recording(self):
        """Starts or stops recording an episode."""
        with self.state_lock:
            self.is_recording = not self.is_recording
            if self.is_recording:
                self.start_new_episode()
            else:
                self.save_episode()

    def start_new_episode(self):
        """Initializes buffers for a new recording episode."""
        self.get_logger().info(f"🔴 STARTING RECORDING: Episode {self.episode_count}")
        self.episode_data = {
            "observation/top_image": [],
            "observation/base_image": [],
            "observation/joint_states": [],
            "observation/ee_pose": [],
            "action": []
        }

    def save_episode(self):
        """Saves the buffered episode data to the HDF5 file."""
        self.get_logger().info(f"💾 SAVING Episode {self.episode_count}...")
        
        # Check if any data was recorded
        if not self.episode_data["action"]:
            self.get_logger().warn("No data recorded for this episode, skipping save.")
            self.episode_count += 1
            return

        with h5py.File(self.dataset_path, 'a') as f:
            episode_group = f.create_group(f"episode_{self.episode_count}")
            for key, data_list in self.episode_data.items():
                episode_group.create_dataset(key, data=np.array(data_list), compression="gzip")
        
        self.get_logger().info(f"✅ Episode {self.episode_count} saved with {len(self.episode_data['action'])} steps.")
        self.episode_count += 1
        self.episode_data = {}

    def toggle_electromagnet(self):
        """Toggles the state of the electromagnet."""
        self.electromagnet_state = not hasattr(self, 'electromagnet_state') or not self.electromagnet_state
        msg = Bool(data=self.electromagnet_state)
        self.electromagnet_pub.publish(msg)
        self.get_logger().info(f"🧲 Electromagnet toggled {'ON' if self.electromagnet_state else 'OFF'}")

    def move_to_pose(self, pose_stamped):
        """Sends a goal to the MoveGroup action server."""
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = self.move_group_name
        request.goal_constraints.append(self.pose_to_constraints(pose_stamped))
        goal_msg.request = request
        
        future = self.move_group_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_done_callback)

    def pose_to_constraints(self, pose_stamped):
        """Converts a PoseStamped to MoveIt constraints."""
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint
        constraints = Constraints()
        # Position
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        # Orientation
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.base_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = pose_stamped.pose.orientation
        orient_constraint.weight = 1.0
        constraints.orientation_constraints.append(orient_constraint)
        return constraints

    def goal_done_callback(self, future):
        """Callback for when a motion is complete."""
        # This just releases the lock, allowing the next command to be sent.
        self.motion_in_progress = False
        # Update our knowledge of the current pose after the move
        self.get_current_ee_pose()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    # Use a MultiThreadedExecutor to handle callbacks from multiple subscribers
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