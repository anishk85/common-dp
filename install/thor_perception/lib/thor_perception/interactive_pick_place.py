#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import math

class InteractivePickPlaceController(Node):
    def __init__(self):
        super().__init__('interactive_pick_place_controller')
        
        # Debug flag
        self.debug = True
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        # Action clients
        self.joint_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        # State variables
        self.current_image = None
        self.current_joint_states = None
        self.selected_point = None
        self.movement_in_progress = False
        self.electromagnet_active = False
        self.system_ready = False
        
        # Joint configuration
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # IMPROVED: Better predefined positions based on robot geometry
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.scan_position = [0.0, -0.3, 0.5, 0.0, -0.2, 0.0]  # Better scan pose
        
        # IMPROVED: Better workspace mapping based on robot reach
        self.robot_base_height = 0.83  # Table height
        self.pickup_height = 0.85      # Slightly above table
        self.approach_height = 0.95    # Approach from above
        
        # Workspace parameters (more realistic for robot arm)
        self.workspace_x_min = 0.2   # Robot can't reach too close to base
        self.workspace_x_max = 0.6   # Max reach
        self.workspace_y_min = -0.3
        self.workspace_y_max = 0.3
        
        # Camera parameters
        self.image_width = 640
        self.image_height = 480
        
        # Motion sequence state
        self.motion_sequence_active = False
        self.current_target = None
        
        # Initialize system
        self.create_timer(1.0, self.check_system_status)
        
        self.get_logger().info("🎯 Interactive Pick & Place Controller Started")
        self.get_logger().info("📱 Waiting for system initialization...")
        
        # Start GUI after a delay
        self.create_timer(3.0, self.start_gui_once)
    
    def start_gui_once(self):
        """Start GUI only once"""
        if not hasattr(self, 'gui_started'):
            self.gui_started = True
            self.gui_thread = threading.Thread(target=self.run_gui)
            self.gui_thread.daemon = True
            self.gui_thread.start()
    
    def check_system_status(self):
        """Check if all required components are ready"""
        image_ready = self.current_image is not None
        joints_ready = self.current_joint_states is not None
        action_ready = self.joint_trajectory_client.wait_for_server(timeout_sec=0.1)
        
        # Only log status occasionally to reduce spam
        if hasattr(self, '_status_counter'):
            self._status_counter += 1
        else:
            self._status_counter = 0
            
        if self._status_counter % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f"📊 System Status - Image: {image_ready}, Joints: {joints_ready}, Action: {action_ready}")
        
        # Make system ready when components are available
        if image_ready and joints_ready and action_ready and not self.system_ready:
            self.system_ready = True
            self.get_logger().info("✅ Full System Ready! Moving to scan position...")
            self.move_to_scan_position()
            
            # Log current joint states for debugging
            if self.current_joint_states:
                joint_info = []
                for i, (name, pos) in enumerate(zip(self.current_joint_states.name[:6], self.current_joint_states.position[:6])):
                    joint_info.append(f"{name}: {pos:.3f}")
                self.get_logger().info(f"🤖 Current joint states: {', '.join(joint_info)}")
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not hasattr(self, '_image_logged'):
                self._image_logged = True
                self.get_logger().info(f"📷 Camera image received: {msg.width}x{msg.height}")
        except Exception as e:
            self.get_logger().error(f"❌ Image conversion error: {e}")
    
    def joint_states_callback(self, msg):
        """Store joint states"""
        self.current_joint_states = msg
        if not hasattr(self, '_joints_logged'):
            self._joints_logged = True
            joint_info = ", ".join([f"{name}: {pos:.2f}" for name, pos in zip(msg.name[:6], msg.position[:6])])
            self.get_logger().info(f"🤖 Joint states received: {joint_info}")
    
    def pixel_to_workspace(self, pixel_x, pixel_y):
        """Convert pixel coordinates to workspace coordinates with better mapping"""
        if self.current_image is None:
            return None
        
        # Get actual image dimensions
        img_height, img_width = self.current_image.shape[:2]
        
        # Normalize pixel coordinates to [0, 1]
        norm_x = pixel_x / img_width
        norm_y = pixel_y / img_height
        
        # IMPROVED: Better workspace mapping
        # Map to robot workspace (front of robot, reachable area)
        world_x = self.workspace_x_min + norm_x * (self.workspace_x_max - self.workspace_x_min)
        world_y = self.workspace_y_min + (1.0 - norm_y) * (self.workspace_y_max - self.workspace_y_min)  # Invert Y
        world_z = self.pickup_height
        
        self.get_logger().info(f"🎯 PIXEL TO WORLD CONVERSION:")
        self.get_logger().info(f"   Pixel: ({pixel_x}, {pixel_y}) in {img_width}x{img_height} image")
        self.get_logger().info(f"   Normalized: ({norm_x:.3f}, {norm_y:.3f})")
        self.get_logger().info(f"   World: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        
        return (world_x, world_y, world_z)
    
    def run_gui(self):
        """Run the GUI for object selection"""
        window_name = 'Thor Camera - Click to Pick Objects'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 600)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        self.get_logger().info("🖥️ GUI Started - Click on objects to pick them!")
        
        while True:
            if self.current_image is not None:
                display_image = self.current_image.copy()
                
                # Add instructions and status
                cv2.putText(display_image, "Click on object to pick it up!", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_image, f"System: {'READY' if self.system_ready else 'INITIALIZING'}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.system_ready else (0, 255, 255), 2)
                cv2.putText(display_image, f"Magnet: {'ON' if self.electromagnet_active else 'OFF'}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Show current joint positions if available
                if self.current_joint_states and len(self.current_joint_states.position) >= 6:
                    joint_text = f"Joints: " + ", ".join([f"J{i+1}:{pos:.2f}" for i, pos in enumerate(self.current_joint_states.position[:6])])
                    cv2.putText(display_image, joint_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                cv2.putText(display_image, "Controls: h=home, s=scan, m=magnet, p=pick demo, q=quit", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show workspace boundaries
                img_height, img_width = display_image.shape[:2]
                cv2.rectangle(display_image, (50, 50), (img_width-50, img_height-100), (255, 255, 0), 2)
                cv2.putText(display_image, "Robot Workspace", (60, 45), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Show selected point
                if self.selected_point is not None:
                    x, y = self.selected_point
                    cv2.circle(display_image, (x, y), 20, (0, 0, 255), 3)
                    cv2.circle(display_image, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(display_image, "TARGET", (x+25, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Show movement status
                if self.movement_in_progress:
                    cv2.putText(display_image, "EXECUTING MOTION...", 
                               (10, display_image.shape[0] - 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                if self.motion_sequence_active:
                    cv2.putText(display_image, "PICK SEQUENCE ACTIVE", 
                               (10, display_image.shape[0] - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                cv2.imshow(window_name, display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🏠 Moving to home position...")
                    self.move_to_joint_positions(self.home_position, "HOME")
                elif key == ord('s') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🔍 Moving to scan position...")
                    self.move_to_joint_positions(self.scan_position, "SCAN")
                elif key == ord('m'):
                    self.toggle_electromagnet()
                elif key == ord('p') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🎯 Running pick demo sequence...")
                    self.run_pick_demo()
                elif key == ord('r'):
                    self.selected_point = None
                    self.motion_sequence_active = False
                    self.get_logger().info("🔄 Reset all selections")
                    
            else:
                # Show waiting message
                waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_image, "Waiting for camera...", 
                           (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(window_name, waiting_image)
                cv2.waitKey(100)
        
        cv2.destroyAllWindows()
        self.get_logger().info("🖥️ GUI Closed")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks"""
        if event == cv2.EVENT_LBUTTONDOWN and self.system_ready and not self.movement_in_progress:
            self.selected_point = (x, y)
            self.get_logger().info(f"🎯 OBJECT CLICKED at pixel: ({x}, {y})")
            
            # Convert pixel to workspace coordinates
            world_pos = self.pixel_to_workspace(x, y)
            if world_pos:
                self.current_target = world_pos
                self.get_logger().info(f"📍 Target workspace position: ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
                
                # Start full pick sequence
                self.get_logger().info("🚀 STARTING FULL PICK SEQUENCE")
                self.start_pick_sequence(world_pos)
    
    def start_pick_sequence(self, target_pos):
        """Start complete pick and place sequence"""
        self.motion_sequence_active = True
        self.get_logger().info("🎭 PICK SEQUENCE: Starting complete pick and place sequence")
        
        def sequence_thread():
            try:
                # Step 1: Move to approach position (above target)
                approach_pos = (target_pos[0], target_pos[1], self.approach_height)
                approach_joints = self.calculate_approach_joints(approach_pos)
                self.get_logger().info(f"🎯 STEP 1: Moving to approach position {approach_pos}")
                self.move_to_joint_positions(approach_joints, "APPROACH")
                
                # Wait for movement to complete
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(1)  # Extra settle time
                
                # Step 2: Move down to pickup position
                pickup_joints = self.calculate_pickup_joints(target_pos)
                self.get_logger().info(f"🎯 STEP 2: Moving to pickup position {target_pos}")
                self.move_to_joint_positions(pickup_joints, "PICKUP")
                
                # Wait for movement to complete
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(1)
                
                # Step 3: Activate electromagnet
                self.get_logger().info("🧲 STEP 3: Activating electromagnet")
                self.activate_electromagnet()
                time.sleep(2)  # Wait for magnet to engage
                
                # Step 4: Lift up
                self.get_logger().info("⬆️ STEP 4: Lifting object")
                self.move_to_joint_positions(approach_joints, "LIFT")
                
                # Wait for movement to complete
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(1)
                
                # Step 5: Move to drop location
                drop_pos = (-0.2, -0.2, self.approach_height)  # Drop to the side
                drop_joints = self.calculate_approach_joints(drop_pos)
                self.get_logger().info(f"🎯 STEP 5: Moving to drop position {drop_pos}")
                self.move_to_joint_positions(drop_joints, "DROP_APPROACH")
                
                # Wait for movement to complete
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(1)
                
                # Step 6: Lower and release
                drop_pickup_pos = (drop_pos[0], drop_pos[1], self.pickup_height)
                drop_pickup_joints = self.calculate_pickup_joints(drop_pickup_pos)
                self.get_logger().info("⬇️ STEP 6: Lowering to drop position")
                self.move_to_joint_positions(drop_pickup_joints, "DROP_LOWER")
                
                # Wait for movement to complete
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(1)
                
                # Step 7: Release electromagnet
                self.get_logger().info("⚫ STEP 7: Releasing electromagnet")
                self.deactivate_electromagnet()
                time.sleep(1)
                
                # Step 8: Return to scan position
                self.get_logger().info("🏠 STEP 8: Returning to scan position")
                self.move_to_joint_positions(self.scan_position, "RETURN_SCAN")
                
                # Wait for final movement
                while self.movement_in_progress:
                    time.sleep(0.1)
                
                self.get_logger().info("✅ PICK SEQUENCE COMPLETED SUCCESSFULLY!")
                
            except Exception as e:
                self.get_logger().error(f"❌ PICK SEQUENCE ERROR: {e}")
            finally:
                self.motion_sequence_active = False
                self.selected_point = None
        
        # Run sequence in separate thread
        sequence = threading.Thread(target=sequence_thread)
        sequence.daemon = True
        sequence.start()
    
    def calculate_approach_joints(self, target_pos):
        """Calculate joint positions for approaching target from above"""
        x, y, z = target_pos
        
        # Simple inverse kinematics approximation
        # Base rotation to point towards target
        base_angle = math.atan2(y, x)
        
        # Distance from base to target
        distance = math.sqrt(x*x + y*y)
        
        # Approximate joint angles for reaching position
        shoulder_angle = -0.3  # Shoulder down
        elbow_angle = 0.6      # Elbow bent
        wrist1_angle = -0.3    # Wrist compensation
        wrist2_angle = 0.0     # Keep level
        wrist3_angle = 0.0     # No rotation
        
        # Adjust based on distance
        if distance > 0.5:
            shoulder_angle = -0.5
            elbow_angle = 0.8
        elif distance < 0.3:
            shoulder_angle = -0.1
            elbow_angle = 0.3
        
        joints = [base_angle, shoulder_angle, elbow_angle, wrist1_angle, wrist2_angle, wrist3_angle]
        
        self.get_logger().info(f"🧮 CALCULATED APPROACH JOINTS:")
        self.get_logger().info(f"   Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.get_logger().info(f"   Distance: {distance:.3f}m")
        self.get_logger().info(f"   Base angle: {base_angle:.3f} rad ({math.degrees(base_angle):.1f}°)")
        self.get_logger().info(f"   Joints: {[f'{j:.3f}' for j in joints]}")
        
        return joints
    
    def calculate_pickup_joints(self, target_pos):
        """Calculate joint positions for pickup (lower than approach)"""
        approach_joints = self.calculate_approach_joints(target_pos)
        
        # Lower the arm more for pickup
        pickup_joints = approach_joints.copy()
        pickup_joints[1] -= 0.2  # Lower shoulder more
        pickup_joints[2] += 0.2  # Bend elbow more
        pickup_joints[3] -= 0.1  # Adjust wrist
        
        self.get_logger().info(f"🧮 CALCULATED PICKUP JOINTS:")
        self.get_logger().info(f"   Pickup joints: {[f'{j:.3f}' for j in pickup_joints]}")
        
        return pickup_joints
    
    def run_pick_demo(self):
        """Run a demo pick sequence to test motion"""
        if self.movement_in_progress or self.motion_sequence_active:
            self.get_logger().warn("⚠️ Motion already in progress")
            return
        
        # Pick a demo position in front of the robot
        demo_pos = (0.4, 0.0, self.pickup_height)
        self.current_target = demo_pos
        self.selected_point = (320, 300)  # Center of image
        
        self.get_logger().info(f"🎭 DEMO: Starting pick sequence at {demo_pos}")
        self.start_pick_sequence(demo_pos)
    
    def move_to_scan_position(self):
        """Move to scan position"""
        if self.system_ready and not self.movement_in_progress:
            self.get_logger().info("🔍 Moving to scan position...")
            self.move_to_joint_positions(self.scan_position, "INITIAL_SCAN")
    
    def move_to_joint_positions(self, joint_positions, move_type="UNKNOWN"):
        """Move to specified joint positions with detailed logging"""
        if self.movement_in_progress:
            self.get_logger().warn(f"⚠️ Movement already in progress, cannot execute {move_type}")
            return False
        
        self.movement_in_progress = True
        
        # Log detailed movement information
        self.get_logger().info("="*60)
        self.get_logger().info(f"🤖 EXECUTING MOTION: {move_type}")
        
        # Current joints
        if self.current_joint_states and len(self.current_joint_states.position) >= 6:
            current_joints = list(self.current_joint_states.position[:6])
            self.get_logger().info(f"📍 CURRENT JOINTS:")
            for i, (name, current, target) in enumerate(zip(self.joint_names, current_joints, joint_positions)):
                delta = target - current
                self.get_logger().info(f"   {name}: {current:.3f} → {target:.3f} (Δ{delta:+.3f})")
        
        self.get_logger().info(f"🎯 TARGET JOINTS: {[f'{j:.3f}' for j in joint_positions]}")
        
        try:
            # Check action server
            if not self.joint_trajectory_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ Joint trajectory action server not available!")
                self.movement_in_progress = False
                return False
            
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point with appropriate timing
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0] * len(joint_positions)
            point.accelerations = [0.0] * len(joint_positions)
            
            # Adjust timing based on movement type
            if move_type in ["APPROACH", "PICKUP", "LIFT"]:
                duration = 4.0  # Slower for precise movements
            elif move_type in ["DROP_APPROACH", "DROP_LOWER"]:
                duration = 3.0
            else:
                duration = 5.0  # Default timing
            
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            trajectory.points = [point]
            
            # Create and send goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            self.get_logger().info(f"📤 Sending {move_type} trajectory (duration: {duration}s)...")
            
            # Send goal
            future = self.joint_trajectory_client.send_goal_async(goal)
            
            def goal_response_callback(future_result):
                try:
                    goal_handle = future_result.result()
                    if not goal_handle.accepted:
                        self.get_logger().error(f"❌ {move_type} trajectory goal REJECTED")
                        self.movement_in_progress = False
                        return
                    
                    self.get_logger().info(f"✅ {move_type} trajectory goal ACCEPTED")
                    
                    # Get result
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(lambda f: self.trajectory_result_callback(f, move_type))
                    
                except Exception as e:
                    self.get_logger().error(f"❌ {move_type} goal response error: {e}")
                    self.movement_in_progress = False
            
            future.add_done_callback(goal_response_callback)
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ {move_type} trajectory execution error: {e}")
            self.movement_in_progress = False
            return False
    
    def trajectory_result_callback(self, future, move_type):
        """Handle trajectory execution result with move type"""
        try:
            result = future.result()
            if hasattr(result.result, 'error_code'):
                error_code = result.result.error_code
            else:
                error_code = 0  # Assume success if no error code
                
            if error_code == 0:
                self.get_logger().info(f"✅ {move_type} trajectory completed successfully")
                
                # Log final joint positions
                if self.current_joint_states and len(self.current_joint_states.position) >= 6:
                    final_joints = list(self.current_joint_states.position[:6])
                    self.get_logger().info(f"📍 FINAL {move_type} JOINTS: {[f'{j:.3f}' for j in final_joints]}")
            else:
                self.get_logger().error(f"❌ {move_type} trajectory failed with error: {error_code}")
        except Exception as e:
            self.get_logger().error(f"❌ {move_type} trajectory result error: {e}")
        finally:
            self.movement_in_progress = False
            self.get_logger().info(f"🏁 {move_type} motion completed")
            self.get_logger().info("="*60)
    
    def activate_electromagnet(self):
        """Activate electromagnet"""
        self.electromagnet_active = True
        msg = Bool()
        msg.data = True
        self.electromagnet_pub.publish(msg)
        self.get_logger().info("🧲 Electromagnet ACTIVATED")
    
    def deactivate_electromagnet(self):
        """Deactivate electromagnet"""
        self.electromagnet_active = False
        msg = Bool()
        msg.data = False
        self.electromagnet_pub.publish(msg)
        self.get_logger().info("⚫ Electromagnet DEACTIVATED")
    
    def toggle_electromagnet(self):
        """Toggle electromagnet"""
        if self.electromagnet_active:
            self.deactivate_electromagnet()
        else:
            self.activate_electromagnet()

def main(args=None):
    rclpy.init(args=args)
    node = InteractivePickPlaceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()