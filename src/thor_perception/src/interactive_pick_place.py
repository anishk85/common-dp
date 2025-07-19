#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, JointConstraint
from moveit_msgs.srv import GetPlanningScene
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import math

class InteractivePickPlaceController(Node):
    def __init__(self):
        super().__init__('interactive_pick_place_controller')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        # FIXED: Use MoveIt action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Service clients for MoveIt
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
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
        
        # FIXED: MoveIt configuration based on URDF
        self.move_group_name = "arm_group"  # MoveIt planning group
        self.end_effector_link = "electromagnet_plate"  # FIXED: From URDF - the actual end effector
        
        # Robot parameters
        self.robot_base_height = 0.0
        self.ground_level = 0.02
        self.safe_approach_height = 0.15
        
        # FIXED: More conservative predefined poses
        self.home_pose = self.create_pose(0.3, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0)
        self.scan_pose = self.create_pose(0.35, 0.0, 0.3, 0.707, 0.0, 0.0, 0.707)  # Point down for scanning
        
        # Motion sequence state
        self.motion_sequence_active = False
        self.current_target = None
        
        # Initialize system
        self.create_timer(1.0, self.check_system_status)
        
        self.get_logger().info("🎯 Interactive Pick & Place Controller Started - MOVEIT INTEGRATION")
        self.get_logger().info("🔧 FIXED: Using correct end effector link 'electromagnet_plate'")
        self.get_logger().info("📱 Waiting for system initialization...")
        
        # Start GUI after a delay
        self.create_timer(3.0, self.start_gui_once)
    
    def create_pose(self, x, y, z, qx, qy, qz, qw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"  # Robot base frame from URDF
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        return pose
    
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
        moveit_ready = self.move_group_client.wait_for_server(timeout_sec=0.1)
        
        if hasattr(self, '_status_counter'):
            self._status_counter += 1
        else:
            self._status_counter = 0
            
        if self._status_counter % 15 == 0:  # Log every 15 seconds
            self.get_logger().info(f"📊 System Status - Image: {image_ready}, Joints: {joints_ready}, MoveIt: {moveit_ready}")
            if joints_ready:
                self.log_robot_state()
        
        if image_ready and joints_ready and moveit_ready and not self.system_ready:
            self.system_ready = True
            self.get_logger().info("✅ MoveIt System Ready! Moving to scan position...")
            self.move_to_pose(self.scan_pose, "INITIAL_SCAN")
    
    def log_robot_state(self):
        """Log current robot state"""
        if self.current_joint_states and len(self.current_joint_states.position) >= 6:
            joints = list(self.current_joint_states.position[:6])
            joint_degrees = [f'J{i+1}:{math.degrees(j):.1f}°' for i, j in enumerate(joints)]
            self.get_logger().info(f"🤖 Current joints: {joint_degrees}")
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.current_image is not None:
                self.image_height, self.image_width = self.current_image.shape[:2]
            
            if not hasattr(self, '_image_logged'):
                self._image_logged = True
                self.get_logger().info(f"📷 Camera image received: {self.image_width}x{self.image_height}")
        except Exception as e:
            self.get_logger().error(f"❌ Image conversion error: {e}")
    
    def joint_states_callback(self, msg):
        """Store joint states"""
        self.current_joint_states = msg
        if not hasattr(self, '_joints_logged'):
            self._joints_logged = True
            self.get_logger().info(f"🤖 Joint states received: {len(msg.position)} joints")
    
    def pixel_to_ground_coordinates(self, pixel_x, pixel_y):
        """Convert pixel coordinates to ground-level world coordinates"""
        if self.current_image is None:
            return None
        
        img_height, img_width = self.current_image.shape[:2]
        
        # Normalize pixel coordinates
        norm_x = pixel_x / img_width         # 0 = left, 1 = right
        norm_y = pixel_y / img_height        # 0 = top, 1 = bottom
        
        self.get_logger().info(f"🎯 PIXEL TO WORLD CONVERSION:")
        self.get_logger().info(f"   Pixel: ({pixel_x}, {pixel_y}) in {img_width}x{img_height} image")
        self.get_logger().info(f"   Normalized: ({norm_x:.3f}, {norm_y:.3f})")
        
        # FIXED: Map to robot reachable workspace based on URDF dimensions
        # From URDF: Base height = 0.099, Link lengths suggest max reach ~0.5m
        world_x = 0.20 + (1.0 - norm_y) * 0.25  # 0.20m to 0.45m forward
        world_y = (norm_x - 0.5) * 0.30         # -0.15m to +0.15m left/right
        world_z = self.ground_level
        
        self.get_logger().info(f"   World coordinates: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        
        return (world_x, world_y, world_z)
    
    def create_ground_pickup_pose(self, target_pos):
        """Create pose for ground-level pickup"""
        x, y, z = target_pos
        
        self.get_logger().info(f"🧮 CREATING GROUND PICKUP POSE:")
        self.get_logger().info(f"   Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # FIXED: Create pose with end effector pointing DOWN
        # Quaternion for 90° rotation around X-axis (pointing down)
        ground_pose = self.create_pose(
            x, y, z,
            0.707, 0.0, 0.0, 0.707  # Point electromagnet straight down
        )
        
        return ground_pose
    
    def create_approach_pose(self, target_pos):
        """Create approach pose above target"""
        x, y, z = target_pos
        approach_z = z + self.safe_approach_height
        
        self.get_logger().info(f"🧮 CREATING APPROACH POSE:")
        self.get_logger().info(f"   Approach: ({x:.3f}, {y:.3f}, {approach_z:.3f})")
        
        # Same orientation as ground pose but higher
        approach_pose = self.create_pose(
            x, y, approach_z,
            0.707, 0.0, 0.0, 0.707  # Point down
        )
        
        return approach_pose
    
    def move_to_pose(self, target_pose, move_type="MOVE"):
        """FIXED: Use MoveIt to move to target pose with correct end effector"""
        if self.movement_in_progress:
            self.get_logger().warn(f"⚠️ Movement in progress, cannot execute {move_type}")
            return False
        
        self.movement_in_progress = True
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"🎯 MOVEIT EXECUTION: {move_type}")
        
        # Log target pose
        pos = target_pose.pose.position
        orient = target_pose.pose.orientation
        self.get_logger().info(f"📍 Target pose: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        self.get_logger().info(f"📐 Target orientation: ({orient.x:.3f}, {orient.y:.3f}, {orient.z:.3f}, {orient.w:.3f})")
        self.get_logger().info(f"🔗 End effector link: {self.end_effector_link}")
        
        try:
            # Check MoveIt action server
            if not self.move_group_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ MoveIt action server not available!")
                self.movement_in_progress = False
                return False
            
            # FIXED: Create simplified MoveIt goal
            goal = MoveGroup.Goal()
            
            # Set up planning request
            goal.request.group_name = self.move_group_name
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 10.0
            # Remove specific planner to use default
            
            # Set workspace parameters
            goal.request.workspace_parameters = WorkspaceParameters()
            goal.request.workspace_parameters.header.frame_id = "base_link"
            goal.request.workspace_parameters.min_corner.x = -0.6
            goal.request.workspace_parameters.min_corner.y = -0.6
            goal.request.workspace_parameters.min_corner.z = 0.0
            goal.request.workspace_parameters.max_corner.x = 0.6
            goal.request.workspace_parameters.max_corner.y = 0.6
            goal.request.workspace_parameters.max_corner.z = 0.8
            
            # FIXED: Simplified goal constraint using pose goal
            from moveit_msgs.msg import PositionConstraint, OrientationConstraint
            from shape_msgs.msg import SolidPrimitive
            
            # Position constraint with CORRECT end effector link
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = self.end_effector_link  # FIXED: electromagnet_plate
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Create bounding volume (larger tolerance)
            pos_constraint.constraint_region.primitives = [SolidPrimitive()]
            pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
            pos_constraint.constraint_region.primitives[0].dimensions = [0.05, 0.05, 0.05]  # 5cm tolerance
            
            pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
            pos_constraint.weight = 1.0
            
            # Orientation constraint with CORRECT end effector link
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = "base_link"
            orient_constraint.link_name = self.end_effector_link  # FIXED: electromagnet_plate
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.2  # More tolerant
            orient_constraint.absolute_y_axis_tolerance = 0.2
            orient_constraint.absolute_z_axis_tolerance = 0.2
            orient_constraint.weight = 0.5  # Lower weight for orientation
            
            # Add constraints to goal
            goal.request.goal_constraints = [Constraints()]
            goal.request.goal_constraints[0].position_constraints = [pos_constraint]
            goal.request.goal_constraints[0].orientation_constraints = [orient_constraint]
            
            self.get_logger().info(f"📤 Sending MoveIt goal for {move_type} with end effector '{self.end_effector_link}'...")
            
            # Send goal
            future = self.move_group_client.send_goal_async(goal)
            
            def goal_response_callback(future_result):
                try:
                    goal_handle = future_result.result()
                    if not goal_handle.accepted:
                        self.get_logger().error(f"❌ MoveIt goal REJECTED for {move_type}")
                        self.movement_in_progress = False
                        return
                    
                    self.get_logger().info(f"✅ MoveIt goal ACCEPTED for {move_type}")
                    
                    # Get result
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(lambda f: self.moveit_result_callback(f, move_type))
                    
                except Exception as e:
                    self.get_logger().error(f"❌ MoveIt goal response error: {e}")
                    self.movement_in_progress = False
            
            future.add_done_callback(goal_response_callback)
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt execution error: {e}")
            self.movement_in_progress = False
            return False
    
    def moveit_result_callback(self, future, move_type):
        """Handle MoveIt execution result"""
        try:
            result = future.result()
            
            # Check if motion planning succeeded
            if hasattr(result.result, 'error_code') and hasattr(result.result.error_code, 'val'):
                error_code = result.result.error_code.val
            else:
                error_code = 1  # Assume success if no error code
                
            if error_code == 1:  # MoveIt success code
                self.get_logger().info(f"✅ MoveIt {move_type} completed successfully!")
                
                # Log final state
                if self.current_joint_states:
                    joints = list(self.current_joint_states.position[:6])
                    joint_degrees = [f'{math.degrees(j):.1f}°' for j in joints]
                    self.get_logger().info(f"📍 FINAL {move_type} JOINTS: {joint_degrees}")
            else:
                self.get_logger().error(f"❌ MoveIt {move_type} failed with error code: {error_code}")
                
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt {move_type} result error: {e}")
        finally:
            self.movement_in_progress = False
            self.get_logger().info(f"🏁 MoveIt {move_type} completed")
            self.get_logger().info("="*60)
    
    def run_gui(self):
        """Run the GUI for object selection"""
        window_name = 'Thor MoveIt Ground Picker - FIXED'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 900, 700)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        self.get_logger().info("🖥️ FIXED MoveIt Ground-Level GUI Started!")
        
        while True:
            if self.current_image is not None:
                display_image = self.current_image.copy()
                
                # Add instructions
                cv2.putText(display_image, "FIXED MOVEIT: Click objects on ground!", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, f"System: {'READY' if self.system_ready else 'INITIALIZING'}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.system_ready else (0, 255, 255), 2)
                cv2.putText(display_image, f"Magnet: {'ON' if self.electromagnet_active else 'OFF'}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(display_image, f"End Effector: {self.end_effector_link}", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                # Show current robot state
                if self.current_joint_states and len(self.current_joint_states.position) >= 6:
                    joints = list(self.current_joint_states.position[:6])
                    joint_text = f"Joints: " + ", ".join([f"J{i+1}:{math.degrees(j):.0f}°" for i, j in enumerate(joints)])
                    cv2.putText(display_image, joint_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # Controls
                cv2.putText(display_image, "h=home, s=scan, m=magnet, p=demo, q=quit", 
                           (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show coordinate mapping
                img_height, img_width = display_image.shape[:2]
                cv2.line(display_image, (img_width//2, 0), (img_width//2, img_height), (255, 0, 0), 1)
                cv2.line(display_image, (0, img_height//2), (img_width, img_height//2), (0, 255, 0), 1)
                
                cv2.putText(display_image, "FAR", (img_width//2 + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                cv2.putText(display_image, "CLOSE", (img_width//2 + 5, img_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                cv2.putText(display_image, "LEFT", (10, img_height//2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.putText(display_image, "RIGHT", (img_width - 50, img_height//2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                # Show selected target
                if self.selected_point is not None:
                    x, y = self.selected_point
                    cv2.circle(display_image, (x, y), 20, (0, 0, 255), 3)
                    cv2.circle(display_image, (x, y), 5, (0, 255, 0), -1)
                    cv2.putText(display_image, "FIXED TARGET", (x+25, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    if hasattr(self, 'current_target') and self.current_target:
                        coord_text = f"({self.current_target[0]:.2f}, {self.current_target[1]:.2f}, {self.current_target[2]:.2f})"
                        cv2.putText(display_image, coord_text, (x+25, y+20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                # Show status
                if self.movement_in_progress:
                    cv2.putText(display_image, "EXECUTING FIXED MOVEIT MOTION...", 
                               (10, img_height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                if self.motion_sequence_active:
                    cv2.putText(display_image, "FIXED MOVEIT SEQUENCE ACTIVE", 
                               (10, img_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                cv2.imshow(window_name, display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🏠 Moving to FIXED home pose...")
                    self.move_to_pose(self.home_pose, "HOME")
                elif key == ord('s') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🔍 Moving to FIXED scan pose...")
                    self.move_to_pose(self.scan_pose, "SCAN")
                elif key == ord('m'):
                    self.toggle_electromagnet()
                elif key == ord('p') and self.system_ready and not self.movement_in_progress:
                    self.get_logger().info("🎯 Running FIXED MoveIt demo...")
                    self.run_moveit_demo()
                elif key == ord('r'):
                    self.selected_point = None
                    self.motion_sequence_active = False
                    self.current_target = None
                    self.get_logger().info("🔄 Reset")
                    
            else:
                waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_image, "Waiting for camera...", 
                           (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(waiting_image, "FIXED MoveIt Mode", 
                           (190, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow(window_name, waiting_image)
                cv2.waitKey(100)
        
        cv2.destroyAllWindows()
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks"""
        if event == cv2.EVENT_LBUTTONDOWN and self.system_ready and not self.movement_in_progress:
            self.selected_point = (x, y)
            self.get_logger().info(f"🎯 FIXED MOVEIT TARGET CLICKED: ({x}, {y})")
            
            ground_pos = self.pixel_to_ground_coordinates(x, y)
            if ground_pos:
                self.current_target = ground_pos
                self.get_logger().info(f"📍 FIXED MoveIt target: ({ground_pos[0]:.3f}, {ground_pos[1]:.3f}, {ground_pos[2]:.3f})")
                self.start_moveit_ground_sequence(ground_pos)
    
    def start_moveit_ground_sequence(self, target_pos):
        """Start FIXED MoveIt ground pickup sequence"""
        self.motion_sequence_active = True
        self.get_logger().info("🎭 FIXED MOVEIT GROUND SEQUENCE STARTED!")
        
        def sequence_thread():
            try:
                # Step 1: Approach pose
                approach_pose = self.create_approach_pose(target_pos)
                self.get_logger().info("🎯 STEP 1: FIXED MoveIt approach")
                if not self.move_to_pose(approach_pose, "FIXED_MOVEIT_APPROACH"):
                    raise Exception("Failed approach")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 2: Ground pickup pose
                pickup_pose = self.create_ground_pickup_pose(target_pos)
                self.get_logger().info("🎯 STEP 2: FIXED MoveIt ground pickup")
                if not self.move_to_pose(pickup_pose, "FIXED_MOVEIT_GROUND_PICKUP"):
                    raise Exception("Failed ground pickup")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 3: Activate magnet
                self.get_logger().info("🧲 STEP 3: Magnet ON")
                self.activate_electromagnet()
                time.sleep(3.0)
                
                # Step 4: Lift back to approach
                self.get_logger().info("⬆️ STEP 4: FIXED MoveIt lift")
                if not self.move_to_pose(approach_pose, "FIXED_MOVEIT_LIFT"):
                    raise Exception("Failed lift")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 5: Drop location
                drop_pos = (0.25, -target_pos[1], target_pos[2])
                drop_pose = self.create_ground_pickup_pose(drop_pos)
                self.get_logger().info("🎯 STEP 5: FIXED MoveIt drop")
                if not self.move_to_pose(drop_pose, "FIXED_MOVEIT_DROP"):
                    raise Exception("Failed drop move")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 6: Release magnet
                self.get_logger().info("⚫ STEP 6: Magnet OFF")
                self.deactivate_electromagnet()
                time.sleep(2.0)
                
                # Step 7: Return to scan
                self.get_logger().info("🏠 STEP 7: Return to scan")
                if not self.move_to_pose(self.scan_pose, "FIXED_MOVEIT_RETURN"):
                    raise Exception("Failed return")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                
                self.get_logger().info("✅ FIXED MOVEIT SEQUENCE COMPLETED!")
                
            except Exception as e:
                self.get_logger().error(f"❌ FIXED MOVEIT SEQUENCE ERROR: {e}")
                try:
                    self.deactivate_electromagnet()
                    self.move_to_pose(self.scan_pose, "EMERGENCY")
                except:
                    pass
            finally:
                self.motion_sequence_active = False
                self.selected_point = None
        
        sequence = threading.Thread(target=sequence_thread)
        sequence.daemon = True
        sequence.start()
    
    def run_moveit_demo(self):
        """Run FIXED MoveIt demo"""
        if self.movement_in_progress or self.motion_sequence_active:
            self.get_logger().warn("⚠️ Already in motion")
            return
        
        # Demo target
        demo_pos = (0.30, -0.08, self.ground_level)
        self.current_target = demo_pos
        self.selected_point = (420, 350)
        
        self.get_logger().info(f"🎭 FIXED MOVEIT DEMO at {demo_pos}")
        self.start_moveit_ground_sequence(demo_pos)
    
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