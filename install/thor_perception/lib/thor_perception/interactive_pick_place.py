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
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF2 for getting actual end effector position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.electromagnet_pub = self.create_publisher(Bool, '/thor_arm/electromagnet/control', 10)
        
        # MoveIt action client
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
        self.initial_scan_done = False
        
        # FIXED: MoveIt configuration
        self.move_group_name = "arm_group"
        self.end_effector_link = "electromagnet_plate"
        
        # Robot parameters - TUNED for better workspace mapping
        self.robot_base_height = 0.0
        self.ground_level = 0.15
        self.safe_approach_height = 0.15
        
        # EXPANDED: Multiple predefined test poses
                # FIXED: Reachable predefined test poses - adjusted based on robot's actual workspace
        self.predefined_poses = {
            # Safe poses that robot can actually reach
            'home': self.create_pose(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),  # Straight up, reachable
            'scan': self.create_pose(0.3, 0.0, 0.4, 0.707, 0.0, 0.0, 0.707),  # Forward, moderate height
            
            # FIXED: Test positions - REACHABLE WORKSPACE
            # Front row - closer and higher
            'front_left': self.create_pose(0.3, 0.1, 0.25, 0.707, 0.0, 0.0, 0.707),
            'front_center': self.create_pose(0.35, 0.0, 0.2, 0.707, 0.0, 0.0, 0.707),
            'front_right': self.create_pose(0.3, -0.1, 0.25, 0.707, 0.0, 0.0, 0.707),
            
            # Middle row - moderate reach
            'mid_left': self.create_pose(0.4, 0.08, 0.3, 0.707, 0.0, 0.0, 0.707),
            'mid_center': self.create_pose(0.45, 0.0, 0.25, 0.707, 0.0, 0.0, 0.707),
            'mid_right': self.create_pose(0.4, -0.08, 0.3, 0.707, 0.0, 0.0, 0.707),
            
            # Far row - maximum safe reach
            'far_left': self.create_pose(0.5, 0.05, 0.35, 0.707, 0.0, 0.0, 0.707),
            'far_center': self.create_pose(0.55, 0.0, 0.3, 0.707, 0.0, 0.0, 0.707),
            'far_right': self.create_pose(0.5, -0.05, 0.35, 0.707, 0.0, 0.0, 0.707),
            
            # Elevated - safe high positions
            'elevated_center': self.create_pose(0.35, 0.0, 0.45, 0.707, 0.0, 0.0, 0.707),
            'elevated_left': self.create_pose(0.3, 0.08, 0.4, 0.707, 0.0, 0.0, 0.707),
            'elevated_right': self.create_pose(0.3, -0.08, 0.4, 0.707, 0.0, 0.0, 0.707),
        }
        
        # DEBUG: Enable extensive logging
        self.debug_mode = True
        self.pose_history = []
        self.current_test_pose = None
        
        # Motion sequence state
        self.motion_sequence_active = False
        self.current_target = None
        
        # Initialize system
        self.create_timer(1.0, self.check_system_status)
        
        self.get_logger().info("🎯 FIXED Interactive Pick & Place Controller with DEBUG")
        self.get_logger().info("🔧 MULTIPLE PREDEFINED POSES FOR TESTING")
        self.get_logger().info("📊 DEBUG MODE ENABLED - Extensive position logging")
        self.get_logger().info("📱 Waiting for system initialization...")
        
        # Start GUI after a delay
        self.create_timer(3.0, self.start_gui_once)
    
    def create_pose(self, x, y, z, qx, qy, qz, qw):
        """Create a PoseStamped message with debug logging"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        if hasattr(self, 'debug_mode') and self.debug_mode:
            self.get_logger().info(f"🎯 POSE CREATED: ({x:.3f}, {y:.3f}, {z:.3f}) | Q({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
        
        return pose
    
    def get_current_end_effector_pose(self):
        """Get actual current end effector position using TF2"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', self.end_effector_link, rclpy.time.Time()
            )
            
            pos = transform.transform.translation
            orient = transform.transform.rotation
            
            return (pos.x, pos.y, pos.z), (orient.x, orient.y, orient.z, orient.w)
            
        except Exception as e:
            if self.debug_mode:
                self.get_logger().warn(f"⚠️ Could not get end effector pose: {e}")
            return None, None
    
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
            if joints_ready and self.debug_mode:
                self.log_robot_state()
        
        if image_ready and joints_ready and moveit_ready and not self.system_ready:
            self.system_ready = True
            self.get_logger().info("✅ DEBUG MoveIt System Ready!")
            self.get_logger().info("🎮 CONTROLS: 1-9=test poses, h=home, s=scan, p=demo, click=pick")
            self.print_predefined_poses()
    
    def print_predefined_poses(self):
        """Print all predefined poses for reference"""
        self.get_logger().info("📍 PREDEFINED TEST POSES:")
        for name, pose in self.predefined_poses.items():
            pos = pose.pose.position
            self.get_logger().info(f"   {name}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    
    def log_robot_state(self):
        """Log current robot state with detailed information"""
        if self.current_joint_states and len(self.current_joint_states.position) >= 6:
            joints = list(self.current_joint_states.position[:6])
            joint_degrees = [f'J{i+1}:{math.degrees(j):.1f}°' for i, j in enumerate(joints)]
            self.get_logger().info(f"🤖 CURRENT JOINTS: {joint_degrees}")
            
            # Get actual end effector position
            end_pos, end_orient = self.get_current_end_effector_pose()
            if end_pos:
                self.get_logger().info(f"🦾 ACTUAL END EFFECTOR: ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})")
                
                # Store in history for debugging
                self.pose_history.append({
                    'timestamp': time.time(),
                    'position': end_pos,
                    'joints': joints
                })
                
                # Keep only last 10 positions
                if len(self.pose_history) > 10:
                    self.pose_history.pop(0)
    
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
        """FIXED: Map to REACHABLE workspace"""
        if self.current_image is None:
            return None
        
        img_height, img_width = self.current_image.shape[:2]
        
        # Normalize pixel coordinates
        norm_x = pixel_x / img_width
        norm_y = pixel_y / img_height
        
        if self.debug_mode:
            self.get_logger().info("="*50)
            self.get_logger().info("🎯 PIXEL TO REACHABLE WORKSPACE:")
            self.get_logger().info(f"   📷 Image size: {img_width}x{img_height}")
            self.get_logger().info(f"   🖱️ Clicked pixel: ({pixel_x}, {pixel_y})")
            self.get_logger().info(f"   📊 Normalized: ({norm_x:.3f}, {norm_y:.3f})")
        
        # FIXED: Map to REACHABLE workspace
        world_x = 0.30 + (1.0 - norm_y) * 0.25  # 0.30m to 0.55m forward (REACHABLE)
        world_y = (norm_x - 0.5) * 0.16          # -0.08m to +0.08m left/right (SAFE)
        world_z = self.ground_level              # 0.15m (REACHABLE minimum)
        
        if self.debug_mode:
            self.get_logger().info(f"   🎯 REACHABLE COORDS: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
            self.get_logger().info(f"   ✅ Safe workspace: X[0.30-0.55], Y[-0.08-0.08], Z[{world_z:.3f}]")
            self.get_logger().info("="*50)
        
        return (world_x, world_y, world_z)
    
    def create_ground_pickup_pose(self, target_pos):
        """Create pose for ground-level pickup with debug"""
        x, y, z = target_pos
        
        if self.debug_mode:
            self.get_logger().info(f"🎯 CREATING GROUND PICKUP POSE:")
            self.get_logger().info(f"   📍 Target: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Create pose with end effector pointing DOWN
        ground_pose = self.create_pose(
            x, y, z,
            0.707, 0.0, 0.0, 0.707  # Point electromagnet straight down
        )
        
        return ground_pose
    
    def create_approach_pose(self, target_pos):
        """Create approach pose above target"""
        x, y, z = target_pos
        approach_z = z + self.safe_approach_height
        
        if self.debug_mode:
            self.get_logger().info(f"⬆️ CREATING APPROACH POSE:")
            self.get_logger().info(f"   📍 Approach: ({x:.3f}, {y:.3f}, {approach_z:.3f})")
        
        approach_pose = self.create_pose(
            x, y, approach_z,
            0.707, 0.0, 0.0, 0.707  # Point down
        )
        
        return approach_pose
    
    def move_to_pose(self, target_pose, move_type="MOVE"):
        """FIXED: Simplified MoveIt execution with better constraints"""
        if self.movement_in_progress:
            self.get_logger().warn(f"⚠️ Movement in progress, cannot execute {move_type}")
            return False
        
        self.movement_in_progress = True
        
        self.get_logger().info("="*80)
        self.get_logger().info(f"🎯 MOVEIT EXECUTION: {move_type}")
        
        # Log target pose
        pos = target_pose.pose.position
        orient = target_pose.pose.orientation
        self.get_logger().info(f"📍 TARGET POSE: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        self.get_logger().info(f"📐 TARGET ORIENTATION: ({orient.x:.3f}, {orient.y:.3f}, {orient.z:.3f}, {orient.w:.3f})")
        
        # Log current position
        current_pos, current_orient = self.get_current_end_effector_pose()
        if current_pos:
            self.get_logger().info(f"📍 CURRENT POSITION: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            distance = math.sqrt((pos.x - current_pos[0])**2 + (pos.y - current_pos[1])**2 + (pos.z - current_pos[2])**2)
            self.get_logger().info(f"📏 DISTANCE TO MOVE: {distance:.3f}m")
            
            # DEBUG: Check if positions are actually different
            if distance < 0.01:
                self.get_logger().warn(f"⚠️ WARNING: Very small movement distance ({distance*1000:.1f}mm)")
        
        try:
            # Check MoveIt action server
            if not self.move_group_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ MoveIt action server not available!")
                self.movement_in_progress = False
                return False
            
            # FIXED: Create simplified MoveIt goal
            goal = MoveGroup.Goal()
            
            # Basic planning request
            goal.request.group_name = self.move_group_name
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 10.0
            
            # FIXED: Simplified approach - just use position/orientation constraints
            from moveit_msgs.msg import PositionConstraint, OrientationConstraint
            from shape_msgs.msg import SolidPrimitive
            
            # Position constraint with larger tolerance
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = self.end_effector_link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Create bounding volume - LARGER tolerance
            pos_constraint.constraint_region.primitives = [SolidPrimitive()]
            pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
            pos_constraint.constraint_region.primitives[0].dimensions = [0.10, 0.10, 0.10]  # 10cm tolerance
            
            pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
            pos_constraint.weight = 1.0
            
            # Orientation constraint with relaxed tolerance
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = "base_link"
            orient_constraint.link_name = self.end_effector_link
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.3  # More relaxed
            orient_constraint.absolute_y_axis_tolerance = 0.3
            orient_constraint.absolute_z_axis_tolerance = 0.3
            orient_constraint.weight = 0.3  # Lower weight on orientation
            
            # Add constraints to goal
            goal.request.goal_constraints = [Constraints()]
            goal.request.goal_constraints[0].position_constraints = [pos_constraint]
            goal.request.goal_constraints[0].orientation_constraints = [orient_constraint]
            
            self.get_logger().info(f"📤 Sending MoveIt goal for {move_type}...")
            self.get_logger().info(f"🎯 Constraint tolerance: ±10cm position, ±17° orientation")
            
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
                    result_future.add_done_callback(lambda f: self.moveit_result_callback(f, move_type, target_pose))
                    
                except Exception as e:
                    self.get_logger().error(f"❌ MoveIt goal response error: {e}")
                    self.movement_in_progress = False
            
            future.add_done_callback(goal_response_callback)
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt execution error: {e}")
            self.movement_in_progress = False
            return False
    
    def moveit_result_callback(self, future, move_type, target_pose):
        """Handle MoveIt execution result with extensive debugging"""
        try:
            result = future.result()
            
            # Check if motion planning succeeded
            if hasattr(result.result, 'error_code') and hasattr(result.result.error_code, 'val'):
                error_code = result.result.error_code.val
            else:
                error_code = 1  # Assume success if no error code
                
            if error_code == 1:  # MoveIt success code
                self.get_logger().info(f"✅ MoveIt {move_type} completed successfully!")
                
                # EXTENSIVE DEBUG: Compare target vs actual
                target_pos = target_pose.pose.position
                final_pos, final_orient = self.get_current_end_effector_pose()
                
                if final_pos:
                    self.get_logger().info("📊 POSITION COMPARISON:")
                    self.get_logger().info(f"   🎯 TARGET: ({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f})")
                    self.get_logger().info(f"   📍 ACTUAL: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
                    
                    # Calculate errors
                    error_x = abs(final_pos[0] - target_pos.x)
                    error_y = abs(final_pos[1] - target_pos.y)
                    error_z = abs(final_pos[2] - target_pos.z)
                    total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                    
                    self.get_logger().info(f"   📏 ERRORS: X={error_x*1000:.1f}mm, Y={error_y*1000:.1f}mm, Z={error_z*1000:.1f}mm")
                    self.get_logger().info(f"   📏 TOTAL ERROR: {total_error*1000:.1f}mm")
                    
                    # Check for problematic patterns
                    if error_x < 0.005 and error_y < 0.005 and error_z < 0.005:
                        self.get_logger().info("✅ EXCELLENT ACCURACY!")
                    elif total_error > 0.05:
                        self.get_logger().warn(f"⚠️ LARGE POSITION ERROR: {total_error*1000:.1f}mm")
                    
                    # Special logging for pickup operations
                    if "PICKUP" in move_type:
                        ground_distance = final_pos[2] - self.ground_level
                        self.get_logger().info(f"📏 HEIGHT ABOVE GROUND: {ground_distance*1000:.1f}mm")
                        
                        if ground_distance < 0.03:
                            self.get_logger().info("✅ GOOD GROUND APPROACH!")
                        else:
                            self.get_logger().warn(f"⚠️ HIGH ABOVE GROUND: {ground_distance*1000:.1f}mm")
                
                # Log joint positions
                if self.current_joint_states and self.debug_mode:
                    joints = list(self.current_joint_states.position[:6])
                    joint_degrees = [f'{math.degrees(j):.1f}°' for j in joints]
                    self.get_logger().info(f"🤖 FINAL JOINTS: {joint_degrees}")
                    
            else:
                self.get_logger().error(f"❌ MoveIt {move_type} failed with error code: {error_code}")
                self.debug_moveit_error(error_code)
                
        except Exception as e:
            self.get_logger().error(f"❌ MoveIt {move_type} result error: {e}")
        finally:
            self.movement_in_progress = False
            self.get_logger().info(f"🏁 MoveIt {move_type} completed")
            self.get_logger().info("="*80)
    
    def debug_moveit_error(self, error_code):
        """Debug MoveIt error codes"""
        error_messages = {
            -1: "FAILURE",
            0: "SUCCESS but with warnings",
            1: "SUCCESS",
            -2: "COLLISION",
            -3: "KINEMATIC_CONSTRAINTS",
            -4: "KINEMATIC_PATH",
            -5: "ROBOT_STATE_STALE",
            -6: "INVALID_ROBOT_STATE",
            -7: "PLANNING_FAILED",
            -10: "INVALID_MOTION_PLAN",
            -12: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -13: "TIMED_OUT",
            -19: "INVALID_GROUP_NAME"
        }
        
        error_msg = error_messages.get(error_code, f"UNKNOWN_ERROR_{error_code}")
        self.get_logger().error(f"🔍 MoveIt Error: {error_msg}")
        
        if error_code == -2:
            self.get_logger().error("💥 COLLISION detected - check workspace boundaries")
        elif error_code == -3:
            self.get_logger().error("🔒 KINEMATIC_CONSTRAINTS - target may be unreachable")
        elif error_code == -7:
            self.get_logger().error("🧭 PLANNING_FAILED - try different target or approach")
    
    def run_gui(self):
        """Run the GUI with enhanced debugging features"""
        window_name = 'Thor MoveIt FIXED Controller - DEBUG MODE'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1000, 800)
        cv2.setMouseCallback(window_name, self.mouse_callback)
        
        self.get_logger().info("🖥️ DEBUG GUI Started with extensive logging!")
        
        while True:
            if self.current_image is not None:
                display_image = self.current_image.copy()
                
                # Enhanced GUI overlay with comprehensive information
                cv2.putText(display_image, "FIXED CONTROLLER - DEBUG MODE", 
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_image, "1-9=Test poses, h=home, s=scan, p=demo, click=pick", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # System status
                status_color = (0, 255, 0) if self.system_ready else (0, 255, 255)
                cv2.putText(display_image, f"System: {'READY' if self.system_ready else 'INIT'}", 
                           (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
                
                motion_color = (255, 255, 0) if self.movement_in_progress else (0, 255, 0)
                cv2.putText(display_image, f"Motion: {'BUSY' if self.movement_in_progress else 'READY'}", 
                           (200, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, motion_color, 1)
                
                cv2.putText(display_image, f"Magnet: {'ON' if self.electromagnet_active else 'OFF'}", 
                           (350, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Current position
                end_pos, _ = self.get_current_end_effector_pose()
                if end_pos:
                    pos_text = f"Position: ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})"
                    cv2.putText(display_image, pos_text, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    
                    height_mm = (end_pos[2] - self.ground_level) * 1000
                    height_text = f"Height: {height_mm:.0f}mm above ground"
                    cv2.putText(display_image, height_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                # Current test pose
                if self.current_test_pose:
                    cv2.putText(display_image, f"Test Pose: {self.current_test_pose}", 
                               (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                
                # Joint states
                if self.current_joint_states and len(self.current_joint_states.position) >= 6:
                    joints = list(self.current_joint_states.position[:6])
                    joint_text = "Joints: " + ", ".join([f"J{i+1}:{math.degrees(j):.0f}°" for i, j in enumerate(joints)])
                    cv2.putText(display_image, joint_text, (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                
                # Movement history visualization
                if len(self.pose_history) > 1:
                    cv2.putText(display_image, f"History: {len(self.pose_history)} poses", 
                               (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 100), 1)
                    
                    # Show last movement distance
                    last_pos = self.pose_history[-1]['position']
                    prev_pos = self.pose_history[-2]['position']
                    last_distance = math.sqrt((last_pos[0] - prev_pos[0])**2 + 
                                            (last_pos[1] - prev_pos[1])**2 + 
                                            (last_pos[2] - prev_pos[2])**2)
                    cv2.putText(display_image, f"Last move: {last_distance*1000:.1f}mm", 
                               (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 255, 100), 1)
                
                # Workspace visualization
                img_height, img_width = display_image.shape[:2]
                
                # Draw coordinate system
                cv2.line(display_image, (img_width//2, 0), (img_width//2, img_height), (255, 0, 0), 1)  # Vertical center
                cv2.line(display_image, (0, img_height//2), (img_width, img_height//2), (0, 255, 0), 1)  # Horizontal center
                
                # Draw workspace boundaries
                workspace_color = (0, 255, 255)
                workspace_rect = (50, 250, img_width-100, img_height-300)
                cv2.rectangle(display_image, (workspace_rect[0], workspace_rect[1]), 
                             (workspace_rect[0] + workspace_rect[2], workspace_rect[1] + workspace_rect[3]), 
                             workspace_color, 2)
                cv2.putText(display_image, "WORKSPACE", (workspace_rect[0] + 5, workspace_rect[1] + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, workspace_color, 1)
                
                # Show selected target
                if self.selected_point is not None:
                    x, y = self.selected_point
                    cv2.circle(display_image, (x, y), 25, (0, 0, 255), 3)  # Red outer circle
                    cv2.circle(display_image, (x, y), 8, (0, 255, 0), -1)   # Green center
                    cv2.putText(display_image, "TARGET", (x + 30, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # Show target coordinates if available
                    if self.current_target:
                        target_text = f"({self.current_target[0]:.2f}, {self.current_target[1]:.2f}, {self.current_target[2]:.2f})"
                        cv2.putText(display_image, target_text, (x + 30, y + 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                
                # Status overlays
                if self.movement_in_progress:
                    cv2.putText(display_image, "EXECUTING MOTION...", 
                               (10, img_height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                if self.motion_sequence_active:
                    cv2.putText(display_image, "PICKUP SEQUENCE ACTIVE", 
                               (10, img_height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                # Enhanced keyboard instructions
                instructions = [
                    "KEYBOARD CONTROLS:",
                    "1-3: Front row (left/center/right)",
                    "4-6: Middle row (left/center/right)",  
                    "7-9: Far row (left/center/right)",
                    "0: Elevated poses",
                    "h: Home, s: Scan, p: Demo",
                    "m: Toggle magnet, r: Reset, q: Quit",
                    "CLICK anywhere to pick!"
                ]
                
                start_y = img_height - 200
                for i, instruction in enumerate(instructions):
                    color = (255, 255, 255) if i == 0 else (200, 200, 200)
                    cv2.putText(display_image, instruction, (img_width - 300, start_y + i*20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
                
                cv2.imshow(window_name, display_image)
                
                # Enhanced keyboard handling
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif self.system_ready and not self.movement_in_progress:
                    self.handle_keyboard_input(key)
                    
            else:
                # Waiting screen with better visuals
                waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_image, "INITIALIZING DEBUG SYSTEM...", 
                           (150, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(waiting_image, "Waiting for camera and MoveIt...", 
                           (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(waiting_image, "DEBUG MODE - Extensive logging enabled", 
                           (120, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imshow(window_name, waiting_image)
                cv2.waitKey(100)
        
        cv2.destroyAllWindows()
    
    def handle_keyboard_input(self, key):
        """Handle keyboard inputs for test poses and commands"""
        pose_map = {
            ord('1'): 'front_left', ord('2'): 'front_center', ord('3'): 'front_right',
            ord('4'): 'mid_left', ord('5'): 'mid_center', ord('6'): 'mid_right',
            ord('7'): 'far_left', ord('8'): 'far_center', ord('9'): 'far_right',
            ord('0'): 'elevated_center'
        }
        
        if key in pose_map:
            pose_name = pose_map[key]
            self.current_test_pose = pose_name
            self.get_logger().info(f"🎯 Moving to test pose: {pose_name}")
            self.move_to_pose(self.predefined_poses[pose_name], f"TEST_{pose_name.upper()}")
            
        elif key == ord('h'):
            self.current_test_pose = "home"
            self.get_logger().info("🏠 Moving to home pose...")
            self.move_to_pose(self.predefined_poses['home'], "HOME")
            
        elif key == ord('s'):
            self.current_test_pose = "scan"
            self.get_logger().info("🔍 Moving to scan pose...")
            self.move_to_pose(self.predefined_poses['scan'], "SCAN")
            
        elif key == ord('m'):
            self.toggle_electromagnet()
            
        elif key == ord('p') and not self.motion_sequence_active:
            self.get_logger().info("🎯 Running demo sequence...")
            self.run_demo()
            
        elif key == ord('r'):
            self.reset_system()
            
        elif key == ord('t'):
            self.run_test_sequence()
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks with enhanced debugging"""
        if event == cv2.EVENT_LBUTTONDOWN and self.system_ready and not self.movement_in_progress and not self.motion_sequence_active:
            self.selected_point = (x, y)
            self.get_logger().info(f"🎯 PICKUP TARGET CLICKED: Pixel ({x}, {y})")
            
            ground_pos = self.pixel_to_ground_coordinates(x, y)
            if ground_pos:
                self.current_target = ground_pos
                self.get_logger().info(f"📍 GROUND PICKUP TARGET: ({ground_pos[0]:.3f}, {ground_pos[1]:.3f}, {ground_pos[2]:.3f})")
                self.start_moveit_ground_sequence(ground_pos)
    
    def start_moveit_ground_sequence(self, target_pos):
        """Start ground pickup sequence with proper state management"""
        if self.motion_sequence_active or self.movement_in_progress:
            self.get_logger().warn("⚠️ Motion already in progress!")
            return
            
        self.motion_sequence_active = True
        self.get_logger().info("🎭 POSITION-LOGGED PICKUP SEQUENCE STARTED!")
        self.get_logger().info(f"🎯 SEQUENCE TARGET: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        
        def sequence_thread():
            try:
                # Step 1: Approach pose
                approach_pose = self.create_approach_pose(target_pos)
                self.get_logger().info("🎯 STEP 1: Moving to approach position")
                if not self.move_to_pose(approach_pose, "APPROACH"):
                    raise Exception("Failed approach")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 2: Ground pickup pose
                pickup_pose = self.create_ground_pickup_pose(target_pos)
                self.get_logger().info("🎯 STEP 2: GROUND PICKUP - Moving to ground level")
                if not self.move_to_pose(pickup_pose, "GROUND_PICKUP"):
                    raise Exception("Failed ground pickup")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 3: Activate magnet
                self.get_logger().info("🧲 STEP 3: Activating electromagnet")
                self.activate_electromagnet()
                time.sleep(3.0)
                
                # Step 4: Lift back to approach
                self.get_logger().info("⬆️ STEP 4: Lifting object")
                if not self.move_to_pose(approach_pose, "LIFT"):
                    raise Exception("Failed lift")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 5: Drop location
                drop_pos = (0.25, -target_pos[1], target_pos[2])
                drop_pose = self.create_ground_pickup_pose(drop_pos)
                self.get_logger().info(f"🎯 STEP 5: Moving to drop location ({drop_pos[0]:.3f}, {drop_pos[1]:.3f}, {drop_pos[2]:.3f})")
                if not self.move_to_pose(drop_pose, "DROP_POSITION"):
                    raise Exception("Failed drop move")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                time.sleep(2.0)
                
                # Step 6: Release magnet
                self.get_logger().info("⚫ STEP 6: Releasing electromagnet")
                self.deactivate_electromagnet()
                time.sleep(2.0)
                
                # Step 7: Return home
                self.get_logger().info("🏠 STEP 7: Returning home")
                self.move_to_pose(self.predefined_poses['home'], "RETURN_HOME")
                
                while self.movement_in_progress:
                    time.sleep(0.1)
                
                self.get_logger().info("✅ DEBUG PICKUP SEQUENCE COMPLETED!")
                
            except Exception as e:
                self.get_logger().error(f"❌ PICKUP SEQUENCE ERROR: {e}")
                try:
                    self.deactivate_electromagnet()
                    self.move_to_pose(self.predefined_poses['home'], "EMERGENCY_HOME")
                except:
                    pass
            finally:
                self.motion_sequence_active = False
                self.selected_point = None
        
        sequence = threading.Thread(target=sequence_thread)
        sequence.daemon = True
        sequence.start()
    
    def run_demo(self):
        """Run demo with predefined target"""
        if self.movement_in_progress or self.motion_sequence_active:
            self.get_logger().warn("⚠️ Already in motion")
            return
        
        demo_pos = (0.30, -0.08, self.ground_level)
        self.current_target = demo_pos
        self.selected_point = (420, 350)
        
        self.get_logger().info(f"🎭 DEMO SEQUENCE at ({demo_pos[0]:.3f}, {demo_pos[1]:.3f}, {demo_pos[2]:.3f})")
        self.start_moveit_ground_sequence(demo_pos)
    
    def run_test_sequence(self):
        """Run through all test poses in sequence"""
        if self.movement_in_progress or self.motion_sequence_active:
            self.get_logger().warn("⚠️ Already in motion")
            return
        
        self.get_logger().info("🧪 RUNNING TEST SEQUENCE - All predefined poses")
        
        def test_thread():
            test_poses = ['home', 'front_center', 'mid_center', 'far_center', 'elevated_center', 'home']
            
            for pose_name in test_poses:
                if pose_name in self.predefined_poses:
                    self.current_test_pose = pose_name
                    self.get_logger().info(f"🧪 Testing pose: {pose_name}")
                    self.move_to_pose(self.predefined_poses[pose_name], f"TEST_{pose_name.upper()}")
                    
                    while self.movement_in_progress:
                        time.sleep(0.1)
                    time.sleep(3.0)  # Pause between poses
            
            self.get_logger().info("✅ TEST SEQUENCE COMPLETED!")
        
        test = threading.Thread(target=test_thread)
        test.daemon = True
        test.start()
    
    def reset_system(self):
        """Reset system state"""
        self.selected_point = None
        self.motion_sequence_active = False
        self.current_target = None
        self.current_test_pose = None
        if self.electromagnet_active:
            self.deactivate_electromagnet()
        self.get_logger().info("🔄 DEBUG SYSTEM RESET - All states cleared")
    
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