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
        
        # Action clients - Using the correct action from your topic list
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
        
        # Predefined positions (adjust these based on your robot's configuration)
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.scan_position = [0.0, -0.5, 0.3, 0.0, 0.2, 0.0]
        
        # Camera and workspace parameters (adjust based on your setup)
        self.image_width = 640
        self.image_height = 480
        self.workspace_x_min = -0.5  # meters
        self.workspace_x_max = 0.5
        self.workspace_y_min = -0.5
        self.workspace_y_max = 0.5
        self.workspace_z = 0.1  # Height above table/ground for picking
        
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
        
        if self.debug:
            self.get_logger().info(f"📊 System Status - Image: {image_ready}, Joints: {joints_ready}, Action: {action_ready}")
            
            # Debug joint states issue
            if not joints_ready:
                self.get_logger().info("🔍 Checking joint states topic...")
                # Let's force check what's available
                
            # Debug action server issue  
            if not action_ready:
                self.get_logger().info("🔍 Action server not ready, checking available actions...")
        
        # Make system ready if at least image is available - we'll handle other issues
        if image_ready and not self.system_ready:
            self.system_ready = True
            self.get_logger().info("✅ System Ready with Image! (Partial readiness)")
            if joints_ready and action_ready:
                self.get_logger().info("✅ Full System Ready! Moving to scan position...")
                self.move_to_scan_position()
    
    def image_callback(self, msg):
        """Store latest image and log receipt"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.debug and not hasattr(self, '_image_logged'):
                self._image_logged = True
                self.get_logger().info(f"📷 Camera image received: {msg.width}x{msg.height}")
        except Exception as e:
            self.get_logger().error(f"❌ Image conversion error: {e}")
    
    def joint_states_callback(self, msg):
        """Store joint states and log receipt"""
        self.current_joint_states = msg
        if self.debug and not hasattr(self, '_joints_logged'):
            self._joints_logged = True
            joint_info = ", ".join([f"{name}: {pos:.2f}" for name, pos in zip(msg.name[:6], msg.position[:6])])
            self.get_logger().info(f"🤖 Joint states received: {joint_info}")
    
    def pixel_to_workspace(self, pixel_x, pixel_y):
        """Convert pixel coordinates to workspace coordinates"""
        if self.current_image is None:
            return None
        
        # Get actual image dimensions
        img_height, img_width = self.current_image.shape[:2]
        
        # Normalize pixel coordinates to [0, 1]
        norm_x = pixel_x / img_width
        norm_y = pixel_y / img_height
        
        # Map to workspace coordinates
        world_x = self.workspace_x_min + norm_x * (self.workspace_x_max - self.workspace_x_min)
        world_y = self.workspace_y_min + norm_y * (self.workspace_y_max - self.workspace_y_min)
        world_z = self.workspace_z
        
        self.get_logger().info(f"🎯 Pixel ({pixel_x}, {pixel_y}) -> World ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        
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
                cv2.putText(display_image, "Controls: h=home, s=scan, m=magnet, r=reset, q=quit", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Show workspace boundaries
                img_height, img_width = display_image.shape[:2]
                cv2.rectangle(display_image, (10, 10), (img_width-10, img_height-10), (255, 255, 0), 2)
                cv2.putText(display_image, "Workspace", (20, img_height-20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # Show selected point
                if self.selected_point is not None:
                    x, y = self.selected_point
                    cv2.circle(display_image, (x, y), 20, (0, 0, 255), 3)
                    cv2.putText(display_image, "TARGET", (x+25, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Show movement status
                if self.movement_in_progress:
                    cv2.putText(display_image, "MOVING...", 
                               (10, display_image.shape[0] - 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                
                cv2.imshow(window_name, display_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h') and self.system_ready:
                    self.get_logger().info("🏠 Moving to home position...")
                    self.move_to_joint_positions(self.home_position)
                elif key == ord('s') and self.system_ready:
                    self.get_logger().info("🔍 Moving to scan position...")
                    self.move_to_joint_positions(self.scan_position)
                elif key == ord('m'):
                    self.toggle_electromagnet()
                elif key == ord('r'):
                    self.selected_point = None
                    self.get_logger().info("🔄 Reset target selection")
                    
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
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_point = (x, y)
            self.get_logger().info(f"🎯 OBJECT CLICKED at pixel: ({x}, {y})")
            
            # Convert pixel to workspace coordinates
            world_pos = self.pixel_to_workspace(x, y)
            if world_pos:
                self.get_logger().info(f"📍 Target workspace position: ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
                
                # Check system readiness but proceed anyway for debugging
                if not self.system_ready:
                    self.get_logger().warn("⚠️ System not fully ready, but attempting motion anyway...")
                
                if self.movement_in_progress:
                    self.get_logger().warn("⚠️ Movement already in progress, ignoring click")
                    return
                
                # Trigger motion planning and execution
                self.get_logger().info("🚀 TRIGGERING MOTION PLANNING AND EXECUTION")
                self.plan_and_execute_motion(world_pos)
    
    def plan_and_execute_motion(self, target_pos):
        """Plan and execute motion to target position"""
        self.get_logger().info(f"🎯 MOTION PLANNING: Target = {target_pos}")
        
        # Check if we have joint states
        if self.current_joint_states is None:
            self.get_logger().error("❌ No joint states available - checking topic...")
            # Try to get joint states manually
            self.check_joint_states_topic()
            return
        
        # Check if action server is available
        if not self.joint_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Joint trajectory action server not available")
            self.try_alternative_motion_control(target_pos)
            return
        
        self.get_logger().info("✅ Prerequisites met, executing motion...")
        self.execute_planned_motion(target_pos)
    
    def check_joint_states_topic(self):
        """Debug joint states topic"""
        self.get_logger().info("🔍 Debugging joint states...")
        # Force create a new subscription to debug
        def debug_joint_callback(msg):
            self.get_logger().info(f"📍 DEBUG: Received {len(msg.name)} joints: {msg.name[:6]}")
            self.current_joint_states = msg
        
        debug_sub = self.create_subscription(JointState, '/joint_states', debug_joint_callback, 10)
        self.get_logger().info("🔍 Created debug joint states subscription")
    
    def try_alternative_motion_control(self, target_pos):
        """Try alternative motion control methods"""
        self.get_logger().info("🔄 Trying alternative motion control...")
        
        # Try using execute_trajectory action instead
        try:
            from moveit_msgs.action import ExecuteTrajectory
            self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
            if self.execute_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("✅ Found /execute_trajectory action server")
                self.simple_reach_target(target_pos)
            else:
                self.get_logger().warn("⚠️ No trajectory execution available - simulating motion")
                self.simulate_motion(target_pos)
        except ImportError:
            self.get_logger().warn("⚠️ MoveIt not available - simulating motion")
            self.simulate_motion(target_pos)
    
    def simple_reach_target(self, target_pos):
        """Simple reach target using available action server"""
        self.get_logger().info(f"🎯 Simple reach to: {target_pos}")
        self.execute_planned_motion(target_pos)
    
    def simulate_motion(self, target_pos):
        """Simulate motion for debugging"""
        self.movement_in_progress = True
        self.get_logger().info(f"🎭 SIMULATING motion to {target_pos}")
        
        # Simulate motion sequence
        def motion_simulation():
            self.get_logger().info("🚀 Step 1: Planning motion...")
            time.sleep(1)
            self.get_logger().info("🤖 Step 2: Executing motion...")
            time.sleep(2)
            self.get_logger().info("📍 Step 3: Reached target position!")
            time.sleep(1)
            self.get_logger().info("✅ Motion simulation completed!")
            self.movement_in_progress = False
        
        # Run simulation in thread
        sim_thread = threading.Thread(target=motion_simulation)
        sim_thread.daemon = True
        sim_thread.start()
    
    def execute_planned_motion(self, target_pos):
        """Execute the actual planned motion"""
        if self.current_joint_states and len(self.current_joint_states.position) >= 6:
            current_joints = list(self.current_joint_states.position[:6])
            self.get_logger().info(f"📍 Current joints: {[f'{j:.2f}' for j in current_joints]}")
            
            # Simple mapping: modify joints based on target position
            target_joints = current_joints.copy()
            target_joints[0] = math.atan2(target_pos[1], target_pos[0])  # Base rotation
            target_joints[1] = max(-1.5, min(1.5, current_joints[1] - 0.2))  # Shoulder adjustment
            target_joints[2] = max(-1.5, min(1.5, current_joints[2] + 0.3))   # Elbow adjustment
            
            self.get_logger().info(f"🎯 Target joints: {[f'{j:.2f}' for j in target_joints]}")
            self.move_to_joint_positions(target_joints)
        else:
            self.get_logger().error("❌ Invalid joint states for motion planning")
            self.simulate_motion(target_pos)
    
    def move_to_scan_position(self):
        """Move to scan position"""
        if self.system_ready and not self.movement_in_progress:
            self.get_logger().info("🔍 Moving to scan position...")
            self.move_to_joint_positions(self.scan_position)
    
    def move_to_joint_positions(self, joint_positions):
        """Move to specified joint positions using FollowJointTrajectory action"""
        if self.movement_in_progress:
            self.get_logger().warn("⚠️ Movement already in progress")
            return
        
        self.movement_in_progress = True
        self.get_logger().info(f"🤖 EXECUTING MOTION to joints: {[f'{j:.2f}' for j in joint_positions]}")
        
        try:
            # Check action server availability
            if not self.joint_trajectory_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ Joint trajectory action server not available!")
                self.get_logger().info("🔍 Available actions:")
                # List available actions for debugging
                import subprocess
                result = subprocess.run(['ros2', 'action', 'list'], capture_output=True, text=True)
                self.get_logger().info(f"Available actions: {result.stdout}")
                self.movement_in_progress = False
                return
            
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0] * len(joint_positions)
            point.accelerations = [0.0] * len(joint_positions)
            point.time_from_start.sec = 5  # 5 seconds to reach position
            point.time_from_start.nanosec = 0
            
            trajectory.points = [point]
            
            # Create and send goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            self.get_logger().info("📤 Sending trajectory goal to action server...")
            
            # Send goal and wait for result
            future = self.joint_trajectory_client.send_goal_async(goal)
            
            def goal_response_callback(future):
                try:
                    goal_handle = future.result()
                    if not goal_handle.accepted:
                        self.get_logger().error("❌ Trajectory goal REJECTED by action server")
                        self.movement_in_progress = False
                        return
                    
                    self.get_logger().info("✅ Trajectory goal ACCEPTED by action server")
                    
                    # Get result
                    result_future = goal_handle.get_result_async()
                    result_future.add_done_callback(self.trajectory_result_callback)
                    
                except Exception as e:
                    self.get_logger().error(f"❌ Goal response error: {e}")
                    self.movement_in_progress = False
            
            future.add_done_callback(goal_response_callback)
            
        except Exception as e:
            self.get_logger().error(f"❌ Trajectory execution error: {e}")
            self.movement_in_progress = False
    
    def trajectory_result_callback(self, future):
        """Handle trajectory execution result"""
        try:
            result = future.result()
            if result.result.error_code == 0:  # Assuming 0 is success
                self.get_logger().info("✅ Trajectory execution completed successfully")
            else:
                self.get_logger().error(f"❌ Trajectory execution failed: {result.result.error_code}")
        except Exception as e:
            self.get_logger().error(f"❌ Trajectory result error: {e}")
        finally:
            self.movement_in_progress = False
    
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