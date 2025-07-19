#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading

class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        
        # Publishers
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        self.status_pub = self.create_publisher(Bool, '/electromagnet_status', 10)
        
        # Action client
        self.joint_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.detected_objects_sub = self.create_subscription(
            PoseStamped, '/perception/detected_objects', self.detected_objects_callback, 10)
        
        # State variables
        self.current_joint_states = None
        self.detected_objects = []
        self.electromagnet_active = False
        self.task_running = False
        self.sequence_step = 0
        self.movement_in_progress = False
        
        # IMPROVED: Better joint positions for actual robot workspace
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # FIXED: More realistic joint positions (in radians)
        # self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.scan_position = [0.0, -0.5, 0.3, 0.0, 0.2, 0.0]      # Look down at workspace
        # self.pickup_position = [0.0, -0.8, 0.8, 0.0, 0.0, 0.0]    # Lower to pickup
        # self.place_position = [1.2, -0.8, 0.8, 0.0, 0.0, 0.0]     # Move to place location
         self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.scan_position = [0.0, -1.0, 0.8, 0.0, 0.5, 0.0]      # Better scan position
        self.pickup_position = [0.0, -1.3, 1.2, 0.0, 0.8, 0.0]    # Lower for pickup
        self.place_position = [1.5, -1.3, 1.2, 0.0, 0.8, 0.0]     # Move to place
        # Status timer
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Wait for services
        self.create_timer(2.0, self.start_sequence)
        
        self.get_logger().info("🤖 Pick and Place Controller initialized")
    
    def joint_states_callback(self, msg):
        self.current_joint_states = msg
    
    def detected_objects_callback(self, msg):
        # Only store recent objects
        self.detected_objects.append(msg)
        if len(self.detected_objects) > 10:
            self.detected_objects.pop(0)
    
    def publish_status(self):
        msg = Bool()
        msg.data = self.electromagnet_active
        self.status_pub.publish(msg)
    
    def start_sequence(self):
        """Start the pick and place sequence"""
        if self.task_running or self.movement_in_progress:
            return
            
        if not self.joint_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Waiting for action server...")
            return
            
        if self.current_joint_states is None:
            self.get_logger().warn("⚠️ No joint states received yet...")
            return
        
        self.get_logger().info("🚀 Starting pick and place sequence")
        self.task_running = True
        self.sequence_step = 0
        self.execute_next_step()
    
    def execute_next_step(self):
        """Execute the next step in the sequence"""
        if not self.task_running or self.movement_in_progress:
            return
            
        if self.sequence_step == 0:
            self.get_logger().info("🔍 Step 1: Moving to scan position...")
            self.move_to_position(self.scan_position)
            
        elif self.sequence_step == 1:
            self.get_logger().info("📷 Step 2: Scanning for objects...")
            self.scan_for_objects()
            
        elif self.sequence_step == 2:
            self.get_logger().info("📍 Step 3: Moving to pickup position...")
            self.move_to_position(self.pickup_position)
            
        elif self.sequence_step == 3:
            self.get_logger().info("🧲 Step 4: Activating electromagnet...")
            self.activate_electromagnet()
            self.create_timer(2.0, self.timer_callback_once)  # Wait 2 seconds
            
        elif self.sequence_step == 4:
            self.get_logger().info("📦 Step 5: Moving to place position...")
            self.move_to_position(self.place_position)
            
        elif self.sequence_step == 5:
            self.get_logger().info("🔓 Step 6: Releasing object...")
            self.deactivate_electromagnet()
            self.create_timer(2.0, self.timer_callback_once)  # Wait 2 seconds
            
        elif self.sequence_step == 6:
            self.get_logger().info("🏠 Step 7: Returning home...")
            self.move_to_position(self.home_position)
            
        elif self.sequence_step == 7:
            self.get_logger().info("✅ Sequence complete! Restarting in 5 seconds...")
            self.task_running = False
            self.sequence_step = 0
            self.detected_objects.clear()
            self.create_timer(5.0, self.timer_callback_once)  # Restart after 5 seconds
    
    def timer_callback_once(self):
        """Single-use timer callback"""
        if not self.task_running:
            # Restart sequence
            self.start_sequence()
        else:
            # Continue to next step
            self.sequence_step += 1
            self.execute_next_step()
    
    def scan_for_objects(self):
        """Scan for objects and decide next action"""
        if len(self.detected_objects) > 0:
            self.get_logger().info(f"🎯 Found {len(self.detected_objects)} objects!")
            self.sequence_step += 1
            self.execute_next_step()
        else:
            self.get_logger().info("⚠️ No objects found, waiting 3 seconds...")
            self.create_timer(3.0, self.timer_callback_once)
    
    def move_to_position(self, target_positions):
        """Move robot to target position"""
        if self.movement_in_progress:
            self.get_logger().warn("⚠️ Movement already in progress!")
            return
            
        self.movement_in_progress = True
        
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 5  # 5 seconds for movement
        point.time_from_start.nanosec = 0
        
        goal.trajectory.points = [point]
        
        self.get_logger().info(f"🎯 Moving to: {[f'{pos:.2f}' for pos in target_positions]}")
        
        # Send goal
        future = self.joint_trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected!")
            self.movement_in_progress = False
            return
            
        self.get_logger().info("✅ Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle movement result"""
        self.movement_in_progress = False
        result = future.result().result
        
        if result.error_code == 0:
            self.get_logger().info("✅ Movement completed successfully!")
            self.sequence_step += 1
            # Wait a bit before next step
            self.create_timer(1.0, self.timer_callback_once)
        else:
            self.get_logger().error(f"❌ Movement failed with error: {result.error_code}")
            self.task_running = False
    
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

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()