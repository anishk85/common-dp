#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions, 
    Constraints, 
    JointConstraint,
    MoveItErrorCodes
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time
import math

class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        
        # Publishers
        self.electromagnet_pub = self.create_publisher(
            Bool,
            '/electromagnet_control',
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            Bool,
            '/electromagnet_status',
            10
        )
        
        # MoveIt action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )
        
        # Direct joint trajectory client as backup
        self.joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory'
        )
        
        # Subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # State variables
        self.electromagnet_active = False
        self.current_task = "idle"
        self.current_joint_states = None
        self.task_timer = None
        
        # FIXED: Joint names in correct order from your joint states
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_4', 'joint_5', 'joint_3', 'joint_6'
        ]
        
        # CORRECTED: Joint positions in the right order
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pickup_position = [0.0, -0.5, 0.0, 0.7, 0.8, 0.0]  # [j1, j2, j4, j5, j3, j6]
        self.place_position = [0.8, -0.5, 0.0, 0.7, 0.8, 0.0]   # [j1, j2, j4, j5, j3, j6]
        
        # Timer for status publishing
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        # Wait for action servers
        self.get_logger().info("🤖 Pick and place controller initialized")
        self.get_logger().info("📡 Waiting for action servers...")
        
        # Wait for MoveIt action server
        self.create_timer(1.0, self.check_action_servers)
        
        # Start the main sequence
        self.sequence_timer = self.create_timer(8.0, self.start_sequence)
        self.sequence_started = False
    
    def check_action_servers(self):
        """Check if action servers are available"""
        if self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("✅ MoveIt action server connected")
        else:
            self.get_logger().warn("⚠️ MoveIt action server not available")
            
        if self.joint_trajectory_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("✅ Joint trajectory action server connected")
        else:
            self.get_logger().warn("⚠️ Joint trajectory action server not available")
    
    def start_sequence(self):
        """Start the pick and place sequence"""
        if not self.sequence_started:
            self.sequence_started = True
            self.sequence_timer.cancel()
            self.execute_pick_place_sequence()
    
    def joint_states_callback(self, msg):
        """Store current joint states"""
        self.current_joint_states = msg
        if hasattr(self, 'first_joint_state'):
            return
        self.first_joint_state = True
        self.get_logger().info(f"📊 Received joint states: {msg.name}")
    
    def publish_status(self):
        """Publish current electromagnet status"""
        status_msg = Bool()
        status_msg.data = self.electromagnet_active
        self.status_pub.publish(status_msg)
    
    def execute_pick_place_sequence(self):
        """Execute pick and place sequence"""
        if self.current_joint_states is None:
            self.get_logger().warn("⚠️ No joint states received yet...")
            self.schedule_next_task("idle", 2.0)
            return
        
        if self.current_task == "idle":
            self.get_logger().info("🚀 Starting pick and place sequence...")
            self.current_task = "moving_to_pickup"
            self.move_to_joint_position_direct(self.pickup_position, "pickup")
            
        elif self.current_task == "picking":
            self.get_logger().info("🧲 Activating electromagnet...")
            self.activate_electromagnet()
            self.schedule_next_task("moving_to_place", 3.0)
            
        elif self.current_task == "moving_to_place":
            self.get_logger().info("📦 Moving to place position...")
            self.move_to_joint_position_direct(self.place_position, "place")
            
        elif self.current_task == "placing":
            self.get_logger().info("🔓 Releasing object...")
            self.deactivate_electromagnet()
            self.schedule_next_task("returning_home", 3.0)
            
        elif self.current_task == "returning_home":
            self.get_logger().info("🏠 Returning to home position...")
            self.move_to_joint_position_direct(self.home_position, "complete")
            
        elif self.current_task == "complete":
            self.get_logger().info("✅ Pick and place sequence complete!")
            self.schedule_next_task("idle", 5.0)
    
    def schedule_next_task(self, next_task, delay):
        """Schedule the next task after a delay"""
        if self.task_timer is not None:
            self.task_timer.cancel()
        
        def set_task():
            self.current_task = next_task
            self.task_timer = None
            self.execute_pick_place_sequence()
            
        self.task_timer = self.create_timer(delay, set_task)
    
    def move_to_joint_position_direct(self, target_positions, next_task):
        """Move using direct joint trajectory action"""
        if len(target_positions) != len(self.joint_names):
            self.get_logger().error("❌ Joint position array length mismatch!")
            return
        
        # Create joint trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 5  # 5 seconds to reach target
        point.time_from_start.nanosec = 0
        
        goal.trajectory.points = [point]
        
        # Send goal
        self.get_logger().info(f"🎯 Moving to {next_task} position: {target_positions}")
        
        if self.joint_trajectory_client.wait_for_server(timeout_sec=1.0):
            future = self.joint_trajectory_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.joint_trajectory_response(f, next_task))
        else:
            self.get_logger().error("❌ Joint trajectory action server not available")
            self.schedule_next_task(self.get_next_task(next_task), 3.0)
    
    def joint_trajectory_response(self, future, next_task):
        """Handle joint trajectory response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Joint trajectory goal rejected")
            self.schedule_next_task(self.get_next_task(next_task), 3.0)
            return
        
        self.get_logger().info("✅ Joint trajectory goal accepted")
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.joint_trajectory_result(f, next_task))
    
    def joint_trajectory_result(self, future, next_task):
        """Handle joint trajectory result"""
        result = future.result().result
        if result.error_code == 0:  # Success
            self.get_logger().info("✅ Movement completed successfully")
        else:
            self.get_logger().warn(f"⚠️ Movement completed with error: {result.error_code}")
        
        # Schedule next task
        self.schedule_next_task(self.get_next_task(next_task), 2.0)
    
    def get_next_task(self, task_type):
        """Get the next task in sequence"""
        if task_type == "pickup":
            return "picking"
        elif task_type == "place":
            return "placing"
        elif task_type == "complete":
            return "complete"
        return "idle"
    
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