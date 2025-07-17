#!/usr/bin/env python3
# filepath: /home/anish/dp_ws/Thor-ROS/ws_thor/src/thor_manipulation/scripts/task_coordinator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
from enum import Enum, auto

class TaskState(Enum):
    IDLE = auto()
    SPAWNING = auto()
    DETECTING = auto()
    PICKING = auto()
    COMPLETED = auto()
    FAILED = auto()

class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        
        # State management
        self.current_state = TaskState.IDLE
        self.task_start_time = None
        
        # Publishers
        self.spawn_command_pub = self.create_publisher(String, '/spawn_command', 10)
        self.pick_place_command_pub = self.create_publisher(String, '/thor_arm/pick_place_command', 10)
        self.task_status_pub = self.create_publisher(String, '/task_coordinator_status', 10)
        
        # Subscribers
        self.spawn_status_sub = self.create_subscription(
            String, '/spawn_status', self.spawn_status_callback, 10)
        self.detection_status_sub = self.create_subscription(
            String, '/detection_status', self.detection_status_callback, 10)
        self.pick_place_status_sub = self.create_subscription(
            String, '/thor_arm/pick_place_status', self.pick_place_status_callback, 10)
        self.object_detection_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_detected_callback, 10)
        self.coordinator_command_sub = self.create_subscription(
            String, '/task_coordinator_command', self.coordinator_command_callback, 10)
        
        # Task tracking
        self.object_spawned = False
        self.object_detected = False
        self.task_completed = False
        
        self.get_logger().info("🎯 Task Coordinator initialized")
        self.get_logger().info("Commands: 'start_full_task', 'reset', 'status'")

    def coordinator_command_callback(self, msg):
        """Handle coordinator commands"""
        command = msg.data.lower()
        
        if command == 'start_full_task':
            self.start_full_task()
        elif command == 'reset':
            self.reset_task()
        elif command == 'status':
            self.publish_detailed_status()
        else:
            self.get_logger().warn(f"Unknown coordinator command: {command}")

    def start_full_task(self):
        """Start a complete pick and place task"""
        if self.current_state != TaskState.IDLE:
            self.get_logger().warn(f"Task already in progress (state: {self.current_state.name})")
            return
        
        self.get_logger().info("🚀 Starting full pick and place task...")
        self.task_start_time = time.time()
        self.reset_flags()
        
        # Start by spawning an object
        self.current_state = TaskState.SPAWNING
        self.publish_task_status("STARTING_FULL_TASK")
        
        # Clear existing objects first
        clear_msg = String()
        clear_msg.data = "clear_all"
        self.spawn_command_pub.publish(clear_msg)
        
        # Wait a moment then spawn new object
        self.create_timer(2.0, self.spawn_object_timer)

    def spawn_object_timer(self):
        """Timer callback to spawn object"""
        spawn_msg = String()
        spawn_msg.data = "spawn_can"
        self.spawn_command_pub.publish(spawn_msg)
        self.get_logger().info("📦 Spawning object...")

    def spawn_status_callback(self, msg):
        """Handle spawn status"""
        if self.current_state == TaskState.SPAWNING:
            if "SPAWNED" in msg.data:
                self.object_spawned = True
                self.get_logger().info("✅ Object spawned successfully")
                
                # Move to detection phase
                self.current_state = TaskState.DETECTING
                self.publish_task_status("OBJECT_SPAWNED_DETECTING")
                
                # Wait a moment then trigger detection
                self.create_timer(2.0, self.start_detection_timer)

    def start_detection_timer(self):
        """Timer callback to start detection"""
        detect_msg = String()
        detect_msg.data = "DETECT"
        self.pick_place_command_pub.publish(detect_msg)
        self.get_logger().info("🔍 Starting object detection...")

    def object_detected_callback(self, msg):
        """Handle object detection"""
        if self.current_state == TaskState.DETECTING:
            self.object_detected = True
            self.get_logger().info("📍 Object detected, starting pick sequence...")
            
            # Move to picking phase
            self.current_state = TaskState.PICKING
            self.publish_task_status("OBJECT_DETECTED_PICKING")
            
            # Start pick sequence
            pick_msg = String()
            pick_msg.data = "PICK"
            self.pick_place_command_pub.publish(pick_msg)

    def detection_status_callback(self, msg):
        """Handle detection status"""
        self.get_logger().info(f"🔍 Detection update: {msg.data}")

    def pick_place_status_callback(self, msg):
        """Handle pick and place status"""
        if self.current_state == TaskState.PICKING:
            if "TASK_COMPLETED" in msg.data:
                self.task_completed = True
                self.current_state = TaskState.COMPLETED
                
                elapsed_time = time.time() - self.task_start_time
                self.get_logger().info(f"🎉 Full task completed in {elapsed_time:.1f} seconds!")
                self.publish_task_status(f"FULL_TASK_COMPLETED_{elapsed_time:.1f}s")
                
                # Reset to idle after a delay
                self.create_timer(3.0, self.reset_to_idle_timer)
                
            elif "FAILED" in msg.data:
                self.current_state = TaskState.FAILED
                self.get_logger().error("❌ Pick and place task failed")
                self.publish_task_status("TASK_FAILED")
                
                # Reset to idle after a delay
                self.create_timer(5.0, self.reset_to_idle_timer)

    def reset_to_idle_timer(self):
        """Timer callback to reset to idle state"""
        self.current_state = TaskState.IDLE
        self.get_logger().info("🏠 Task coordinator ready for next task")

    def reset_task(self):
        """Reset task state"""
        self.current_state = TaskState.IDLE
        self.reset_flags()
        self.get_logger().info("🔄 Task coordinator reset")
        self.publish_task_status("RESET")

    def reset_flags(self):
        """Reset all task flags"""
        self.object_spawned = False
        self.object_detected = False
        self.task_completed = False
        self.task_start_time = None

    def publish_task_status(self, status):
        """Publish task status"""
        msg = String()
        msg.data = status
        self.task_status_pub.publish(msg)

    def publish_detailed_status(self):
        """Publish detailed status"""
        elapsed = time.time() - self.task_start_time if self.task_start_time else 0
        
        status_text = f"""
        🎯 Task Coordinator Status
        =========================
        Current State: {self.current_state.name}
        Object Spawned: {self.object_spawned}
        Object Detected: {self.object_detected}
        Task Completed: {self.task_completed}
        Elapsed Time: {elapsed:.1f}s
        """
        
        self.get_logger().info(status_text)
        self.publish_task_status(f"STATUS_{self.current_state.name}")

def main(args=None):
    rclpy.init(args=args)
    
    node = TaskCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()