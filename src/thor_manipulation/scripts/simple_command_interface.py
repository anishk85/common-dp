#!/usr/bin/env python3
# filepath: /home/anish/dp_ws/Thor-ROS/ws_thor/src/thor_manipulation/scripts/simple_command_interface.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import time

class SimpleCommandInterface(Node):
    def __init__(self):
        super().__init__('simple_command_interface')
        
        # Publishers for various commands
        self.spawn_command_pub = self.create_publisher(String, '/spawn_command', 10)
        self.user_command_pub = self.create_publisher(String, '/user_command', 10)
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        
        # Subscribers for status monitoring
        self.spawn_status_sub = self.create_subscription(
            String, '/spawn_status', self.spawn_status_callback, 10)
        self.pick_place_status_sub = self.create_subscription(
            String, '/thor_arm/pick_place_status', self.pick_place_status_callback, 10)
        self.object_detection_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_detected_callback, 10)
        
        # User command subscriber
        self.user_input_sub = self.create_subscription(
            String, '/simple_command', self.user_input_callback, 10)
        
        # State tracking
        self.last_spawn_status = None
        self.last_pick_place_status = None
        self.object_detected = False
        self.object_position = None
        
        self.get_logger().info("🎮 Simple Command Interface initialized")
        self.print_help()
        
        # Create a timer for periodic status updates
        self.create_timer(10.0, self.status_timer_callback)

    def print_help(self):
        """Print available commands"""
        help_text = """
        🎮 Simple Thor Manipulation Control Interface
        ============================================
        
        Available Commands (publish to /simple_command):
        
        📦 Object Management:
        - spawn_can      : Spawn a red can
        - spawn_box      : Spawn a green box  
        - spawn_sphere   : Spawn a blue sphere
        - clear_all      : Remove all objects
        
        🔍 Detection & Manipulation:
        - detect         : Start camera-based object detection
        - stop_detect    : Stop object detection
        - pick           : Pick detected object
        - place          : Place object at predefined location
        - home           : Move robot to home position
        
        🧲 Manual Controls:
        - magnet_on      : Activate electromagnet
        - magnet_off     : Deactivate electromagnet
        
        📊 System:
        - status         : Show current system status
        - help           : Show this help message
        - demo           : Run full demo sequence
        
        Example usage:
        ros2 topic pub /simple_command std_msgs/String "data: spawn_can" --once
        ros2 topic pub /simple_command std_msgs/String "data: detect" --once
        ros2 topic pub /simple_command std_msgs/String "data: pick" --once
        """
        self.get_logger().info(help_text)

    def user_input_callback(self, msg):
        """Handle user commands"""
        command = msg.data.lower().strip()
        
        self.get_logger().info(f"🎮 Received command: {command}")
        
        # Object spawning commands
        if command in ['spawn_can', 'spawn_box', 'spawn_sphere', 'clear_all']:
            self.send_spawn_command(command)
        
        # Detection and manipulation commands
        elif command == 'detect':
            self.send_user_command('detect')
        elif command == 'stop_detect':
            self.send_user_command('stop')
        elif command == 'pick':
            self.send_user_command('pick')
        elif command == 'place':
            self.send_user_command('place')
        elif command == 'home':
            self.send_user_command('home')
        
        # Electromagnet commands
        elif command == 'magnet_on':
            self.control_electromagnet(True)
        elif command == 'magnet_off':
            self.control_electromagnet(False)
        
        # System commands
        elif command == 'status':
            self.print_status()
        elif command == 'help':
            self.print_help()
        
        # Demo sequence
        elif command == 'demo':
            self.run_demo_sequence()
        
        else:
            self.get_logger().warn(f"❌ Unknown command: {command}")
            self.get_logger().info("💡 Type 'help' for available commands")

    def send_spawn_command(self, command):
        """Send spawn command"""
        msg = String()
        msg.data = command
        self.spawn_command_pub.publish(msg)
        self.get_logger().info(f"📦 Sent spawn command: {command}")

    def send_user_command(self, command):
        """Send user command to enhanced pick place node"""
        msg = String()
        msg.data = command
        self.user_command_pub.publish(msg)
        self.get_logger().info(f"🤖 Sent user command: {command}")

    def control_electromagnet(self, activate):
        """Control electromagnet"""
        msg = Bool()
        msg.data = activate
        self.electromagnet_pub.publish(msg)
        action = "activated" if activate else "deactivated"
        self.get_logger().info(f"🧲 Electromagnet {action}")

    def run_demo_sequence(self):
        """Run a complete demo sequence"""
        self.get_logger().info("🎬 Starting demo sequence...")
        
        # Step 1: Clear any existing objects
        self.send_spawn_command("clear_all")
        self.get_logger().info("🎬 Step 1: Clearing existing objects...")
        
        # Create a timer chain for the demo
        self.create_timer(2.0, self.demo_step_2)

    def demo_step_2(self):
        """Demo step 2: Spawn object"""
        self.send_spawn_command("spawn_can")
        self.get_logger().info("🎬 Step 2: Spawning red can...")
        self.create_timer(3.0, self.demo_step_3)

    def demo_step_3(self):
        """Demo step 3: Move to home"""
        self.send_user_command("home")
        self.get_logger().info("🎬 Step 3: Moving to home position...")
        self.create_timer(5.0, self.demo_step_4)

    def demo_step_4(self):
        """Demo step 4: Start detection"""
        self.send_user_command("detect")
        self.get_logger().info("🎬 Step 4: Starting object detection...")
        self.create_timer(3.0, self.demo_step_5)

    def demo_step_5(self):
        """Demo step 5: Wait and check if object detected"""
        if self.object_detected:
            self.send_user_command("pick")
            self.get_logger().info("🎬 Step 5: Starting pick sequence...")
        else:
            self.get_logger().warn("🎬 Demo aborted: No object detected")

    def spawn_status_callback(self, msg):
        """Handle spawn status updates"""
        self.last_spawn_status = msg.data
        self.get_logger().info(f"📦 Spawn status: {msg.data}")

    def pick_place_status_callback(self, msg):
        """Handle pick and place status updates"""
        self.last_pick_place_status = msg.data
        self.get_logger().info(f"🤖 Pick/Place status: {msg.data}")
        
        # Auto-continue demo if task completed
        if "TASK_COMPLETED" in msg.data:
            self.get_logger().info("🎉 Demo sequence completed successfully!")

    def object_detected_callback(self, msg):
        """Handle object detection"""
        self.object_detected = True
        self.object_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.get_logger().info(f"📍 Object detected at: "
                              f"x={msg.pose.position.x:.3f}, "
                              f"y={msg.pose.position.y:.3f}, "
                              f"z={msg.pose.position.z:.3f}")

    def print_status(self):
        """Print current system status"""
        status_text = f"""
        📊 Simple Interface Status Report
        ================================
        
        📦 Last Spawn Status: {self.last_spawn_status or 'No status'}
        🤖 Last Pick/Place Status: {self.last_pick_place_status or 'No status'}
        📍 Object Detected: {'Yes' if self.object_detected else 'No'}
        📍 Object Position: {self.object_position if self.object_position else 'None'}
        
        ⏰ Timestamp: {time.strftime('%H:%M:%S')}
        
        💡 Quick Commands:
        - ros2 topic pub /simple_command std_msgs/String "data: demo" --once
        - ros2 topic pub /simple_command std_msgs/String "data: spawn_can" --once
        - ros2 topic pub /simple_command std_msgs/String "data: detect" --once
        """
        self.get_logger().info(status_text)

    def status_timer_callback(self):
        """Periodic status reminder"""
        if not self.object_detected and not self.last_pick_place_status:
            self.get_logger().info("💡 Ready for commands! Try: ros2 topic pub /simple_command std_msgs/String \"data: help\" --once")

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleCommandInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Simple Command Interface shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()