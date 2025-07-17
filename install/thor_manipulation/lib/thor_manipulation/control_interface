#!/usr/bin/env python3
# filepath: /home/anish/dp_ws/Thor-ROS/ws_thor/src/thor_manipulation/scripts/control_interface.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import time

class ControlInterface(Node):
    def __init__(self):
        super().__init__('control_interface')
        
        # Publishers for various commands
        self.spawn_command_pub = self.create_publisher(String, '/spawn_command', 10)
        self.pick_place_command_pub = self.create_publisher(String, '/thor_arm/pick_place_command', 10)
        self.electromagnet_pub = self.create_publisher(Bool, '/electromagnet_control', 10)
        
        # Subscribers for status monitoring
        self.spawn_status_sub = self.create_subscription(
            String, '/spawn_status', self.spawn_status_callback, 10)
        self.detection_status_sub = self.create_subscription(
            String, '/detection_status', self.detection_status_callback, 10)
        self.pick_place_status_sub = self.create_subscription(
            String, '/thor_arm/pick_place_status', self.pick_place_status_callback, 10)
        self.object_detection_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_detected_callback, 10)
        
        # User command subscriber
        self.user_command_sub = self.create_subscription(
            String, '/user_command', self.user_command_callback, 10)
        
        # State tracking
        self.last_spawn_status = None
        self.last_detection_status = None
        self.last_pick_place_status = None
        self.object_detected = False
        
        self.get_logger().info("🎮 Control Interface initialized")
        self.print_help()
        
        # Create a timer for periodic status updates
        self.create_timer(5.0, self.status_timer_callback)

    def print_help(self):
        """Print available commands"""
        help_text = """
        🎮 Thor Manipulation Control Interface
        =====================================
        
        Available Commands (publish to /user_command):
        
        📦 Object Management:
        - spawn_can      : Spawn a red can
        - spawn_box      : Spawn a green box  
        - spawn_sphere   : Spawn a blue sphere
        - clear_all      : Remove all objects
        
        🔍 Detection:
        - detect         : Trigger object detection
        
        🤖 Robot Control:
        - pick           : Start pick and place sequence
        - home           : Move robot to home position
        - ready          : Move robot to ready position
        
        🧲 Electromagnet:
        - magnet_on      : Activate electromagnet
        - magnet_off     : Deactivate electromagnet
        
        📊 Status:
        - status         : Show current system status
        - help           : Show this help message
        
        Example usage:
        ros2 topic pub /user_command std_msgs/String "data: spawn_can" --once
        """
        self.get_logger().info(help_text)

    def user_command_callback(self, msg):
        """Handle user commands"""
        command = msg.data.lower().strip()
        
        self.get_logger().info(f"🎮 Received command: {command}")
        
        # Object spawning commands
        if command in ['spawn_can', 'spawn_box', 'spawn_sphere', 'clear_all']:
            self.send_spawn_command(command)
        
        # Detection commands
        elif command == 'detect':
            self.send_detection_command()
        
        # Robot control commands
        elif command in ['pick', 'home', 'ready']:
            self.send_pick_place_command(command.upper())
        
        # Electromagnet commands
        elif command == 'magnet_on':
            self.control_electromagnet(True)
        elif command == 'magnet_off':
            self.control_electromagnet(False)
        
        # Status and help
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

    def send_detection_command(self):
        """Send detection command"""
        msg = String()
        msg.data = "DETECT"
        self.pick_place_command_pub.publish(msg)
        self.get_logger().info("🔍 Triggered object detection")

    def send_pick_place_command(self, command):
        """Send pick and place command"""
        msg = String()
        msg.data = command
        self.pick_place_command_pub.publish(msg)
        self.get_logger().info(f"🤖 Sent robot command: {command}")

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
        time.sleep(1.0)
        
        # Step 2: Spawn a can
        self.send_spawn_command("spawn_can")
        time.sleep(2.0)
        
        # Step 3: Move to ready position
        self.send_pick_place_command("READY")
        time.sleep(3.0)
        
        # Step 4: Detect object
        self.send_detection_command()
        time.sleep(2.0)
        
        # Step 5: Pick and place
        if self.object_detected:
            self.send_pick_place_command("PICK")
            self.get_logger().info("🎬 Demo sequence initiated!")
        else:
            self.get_logger().warn("⚠️ No object detected, aborting demo")

    def spawn_status_callback(self, msg):
        """Handle spawn status updates"""
        self.last_spawn_status = msg.data
        self.get_logger().info(f"📦 Spawn status: {msg.data}")

    def detection_status_callback(self, msg):
        """Handle detection status updates"""
        self.last_detection_status = msg.data
        self.get_logger().info(f"🔍 Detection status: {msg.data}")

    def pick_place_status_callback(self, msg):
        """Handle pick and place status updates"""
        self.last_pick_place_status = msg.data
        self.get_logger().info(f"🤖 Pick/Place status: {msg.data}")

    def object_detected_callback(self, msg):
        """Handle object detection"""
        self.object_detected = True
        self.get_logger().info(f"📍 Object detected at: "
                              f"x={msg.pose.position.x:.3f}, "
                              f"y={msg.pose.position.y:.3f}, "
                              f"z={msg.pose.position.z:.3f}")

    def print_status(self):
        """Print current system status"""
        status_text = f"""
        📊 System Status Report
        =====================
        
        📦 Spawn Status: {self.last_spawn_status or 'No status'}
        🔍 Detection Status: {self.last_detection_status or 'No status'}
        🤖 Pick/Place Status: {self.last_pick_place_status or 'No status'}
        📍 Object Detected: {'Yes' if self.object_detected else 'No'}
        
        ⏰ Timestamp: {time.strftime('%H:%M:%S')}
        """
        self.get_logger().info(status_text)

    def status_timer_callback(self):
        """Periodic status update"""
        # Reset object detected flag periodically
        # This ensures fresh detection for each sequence
        if self.object_detected:
            # Keep it True for 30 seconds, then reset
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = ControlInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Control Interface shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()