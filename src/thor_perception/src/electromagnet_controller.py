#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import time

class ElectromagnetController(Node):
    def __init__(self):
        super().__init__('electromagnet_controller')
        
        # Publisher for electromagnet control
        self.electromagnet_pub = self.create_publisher(
            Bool, 
            '/electromagnet_control', 
            10
        )
        
        # Service or action clients can be added here for move commands
        # For now, we'll use a simple timer-based demo
        
        # Parameters
        self.declare_parameter('demo_mode', True)
        self.declare_parameter('pickup_sequence_delay', 2.0)
        self.declare_parameter('hold_duration', 5.0)
        
        self.demo_mode = self.get_parameter('demo_mode').value
        self.pickup_delay = self.get_parameter('pickup_sequence_delay').value
        self.hold_duration = self.get_parameter('hold_duration').value
        
        # State variables
        self.electromagnet_active = False
        self.sequence_running = False
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint state subscriber to monitor arm position
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_states = None
        
        # Create service for manual control
        self.create_subscription(
            Bool,
            '/electromagnet_manual_control',
            self.manual_control_callback,
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            Bool,
            '/electromagnet_status',
            10
        )
        
        # Timer for status publishing
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        if self.demo_mode:
            self.get_logger().info("Electromagnet controller started in demo mode")
            # Start demo sequence after a delay
            self.demo_timer = self.create_timer(3.0, self.start_demo_sequence)
        else:
            self.get_logger().info("Electromagnet controller started in manual mode")
            self.get_logger().info("Send Bool messages to /electromagnet_manual_control to control")
    
    def joint_state_callback(self, msg):
        """Store current joint states for monitoring arm position"""
        self.current_joint_states = msg
    
    def manual_control_callback(self, msg):
        """Handle manual electromagnet control"""
        if not self.demo_mode:
            self.set_electromagnet_state(msg.data)
    
    def set_electromagnet_state(self, active):
        """Set electromagnet on/off state"""
        self.electromagnet_active = active
        
        # Create and publish electromagnet control message
        electromagnet_msg = Bool()
        electromagnet_msg.data = active
        self.electromagnet_pub.publish(electromagnet_msg)
        
        if active:
            self.get_logger().info("🧲 Electromagnet ACTIVATED - Attracting objects")
        else:
            self.get_logger().info("⚫ Electromagnet DEACTIVATED - Releasing objects")
    
    def publish_status(self):
        """Publish current electromagnet status"""
        status_msg = Bool()
        status_msg.data = self.electromagnet_active
        self.status_pub.publish(status_msg)
    
    def start_demo_sequence(self):
        """Start automated demo sequence"""
        if self.sequence_running:
            return
        
        self.sequence_running = True
        self.get_logger().info("🎬 Starting electromagnet demo sequence...")
        
        # Cancel the demo timer
        self.demo_timer.cancel()
        
        # Create sequence timer
        self.sequence_step = 0
        self.sequence_timer = self.create_timer(1.0, self.demo_sequence_step)
    
    def demo_sequence_step(self):
        """Execute demo sequence steps"""
        if self.sequence_step == 0:
            self.get_logger().info("📍 Step 1: Waiting for arm to reach pickup position...")
            self.sequence_step += 1
            
        elif self.sequence_step == 1:
            self.get_logger().info("🔄 Step 2: Activating electromagnet...")
            self.set_electromagnet_state(True)
            self.sequence_step += 1
            
        elif self.sequence_step == 2:
            self.get_logger().info(f"⏰ Step 3: Holding object for {self.hold_duration} seconds...")
            self.sequence_step += 1
            
        elif self.sequence_step == 3:
            # Hold for specified duration
            self.hold_counter = getattr(self, 'hold_counter', 0) + 1
            if self.hold_counter >= self.hold_duration:
                self.get_logger().info("📦 Step 4: Releasing object...")
                self.set_electromagnet_state(False)
                self.sequence_step += 1
                self.hold_counter = 0
                
        elif self.sequence_step == 4:
            self.get_logger().info("✅ Demo sequence complete!")
            self.sequence_step += 1
            
        elif self.sequence_step == 5:
            self.get_logger().info("🔄 Restarting demo sequence in 5 seconds...")
            self.sequence_step += 1
            
        elif self.sequence_step == 6:
            # Wait 5 seconds before restarting
            self.restart_counter = getattr(self, 'restart_counter', 0) + 1
            if self.restart_counter >= 5:
                self.sequence_step = 0
                self.restart_counter = 0
                self.get_logger().info("🔄 Restarting demo sequence...")
    
    def get_electromagnet_position(self):
        """Get current electromagnet position in world coordinates"""
        try:
            # Get transform from world to electromagnet_plate
            transform = self.tf_buffer.lookup_transform(
                'world',
                'electromagnet_plate',
                rclpy.time.Time()
            )
            
            # Create pose at electromagnet position
            pose = PoseStamped()
            pose.header.frame_id = 'electromagnet_plate'
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            # Transform to world coordinates
            world_pose = do_transform_pose(pose, transform)
            return world_pose.pose.position
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error getting electromagnet position: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    
    electromagnet_controller = ElectromagnetController()
    
    try:
        rclpy.spin(electromagnet_controller)
    except KeyboardInterrupt:
        electromagnet_controller.get_logger().info("Electromagnet controller shutting down...")
    finally:
        electromagnet_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()