#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test')
        
        # Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray, 
            '/thor_arm_controller/commands', 
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names
        self.joint_names = [
            'base_joint',
            'shoulder_joint', 
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Current joint positions
        self.current_positions = [0.0] * 6
        
        # Test parameters
        self.test_mode = 'individual'  # 'individual', 'sequence', 'sine_wave'
        self.test_amplitude = 0.5  # radians
        self.test_frequency = 0.1  # Hz
        self.current_joint = 0
        
        # Timer for test execution
        self.test_timer = self.create_timer(0.1, self.run_test)
        
        self.get_logger().info('Motor test node started')
        self.get_logger().info(f'Test mode: {self.test_mode}')
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_positions[i] = msg.position[idx]
        except Exception as e:
            self.get_logger().warn(f'Joint state callback error: {str(e)}')
            
    def run_test(self):
        """Execute motor test based on selected mode"""
        if self.test_mode == 'individual':
            self.test_individual_joints()
        elif self.test_mode == 'sequence':
            self.test_joint_sequence()
        elif self.test_mode == 'sine_wave':
            self.test_sine_wave_motion()
            
    def test_individual_joints(self):
        """Test joints individually with sine wave motion"""
        current_time = time.time()
        
        # Create command array
        commands = [0.0] * 6
        
        # Apply sine wave to current joint
        commands[self.current_joint] = self.test_amplitude * math.sin(
            2 * math.pi * self.test_frequency * current_time
        )
        
        # Send command
        self.send_joint_commands(commands)
        
        # Switch joint every 10 seconds
        if int(current_time) % 10 == 0 and int(current_time * 10) % 10 == 0:
            self.current_joint = (self.current_joint + 1) % 5  # Skip gripper for now
            self.get_logger().info(f'Testing joint: {self.joint_names[self.current_joint]}')
            
    def test_joint_sequence(self):
        """Test joints in sequence - simple back and forth motion"""
        current_time = time.time()
        cycle_time = 20.0  # 20 seconds per full cycle
        
        # Determine which joint to move based on time
        phase = (current_time % cycle_time) / cycle_time
        joint_phase = phase * 5  # 5 joints (excluding gripper)
        current_joint = int(joint_phase)
        joint_progress = joint_phase - current_joint
        
        # Create commands
        commands = [0.0] * 6
        
        if current_joint < 5:
            # Triangle wave motion
            if joint_progress < 0.5:
                commands[current_joint] = self.test_amplitude * (joint_progress * 4 - 1)
            else:
                commands[current_joint] = self.test_amplitude * (3 - joint_progress * 4)
                
        self.send_joint_commands(commands)
        
    def test_sine_wave_motion(self):
        """Test all joints with different frequency sine waves"""
        current_time = time.time()
        
        # Different frequencies for each joint
        frequencies = [0.1, 0.15, 0.2, 0.25, 0.3, 0.05]
        amplitudes = [0.5, 0.3, 0.4, 0.6, 0.8, 0.02]  # Smaller for gripper
        
        commands = []
        for i in range(6):
            commands.append(amplitudes[i] * math.sin(
                2 * math.pi * frequencies[i] * current_time
            ))
            
        self.send_joint_commands(commands)
        
    def send_joint_commands(self, positions):
        """Send joint position commands"""
        try:
            msg = Float64MultiArray()
            msg.data = positions
            self.joint_command_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Command sending error: {str(e)}')
            
    def run_safety_test(self):
        """Test emergency stop and safety limits"""
        self.get_logger().info('Running safety tests...')
        
        # Test 1: Send commands within limits
        safe_commands = [0.1, 0.1, 0.1, 0.1, 0.1, 0.01]
        self.send_joint_commands(safe_commands)
        time.sleep(2)
        
        # Test 2: Try to send commands at limits (should be accepted)
        limit_commands = [3.0, 1.5, 3.0, 3.0, 1.5, 0.07]
        self.send_joint_commands(limit_commands)
        time.sleep(2)
        
        # Test 3: Return to home position
        home_commands = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joint_commands(home_commands)
        
        self.get_logger().info('Safety tests completed')
        
    def run_calibration_routine(self):
        """Run motor calibration routine"""
        self.get_logger().info('Running motor calibration...')
        
        # Move to known positions and verify encoder readings
        calibration_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],     # Home
            [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],     # Base +
            [-0.5, 0.0, 0.0, 0.0, 0.0, 0.0],    # Base -
            [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],     # Shoulder +
            [0.0, -0.5, 0.0, 0.0, 0.0, 0.0],    # Shoulder -
            # Add more positions as needed
        ]
        
        for i, positions in enumerate(calibration_positions):
            self.get_logger().info(f'Moving to calibration position {i+1}')
            self.send_joint_commands(positions)
            time.sleep(3)  # Wait for movement
            
            # Log current positions for verification
            self.get_logger().info(f'Target: {positions}')
            self.get_logger().info(f'Actual: {self.current_positions}')
            
        self.get_logger().info('Calibration routine completed')

def main(args=None):
    rclpy.init(args=args)
    
    motor_test_node = MotorTestNode()
    
    try:
        # Run different test modes
        motor_test_node.get_logger().info('Starting motor tests...')
        
        # Option to run specific tests
        import sys
        if len(sys.argv) > 1:
            test_type = sys.argv[1]
            if test_type == 'safety':
                motor_test_node.run_safety_test()
            elif test_type == 'calibration':
                motor_test_node.run_calibration_routine()
        
        rclpy.spin(motor_test_node)
        
    except KeyboardInterrupt:
        motor_test_node.get_logger().info('Motor test interrupted')
    finally:
        motor_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
