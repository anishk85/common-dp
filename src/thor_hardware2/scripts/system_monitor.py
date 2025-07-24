#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String
import serial
import json
import time
from threading import Thread, Lock

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('monitor_frequency', 10.0)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.monitor_freq = self.get_parameter('monitor_frequency').get_parameter_value().double_value
        
        # Serial connection
        self.serial_conn = None
        self.serial_lock = Lock()
        self.connect_to_arduino()
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # System state
        self.last_joint_state = None
        self.system_health = {
            'arduino_connected': False,
            'motors_enabled': [False] * 6,
            'motor_faults': [False] * 6,
            'voltages': {'5v': 0.0, '12v': 0.0},
            'currents': [0.0] * 6,
            'temperatures': {'arduino': 0.0, 'drivers': [0.0] * 3},
            'last_heartbeat': 0.0
        }
        
        # Monitoring timer
        self.monitor_timer = self.create_timer(1.0/self.monitor_freq, self.monitor_system)
        
        # Background thread for serial monitoring
        self.monitoring_thread = Thread(target=self.serial_monitor_loop, daemon=True)
        self.monitoring_thread.start()
        
        self.get_logger().info('System monitor started')
        
    def connect_to_arduino(self):
        """Establish serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1
            )
            self.system_health['arduino_connected'] = True
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.system_health['arduino_connected'] = False
            
    def serial_monitor_loop(self):
        """Background thread for monitoring Arduino"""
        while rclpy.ok():
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    self.query_arduino_status()
                else:
                    self.attempt_reconnection()
                    
                time.sleep(1.0)  # Query every second
                
            except Exception as e:
                self.get_logger().error(f'Serial monitor error: {str(e)}')
                time.sleep(5.0)  # Wait before retry
                
    def query_arduino_status(self):
        """Query Arduino for system status"""
        try:
            with self.serial_lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    return
                    
                # Send status request command
                cmd = bytearray([0xAA, 0x06, 0x00, 0x00, 0x55])  # GET_STATUS command
                self.serial_conn.write(cmd)
                self.serial_conn.flush()
                
                # Read response
                response = self.serial_conn.read(16)  # Expected response size
                
                if len(response) >= 10:
                    self.parse_arduino_status(response)
                    self.system_health['last_heartbeat'] = time.time()
                    
        except Exception as e:
            self.get_logger().warn(f'Arduino query error: {str(e)}')
            self.system_health['arduino_connected'] = False
            
    def parse_arduino_status(self, data):
        """Parse Arduino status response"""
        try:
            if len(data) < 10:
                return
                
            # Parse voltage (bytes 1-2)
            voltage_raw = int.from_bytes(data[1:3], byteorder='little')
            self.system_health['voltages']['5v'] = voltage_raw / 1000.0
            
            # Parse current (bytes 3-4)  
            current_raw = int.from_bytes(data[3:5], byteorder='little')
            total_current = current_raw / 1000.0
            
            # Estimate individual motor currents (simplified)
            for i in range(6):
                self.system_health['currents'][i] = total_current / 6.0
                
            # Parse temperature (bytes 5-6)
            temp_raw = int.from_bytes(data[5:7], byteorder='little')
            self.system_health['temperatures']['arduino'] = temp_raw / 100.0
            
            # Parse status flags (byte 7)
            status_flags = data[7]
            for i in range(6):
                self.system_health['motors_enabled'][i] = bool(status_flags & (1 << i))
                
            self.system_health['arduino_connected'] = True
            
        except Exception as e:
            self.get_logger().warn(f'Status parsing error: {str(e)}')
            
    def attempt_reconnection(self):
        """Try to reconnect to Arduino"""
        try:
            if self.serial_conn:
                self.serial_conn.close()
                
            time.sleep(2)
            self.connect_to_arduino()
            
        except Exception as e:
            self.get_logger().debug(f'Reconnection attempt failed: {str(e)}')
            
    def joint_state_callback(self, msg):
        """Update joint state information"""
        self.last_joint_state = msg
        
        # Check for unusual joint values
        for i, position in enumerate(msg.position[:6]):
            if abs(position) > 4.0:  # Position limit check
                self.get_logger().warn(f'Joint {i} position {position:.3f} exceeds limits')
                
        # Check velocities if available
        if len(msg.velocity) >= 6:
            for i, velocity in enumerate(msg.velocity[:6]):
                if abs(velocity) > 7.0:  # Velocity limit check
                    self.get_logger().warn(f'Joint {i} velocity {velocity:.3f} exceeds limits')
                    
    def monitor_system(self):
        """Main system monitoring function"""
        # Create diagnostic message
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Arduino connection status
        arduino_status = DiagnosticStatus()
        arduino_status.name = "Arduino Connection"
        arduino_status.hardware_id = self.serial_port
        
        if self.system_health['arduino_connected']:
            time_since_heartbeat = time.time() - self.system_health['last_heartbeat']
            if time_since_heartbeat < 5.0:
                arduino_status.level = DiagnosticStatus.OK
                arduino_status.message = "Connected and responding"
            else:
                arduino_status.level = DiagnosticStatus.WARN
                arduino_status.message = f"No response for {time_since_heartbeat:.1f}s"
        else:
            arduino_status.level = DiagnosticStatus.ERROR
            arduino_status.message = "Disconnected"
            
        arduino_status.values.append(KeyValue(
            key="port", value=self.serial_port
        ))
        diag_array.status.append(arduino_status)
        
        # Motor status
        for i in range(6):
            motor_status = DiagnosticStatus()
            motor_status.name = f"Motor {i}"
            motor_status.hardware_id = f"motor_{i}"
            
            if self.system_health['motor_faults'][i]:
                motor_status.level = DiagnosticStatus.ERROR
                motor_status.message = "Motor fault detected"
            elif not self.system_health['motors_enabled'][i]:
                motor_status.level = DiagnosticStatus.WARN
                motor_status.message = "Motor disabled"
            else:
                motor_status.level = DiagnosticStatus.OK
                motor_status.message = "Operating normally"
                
            motor_status.values.extend([
                KeyValue(key="enabled", value=str(self.system_health['motors_enabled'][i])),
                KeyValue(key="current", value=f"{self.system_health['currents'][i]:.3f}A"),
                KeyValue(key="fault", value=str(self.system_health['motor_faults'][i]))
            ])
            diag_array.status.append(motor_status)
            
        # Power status
        power_status = DiagnosticStatus()
        power_status.name = "Power System"
        power_status.hardware_id = "power_supply"
        
        voltage_5v = self.system_health['voltages']['5v']
        if 4.8 <= voltage_5v <= 5.2:
            power_status.level = DiagnosticStatus.OK
            power_status.message = "Voltages normal"
        elif 4.5 <= voltage_5v <= 5.5:
            power_status.level = DiagnosticStatus.WARN
            power_status.message = "Voltage slightly out of range"
        else:
            power_status.level = DiagnosticStatus.ERROR
            power_status.message = "Voltage critical"
            
        power_status.values.extend([
            KeyValue(key="5v_rail", value=f"{voltage_5v:.2f}V"),
            KeyValue(key="total_current", value=f"{sum(self.system_health['currents']):.2f}A")
        ])
        diag_array.status.append(power_status)
        
        # Temperature status
        temp_status = DiagnosticStatus()
        temp_status.name = "Temperature"
        temp_status.hardware_id = "thermal"
        
        arduino_temp = self.system_health['temperatures']['arduino']
        if arduino_temp < 60.0:
            temp_status.level = DiagnosticStatus.OK
            temp_status.message = "Temperature normal"
        elif arduino_temp < 75.0:
            temp_status.level = DiagnosticStatus.WARN
            temp_status.message = "Temperature elevated"
        else:
            temp_status.level = DiagnosticStatus.ERROR
            temp_status.message = "Temperature critical"
            
        temp_status.values.append(KeyValue(
            key="arduino_temp", value=f"{arduino_temp:.1f}°C"
        ))
        diag_array.status.append(temp_status)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diag_array)
        
        # Publish system status summary
        status_summary = {
            'timestamp': time.time(),
            'arduino_connected': self.system_health['arduino_connected'],
            'motors_ok': not any(self.system_health['motor_faults']),
            'voltage_ok': 4.8 <= voltage_5v <= 5.2,
            'temperature_ok': arduino_temp < 60.0,
            'last_joint_update': self.last_joint_state.header.stamp.sec if self.last_joint_state else 0
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_summary)
        self.status_pub.publish(status_msg)
        
    def destroy_node(self):
        """Clean up resources"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    system_monitor = SystemMonitorNode()
    
    try:
        rclpy.spin(system_monitor)
    except KeyboardInterrupt:
        system_monitor.get_logger().info('System monitor shutting down')
    finally:
        system_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
