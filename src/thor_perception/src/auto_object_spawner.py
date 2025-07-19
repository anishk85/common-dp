#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import random
import time
import threading
import math

class AutoObjectSpawner(Node):
    def __init__(self):
        super().__init__('auto_object_spawner')
        
        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Subscriber for electromagnet state (to know when objects are picked)
        self.electromagnet_sub = self.create_subscription(
            Bool, '/thor_arm/electromagnet/control', self.electromagnet_callback, 10)
        
        # Parameters
        self.declare_parameter('spawn_interval', 20.0)  # Increased from 15.0
        self.declare_parameter('max_objects', 4)        # Decreased from 6
        self.declare_parameter('cleanup_interval', 30.0) # Decreased from 60.0
        
        self.spawn_interval = self.get_parameter('spawn_interval').value
        self.max_objects = self.get_parameter('max_objects').value
        self.cleanup_interval = self.get_parameter('cleanup_interval').value
        
        # State variables
        self.spawned_objects = []
        self.electromagnet_active = False
        self.spawn_count = 0
        self.auto_spawn_enabled = True
        
        # Object templates with more variety
        self.object_templates = {
            'small_cube': {
                'size': [0.03, 0.03, 0.03],
                'mass': 0.05,
                'colors': {
                    'red': '1 0 0 1',
                    'blue': '0 0 1 1', 
                    'green': '0 1 0 1',
                    'yellow': '1 1 0 1',
                    'purple': '0.8 0 0.8 1',
                    'orange': '1 0.5 0 1',
                    'cyan': '0 1 1 1',
                    'pink': '1 0.5 0.8 1'
                }
            },
            'medium_cube': {
                'size': [0.05, 0.05, 0.05],
                'mass': 0.1,
                'colors': {
                    'red': '0.8 0 0 1',
                    'blue': '0 0 0.8 1',
                    'green': '0 0.8 0 1',
                    'yellow': '0.8 0.8 0 1'
                }
            },
            'small_cylinder': {
                'radius': 0.02,
                'length': 0.04,
                'mass': 0.06,
                'colors': {
                    'red': '1 0 0 1',
                    'blue': '0 0 1 1',
                    'green': '0 1 0 1',
                    'yellow': '1 1 0 1'
                }
            },
            'medium_cylinder': {
                'radius': 0.025,
                'length': 0.06,
                'mass': 0.08,
                'colors': {
                    'red': '0.9 0 0 1',
                    'blue': '0 0 0.9 1',
                    'green': '0 0.9 0 1'
                }
            },
            'small_sphere': {
                'radius': 0.02,
                'mass': 0.04,
                'colors': {
                    'red': '1 0 0 1',
                    'blue': '0 0 1 1',
                    'green': '0 1 0 1',
                    'yellow': '1 1 0 1',
                    'purple': '0.7 0 0.7 1'
                }
            }
        }
        
        # FIXED: Adjusted spawn area for better distribution
        self.spawn_area = {
            'x_min': 0.25, 'x_max': 0.55,  # Forward area on table
            'y_min': -0.15, 'y_max': 0.15,  # Left-right spread
            'z': 0.88,  # Table height + offset for proper placement
            'safe_radius': 0.10  # Increased minimum distance between objects
        }
        
        # Wait for services
        self.create_timer(2.0, self.check_services)
        
        # Start spawning after services are ready
        self.services_ready = False
        
        self.get_logger().info("🎯 Auto Object Spawner initialized - FIXED VERSION")
        self.get_logger().info(f"📊 Config: spawn_interval={self.spawn_interval}s, max_objects={self.max_objects}")
        self.get_logger().info(f"📍 Spawn area: X[{self.spawn_area['x_min']:.2f}-{self.spawn_area['x_max']:.2f}], Y[{self.spawn_area['y_min']:.2f}-{self.spawn_area['y_max']:.2f}]")
    
    def check_services(self):
        """FIXED: Better service checking with recovery"""
        if not self.services_ready:
            if (self.spawn_client.wait_for_service(timeout_sec=1.0) and 
                self.delete_client.wait_for_service(timeout_sec=1.0)):
                
                self.services_ready = True
                self.get_logger().info("✅ Gazebo services are ready!")
                
                # FIXED: More conservative initial spawn
                self.create_timer(8.0, self.initial_spawn)  # Wait longer for Gazebo to be fully ready
                self.create_timer(self.spawn_interval, self.auto_spawn_objects)
                self.create_timer(self.cleanup_interval, self.cleanup_old_objects)
            else:
                self.get_logger().warn("⚠️ Waiting for Gazebo services...")
        else:
            # FIXED: Periodic service health check
            if not (self.spawn_client.wait_for_service(timeout_sec=0.1) and 
                    self.delete_client.wait_for_service(timeout_sec=0.1)):
                self.get_logger().warn("⚠️ Gazebo services became unavailable!")
                self.services_ready = False
                self.auto_spawn_enabled = False
    
    def electromagnet_callback(self, msg):
        """Track electromagnet state"""
        old_state = self.electromagnet_active
        self.electromagnet_active = msg.data
        
        # If electromagnet just turned on, an object might be picked up
        if not old_state and self.electromagnet_active:
            self.get_logger().info("🧲 Electromagnet activated - object might be picked up")
        elif old_state and not self.electromagnet_active:
            self.get_logger().info("⚫ Electromagnet deactivated - object released")
    
    def initial_spawn(self):
        """Spawn initial set of objects"""
        if len(self.spawned_objects) == 0:
            initial_count = min(3, self.max_objects)  # Start with 3 objects
            self.get_logger().info(f"🎬 Initial spawn: {initial_count} objects")
            self.spawn_random_objects(initial_count)
    
    def auto_spawn_objects(self):
        """FIXED: Auto spawn with better control and limits"""
        if not self.auto_spawn_enabled or not self.services_ready:
            return
        
        current_count = len(self.spawned_objects)
        
        # FIXED: Check actual objects in Gazebo vs our tracking
        if current_count >= self.max_objects:
            self.get_logger().debug(f"📊 Max objects reached ({current_count}/{self.max_objects})")
            return
        
        # FIXED: Only spawn if we really need more objects
        if current_count < 2:  # Maintain minimum 2 objects
            spawn_count = min(1, self.max_objects - current_count)  # Only spawn 1 at a time
            self.get_logger().info(f"🔄 Auto-spawning {spawn_count} object(s) (current: {current_count})")
            
            success = self.spawn_random_objects(spawn_count)
            if not success:
                self.get_logger().warn("⚠️ Spawn failed - pausing auto-spawn for 30s")
                # Pause spawning temporarily
                def resume_spawning():
                    self.auto_spawn_enabled = True
                    self.get_logger().info("🔄 Auto-spawning resumed")
                
                self.create_timer(30.0, resume_spawning)
                self.auto_spawn_enabled = False
    
    def spawn_random_objects(self, count):
        """FIXED: Return success status and better error handling"""
        success_count = 0
        
        for _ in range(count):
            attempts = 0
            max_attempts = 10  # Reduced attempts to prevent excessive retrying
            
            while attempts < max_attempts:
                # Choose random object type and color
                obj_type = random.choice(list(self.object_templates.keys()))
                color_name = random.choice(list(self.object_templates[obj_type]['colors'].keys()))
                
                # Generate random position
                x = random.uniform(self.spawn_area['x_min'], self.spawn_area['x_max'])
                y = random.uniform(self.spawn_area['y_min'], self.spawn_area['y_max'])
                z = self.spawn_area['z']
                
                # Check for collisions with existing objects
                if self.is_position_safe(x, y):
                    # Create unique name
                    obj_name = f"{obj_type}_{color_name}_{self.spawn_count}"
                    self.spawn_count += 1
                    
                    # Spawn object
                    if self.spawn_object(obj_name, obj_type, color_name, x, y, z):
                        self.spawned_objects.append({
                            'name': obj_name,
                            'type': obj_type,
                            'color': color_name,
                            'position': (x, y, z),
                            'spawn_time': time.time()
                        })
                        self.get_logger().info(f"✅ Spawned {obj_type} ({color_name}) at ({x:.2f}, {y:.2f}, {z:.2f})")
                        success_count += 1
                        break
                    else:
                        # Spawn failed, don't retry immediately
                        self.get_logger().warn(f"⚠️ Failed to spawn {obj_type} at ({x:.2f}, {y:.2f}, {z:.2f})")
                        break
                
                attempts += 1
            
            if attempts >= max_attempts:
                self.get_logger().warn("⚠️ Could not find safe spawn position after 10 attempts")
        
        return success_count > 0
    
    def is_position_safe(self, x, y):
        """Check if position is safe (not too close to existing objects)"""
        for obj in self.spawned_objects:
            obj_x, obj_y, _ = obj['position']
            distance = math.sqrt((x - obj_x)**2 + (y - obj_y)**2)
            if distance < self.spawn_area['safe_radius']:
                return False
        return True
    
    def spawn_object(self, name, obj_type, color_name, x, y, z):
        """FIXED: Spawn a single object with better timeout handling"""
        try:
            # Create SDF content based on object type
            sdf_content = self.create_sdf_content(name, obj_type, color_name)
            
            # Create pose with slight random rotation
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            
            # Random rotation around Z axis
            yaw = random.uniform(0, 2 * math.pi)
            pose.orientation.z = math.sin(yaw / 2)
            pose.orientation.w = math.cos(yaw / 2)
            
            # Create service request
            request = SpawnEntity.Request()
            request.name = name
            request.xml = sdf_content
            request.initial_pose = pose
            
            # FIXED: Better service call with shorter timeout and proper handling
            if not self.spawn_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("⚠️ Spawn service not available, pausing spawning")
                self.auto_spawn_enabled = False  # Temporarily disable spawning
                return False
            
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)  # Shorter timeout
            
            if future.done():
                response = future.result()
                if response.success:
                    return True
                else:
                    self.get_logger().warn(f"⚠️ Spawn failed for {name}: {response.status_message}")
                    return False
            else:
                self.get_logger().warn(f"⚠️ Spawn timeout for {name} - slowing down spawning")
                # FIXED: Slow down spawning when timeouts occur
                self.spawn_interval = min(self.spawn_interval * 1.2, 60.0)  # Increase interval
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Error spawning object: {e}")
            return False
    
    def create_sdf_content(self, name, obj_type, color_name):
        """Create SDF content for different object types"""
        template = self.object_templates[obj_type]
        color = template['colors'][color_name]
        mass = template['mass']
        
        # Create geometry based on object type
        if 'cube' in obj_type:
            size = template['size']
            geometry = f"""
                <geometry>
                    <box>
                        <size>{size[0]} {size[1]} {size[2]}</size>
                    </box>
                </geometry>
            """
        elif 'cylinder' in obj_type:
            radius = template['radius']
            length = template['length']
            geometry = f"""
                <geometry>
                    <cylinder>
                        <radius>{radius}</radius>
                        <length>{length}</length>
                    </cylinder>
                </geometry>
            """
        elif 'sphere' in obj_type:
            radius = template['radius']
            geometry = f"""
                <geometry>
                    <sphere>
                        <radius>{radius}</radius>
                    </sphere>
                </geometry>
            """
        
        # Calculate inertia based on mass and geometry
        inertia_val = mass * 0.001  # Simple approximation
        
        sdf_content = f"""
        <?xml version="1.0"?>
        <sdf version="1.7">
            <model name="{name}">
                <link name="link">
                    <pose>0 0 0 0 0 0</pose>
                    <inertial>
                        <mass>{mass}</mass>
                        <inertia>
                            <ixx>{inertia_val}</ixx>
                            <iyy>{inertia_val}</iyy>
                            <izz>{inertia_val}</izz>
                            <ixy>0.0</ixy>
                            <ixz>0.0</ixz>
                            <iyz>0.0</iyz>
                        </inertia>
                    </inertial>
                    <collision name="collision">
                        {geometry}
                        <surface>
                            <friction>
                                <ode>
                                    <mu>0.8</mu>
                                    <mu2>0.8</mu2>
                                </ode>
                            </friction>
                            <contact>
                                <ode>
                                    <kp>1000000.0</kp>
                                    <kd>1.0</kd>
                                    <min_depth>0.001</min_depth>
                                </ode>
                            </contact>
                        </surface>
                    </collision>
                    <visual name="visual">
                        {geometry}
                        <material>
                            <ambient>{color}</ambient>
                            <diffuse>{color}</diffuse>
                            <specular>0.2 0.2 0.2 1</specular>
                            <emissive>0 0 0 1</emissive>
                        </material>
                    </visual>
                </link>
                
                <!-- Make objects magnetic (for electromagnet interaction) -->
                <plugin name="magnetic_properties" filename="libgazebo_ros_magnetic.so">
                    <ros>
                        <namespace></namespace>
                    </ros>
                    <magnetic_field_strength>10.0</magnetic_field_strength>
                    <magnetic_link>link</magnetic_link>
                </plugin>
            </model>
        </sdf>
        """
        
        return sdf_content
    
    def cleanup_old_objects(self):
        """FIXED: More aggressive cleanup to prevent overcrowding"""
        if not self.services_ready:
            return
            
        current_time = time.time()
        objects_to_remove = []
        
        # FIXED: More aggressive cleanup rules
        for obj in self.spawned_objects:
            age = current_time - obj['spawn_time']
            obj_x, obj_y, _ = obj['position']
            
            # Remove if old or if we have too many
            if age > 90:  # Remove after 90 seconds (was 120)
                objects_to_remove.append(obj)
            elif len(self.spawned_objects) > self.max_objects and age > 30:  # Quick removal if overcrowded
                objects_to_remove.append(obj)
        
        # Sort by age and remove oldest first
        objects_to_remove.sort(key=lambda x: x['spawn_time'])
        
        # Remove duplicates
        objects_to_remove = list({obj['name']: obj for obj in objects_to_remove}.values())
        
        # FIXED: Limit how many we try to delete at once
        objects_to_remove = objects_to_remove[:2]  # Max 2 deletions per cleanup cycle
        
        for obj in objects_to_remove:
            if self.delete_object(obj['name']):
                if obj in self.spawned_objects:
                    self.spawned_objects.remove(obj)
                self.get_logger().info(f"🗑️ Cleaned up: {obj['name']} (age: {current_time - obj['spawn_time']:.1f}s)")
    
    def delete_object(self, name):
        """FIXED: Delete an object with better error handling"""
        try:
            request = DeleteEntity.Request()
            request.name = name
            
            # Check if service is available
            if not self.delete_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"⚠️ Delete service not available for {name}")
                return False
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)  # Shorter timeout
            
            if future.done():
                response = future.result()
                if response.success:
                    return True
                else:
                    self.get_logger().warn(f"⚠️ Delete failed for {name}: {response.status_message}")
                    return False
            else:
                self.get_logger().warn(f"⚠️ Delete service call timed out for {name}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Error deleting object {name}: {e}")
            return False
    
    def cleanup_all_objects(self):
        """Clean up all spawned objects"""
        self.get_logger().info("🧹 Cleaning up all spawned objects...")
        for obj in self.spawned_objects.copy():
            if self.delete_object(obj['name']):
                self.spawned_objects.remove(obj)
                self.get_logger().info(f"🗑️ Cleaned up: {obj['name']}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoObjectSpawner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup_all_objects()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()