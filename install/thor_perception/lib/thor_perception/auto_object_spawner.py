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
        self.declare_parameter('spawn_interval', 15.0)
        self.declare_parameter('max_objects', 6)
        self.declare_parameter('cleanup_interval', 60.0)
        
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
        
        # Spawn area (table surface)
        self.spawn_area = {
            'x_min': -0.25, 'x_max': 0.25,
            'y_min': -0.25, 'y_max': 0.25,
            'z': 0.83,  # Table height + small offset
            'safe_radius': 0.08  # Minimum distance between objects
        }
        
        # Wait for services
        self.create_timer(2.0, self.check_services)
        
        # Start spawning after services are ready
        self.services_ready = False
        
        self.get_logger().info("🎯 Auto Object Spawner initialized")
        self.get_logger().info(f"📊 Config: spawn_interval={self.spawn_interval}s, max_objects={self.max_objects}")
    
    def check_services(self):
        """Check if Gazebo services are available"""
        if not self.services_ready:
            if (self.spawn_client.wait_for_service(timeout_sec=1.0) and 
                self.delete_client.wait_for_service(timeout_sec=1.0)):
                
                self.services_ready = True
                self.get_logger().info("✅ Gazebo services are ready!")
                
                # Start the spawning and cleanup timers
                self.create_timer(5.0, self.initial_spawn)  # Initial spawn after 5s
                self.create_timer(self.spawn_interval, self.auto_spawn_objects)
                self.create_timer(self.cleanup_interval, self.cleanup_old_objects)
            else:
                self.get_logger().warn("⚠️ Waiting for Gazebo services...")
    
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
            initial_count = min(4, self.max_objects)
            self.get_logger().info(f"🎬 Initial spawn: {initial_count} objects")
            self.spawn_random_objects(initial_count)
    
    def auto_spawn_objects(self):
        """Automatically spawn objects if needed"""
        if not self.auto_spawn_enabled or not self.services_ready:
            return
        
        current_count = len(self.spawned_objects)
        
        if current_count < self.max_objects:
            # Spawn 1-2 new objects
            spawn_count = min(random.randint(1, 2), self.max_objects - current_count)
            self.get_logger().info(f"🔄 Auto-spawning {spawn_count} objects (current: {current_count})")
            self.spawn_random_objects(spawn_count)
    
    def spawn_random_objects(self, count):
        """Spawn random objects with collision avoidance"""
        for _ in range(count):
            attempts = 0
            max_attempts = 20
            
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
                        break
                
                attempts += 1
            
            if attempts >= max_attempts:
                self.get_logger().warn("⚠️ Could not find safe spawn position after 20 attempts")
    
    def is_position_safe(self, x, y):
        """Check if position is safe (not too close to existing objects)"""
        for obj in self.spawned_objects:
            obj_x, obj_y, _ = obj['position']
            distance = math.sqrt((x - obj_x)**2 + (y - obj_y)**2)
            if distance < self.spawn_area['safe_radius']:
                return False
        return True
    
    def spawn_object(self, name, obj_type, color_name, x, y, z):
        """Spawn a single object"""
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
            
            # Call service
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                return response.success
            else:
                self.get_logger().error("❌ Spawn service call timed out")
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
        """Remove old objects to prevent clutter"""
        if not self.services_ready:
            return
            
        current_time = time.time()
        objects_to_remove = []
        
        # Remove objects older than 2 minutes that are far from robot
        for obj in self.spawned_objects:
            age = current_time - obj['spawn_time']
            obj_x, obj_y, _ = obj['position']
            
            # Remove if old and far from center (likely placed objects)
            distance_from_center = math.sqrt(obj_x**2 + obj_y**2)
            
            if age > 120 and distance_from_center > 0.3:  # 2 minutes old and >30cm from center
                objects_to_remove.append(obj)
            elif age > 300:  # Force remove after 5 minutes
                objects_to_remove.append(obj)
        
        # Also remove excess objects if we have too many
        if len(self.spawned_objects) > self.max_objects:
            # Sort by age and remove oldest excess
            sorted_objects = sorted(self.spawned_objects, key=lambda x: x['spawn_time'])
            excess_count = len(self.spawned_objects) - self.max_objects
            objects_to_remove.extend(sorted_objects[:excess_count])
        
        # Remove duplicates
        objects_to_remove = list({obj['name']: obj for obj in objects_to_remove}.values())
        
        for obj in objects_to_remove:
            if self.delete_object(obj['name']):
                if obj in self.spawned_objects:
                    self.spawned_objects.remove(obj)
                self.get_logger().info(f"🗑️ Cleaned up: {obj['name']} (age: {current_time - obj['spawn_time']:.1f}s)")
    
    def delete_object(self, name):
        """Delete an object from Gazebo"""
        try:
            request = DeleteEntity.Request()
            request.name = name
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.done():
                response = future.result()
                return response.success
            else:
                self.get_logger().error(f"❌ Delete service call timed out for {name}")
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