#!/usr/bin/env python3
# filepath: /home/anish/dp_ws/Thor-ROS/ws_thor/src/thor_manipulation/scripts/spawn_objects.py

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
import random
import numpy as np
import time

class ObjectSpawnerNode(Node):
    def __init__(self):
        super().__init__('object_spawner_node')
        
        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/spawner_status', 10)
        
        # Subscribers
        self.spawn_request_sub = self.create_subscription(
            String, '/spawn_object', self.spawn_object_callback, 10)
        self.clear_objects_sub = self.create_subscription(
            String, '/clear_objects', self.clear_objects_callback, 10)
        
        # Object tracking
        self.spawned_objects = []
        self.object_counter = 0
        self.spawn_pending = False
        
        # Parameters
        self.declare_parameter('auto_spawn_enabled', True)
        auto_spawn_enabled = self.get_parameter('auto_spawn_enabled').get_parameter_value().bool_value
        
        # Wait for services
        self.get_logger().info("Waiting for Gazebo spawn service...")
        try:
            self.spawn_client.wait_for_service(timeout_sec=10.0)
            self.delete_client.wait_for_service(timeout_sec=10.0)
            self.get_logger().info("📦 Object Spawner Node initialized")
            
            # Only spawn initial objects if auto-spawn is enabled and not done already
            if auto_spawn_enabled and not self.spawn_pending:
                self.spawn_pending = True
                # Use a single-shot timer to spawn initial objects
                self.create_timer(3.0, self.spawn_initial_objects_once)
                
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Gazebo services: {e}")
    
    def spawn_initial_objects_once(self):
        """Spawn initial objects - single execution only"""
        try:
            if self.spawn_pending:
                self.spawn_pending = False  # Prevent multiple executions
                self.spawn_initial_objects()
        except Exception as e:
            self.get_logger().error(f"Error in initial spawn: {e}")
    
    def create_can_sdf(self):
        """Create SDF for a can object"""
        return '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="can">
    <pose>0 0 0.065 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.13</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.13</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

    def create_box_sdf(self):
        """Create SDF for a box object"""
        return '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="box">
    <pose>0 0 0.03 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.15</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.06 0.06 0.06</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.06 0.06 0.06</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
          <diffuse>0.2 0.8 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

    def create_sphere_sdf(self):
        """Create SDF for a sphere object"""
        return '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="sphere">
    <pose>0 0 0.025 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.001</iyy>
          <iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.025</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    
    def spawn_initial_objects(self):
        """Spawn some initial objects in the workspace"""
        initial_objects = [
            {'type': 'can', 'x': 0.3, 'y': 0.1},
            {'type': 'box', 'x': 0.4, 'y': -0.1}, 
            {'type': 'sphere', 'x': 0.2, 'y': 0.2}
        ]
        
        self.get_logger().info("📦 Spawning initial objects...")
        for obj in initial_objects:
            self.spawn_object(obj['type'], obj['x'], obj['y'])
            time.sleep(0.5)  # Small delay between spawns
    
    def spawn_object_callback(self, msg):
        """Handle spawn object request"""
        object_type = msg.data.lower()
        
        if object_type in ['can', 'box', 'sphere']:
            # Generate random position in workspace
            x = random.uniform(0.2, 0.5)
            y = random.uniform(-0.3, 0.3)
            self.spawn_object(object_type, x, y)
        else:
            self.get_logger().warn(f"❌ Unknown object type: {object_type}")
            self.publish_status(f"Unknown object type: {object_type}")
    
    def spawn_object(self, object_type, x, y, z=0.0):
        """Spawn an object in Gazebo"""
        try:
            # Get SDF content
            object_sdfs = {
                'can': self.create_can_sdf(),
                'box': self.create_box_sdf(),
                'sphere': self.create_sphere_sdf()
            }
            
            if object_type not in object_sdfs:
                self.get_logger().error(f"❌ Unknown object type: {object_type}")
                return
            
            # Create unique name with timestamp to avoid conflicts
            import time
            timestamp = int(time.time() * 1000) % 10000  # Last 4 digits of timestamp
            object_name = f"{object_type}_{timestamp}"
            
            # Create pose
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            
            # Random orientation
            yaw = random.uniform(-np.pi, np.pi)
            q = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            # Create service request
            request = SpawnEntity.Request()
            request.name = object_name
            request.xml = object_sdfs[object_type]
            request.initial_pose = pose
            
            # Call service
            future = self.spawn_client.call_async(request)
            future.add_done_callback(
                lambda f, name=object_name: self.spawn_response_callback(f, name))
            
        except Exception as e:
            self.get_logger().error(f"❌ Error spawning object: {str(e)}")
    
    def spawn_response_callback(self, future, object_name):
        """Handle spawn service response"""
        try:
            response = future.result()
            if response.success:
                self.spawned_objects.append(object_name)
                self.get_logger().info(f"✅ Successfully spawned {object_name}")
                self.publish_status(f"Spawned {object_name}")
            else:
                self.get_logger().warn(f"⚠️ Failed to spawn {object_name}: {response.status_message}")
                self.publish_status(f"Failed to spawn {object_name}")
        except Exception as e:
            self.get_logger().error(f"❌ Error in spawn response: {str(e)}")
    
    def clear_objects_callback(self, msg):
        """Clear all spawned objects"""
        self.get_logger().info("🧹 Clearing all spawned objects...")
        self.publish_status("Clearing all objects")
        
        objects_to_clear = self.spawned_objects.copy()  # Create copy to avoid modification during iteration
        for object_name in objects_to_clear:
            self.delete_object(object_name)
        
        self.spawned_objects.clear()
    
    def delete_object(self, object_name):
        """Delete an object from Gazebo"""
        try:
            request = DeleteEntity.Request()
            request.name = object_name
            
            future = self.delete_client.call_async(request)
            future.add_done_callback(
                lambda f, name=object_name: self.delete_response_callback(f, name))
            
        except Exception as e:
            self.get_logger().error(f"❌ Error deleting object {object_name}: {str(e)}")
    
    def delete_response_callback(self, future, object_name):
        """Handle delete service response"""
        try:
            response = future.result()
            if response.success:
                if object_name in self.spawned_objects:
                    self.spawned_objects.remove(object_name)
                self.get_logger().info(f"✅ Successfully deleted {object_name}")
                self.publish_status(f"Deleted {object_name}")
            else:
                self.get_logger().warn(f"⚠️ Failed to delete {object_name}: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"❌ Error in delete response: {str(e)}")
    
    def publish_status(self, status):
        """Publish status message"""
        try:
            msg = String()
            msg.data = status
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = ObjectSpawnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in spawn_objects: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()