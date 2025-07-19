#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
import random
import os

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Service servers
        self.spawn_service = self.create_service(
            Empty,
            '/spawn_random_objects',
            self.spawn_random_objects_callback
        )
        
        self.spawned_objects = []
        
        self.get_logger().info("🎯 Object spawner initialized")
    
    def spawn_random_objects_callback(self, request, response):
        """Spawn random objects in the scene"""
        try:
            # Clear existing objects
            self.clear_objects()
            
            # Spawn 3-5 objects
            num_objects = random.randint(3, 5)
            
            for i in range(num_objects):
                self.spawn_colored_object(i)
            
            self.get_logger().info(f"✅ Spawned {num_objects} objects")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error spawning objects: {e}")
        
        return response
    
    def spawn_colored_object(self, index):
        """Spawn a colored object"""
        colors = ['red', 'blue', 'green', 'yellow', 'orange']
        color = random.choice(colors)
        
        # LARGER object SDF
        sdf_content = f"""
        <?xml version="1.0"?>
        <sdf version="1.7">
          <model name="colored_object_{index}">
            <link name="link">
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>0.001</ixx>
                  <iyy>0.001</iyy>
                  <izz>0.001</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.08 0.08 0.08</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.08 0.08 0.08</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{self.get_color_rgba(color)}</ambient>
                  <diffuse>{self.get_color_rgba(color)}</diffuse>
                  <specular>0.1 0.1 0.1 1</specular>
                </material>
              </visual>
            </link>
            <plugin name="magnetism" filename="libgazebo_magnetism.so">
              <magnetic_material>ferrous</magnetic_material>
              <magnetic_strength>1.0</magnetic_strength>
            </plugin>
          </model>
        </sdf>
        """
        
        # Random position in workspace
        pose = Pose()
        pose.position.x = random.uniform(0.2, 0.6)
        pose.position.y = random.uniform(-0.3, 0.3)
        pose.position.z = 0.8  # Above table
        pose.orientation.w = 1.0
        
        # Spawn request
        request = SpawnEntity.Request()
        request.name = f"colored_object_{index}"
        request.xml = sdf_content
        request.initial_pose = pose
        
        if self.spawn_client.wait_for_service(timeout_sec=1.0):
            future = self.spawn_client.call_async(request)
            self.spawned_objects.append(f"colored_object_{index}")
            self.get_logger().info(f"🎨 Spawned {color} object at ({pose.position.x:.2f}, {pose.position.y:.2f})")
    
    def get_color_rgba(self, color):
        """Get RGBA values for colors"""
        colors = {
            'red': '1 0 0 1',
            'blue': '0 0 1 1',
            'green': '0 1 0 1',
            'yellow': '1 1 0 1',
            'orange': '1 0.5 0 1',
            'purple': '0.5 0 1 1'
        }
        return colors.get(color, '0.5 0.5 0.5 1')
    
    def clear_objects(self):
        """Clear all spawned objects"""
        for obj_name in self.spawned_objects:
            request = DeleteEntity.Request()
            request.name = obj_name
            if self.delete_client.wait_for_service(timeout_sec=1.0):
                self.delete_client.call_async(request)
        
        self.spawned_objects.clear()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()