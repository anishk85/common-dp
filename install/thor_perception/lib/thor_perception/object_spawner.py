#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        
        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Wait for services
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete service...')
        
        # Object counter
        self.object_counter = 0
        self.spawned_objects = []
        
        # Services
        self.spawn_service = self.create_service(
            Empty,
            '/spawn_random_objects',
            self.spawn_random_objects_callback
        )
        
        self.clear_service = self.create_service(
            Empty,
            '/clear_objects',
            self.clear_objects_callback
        )
        
        # Object definitions
        self.object_definitions = {
            'red_cube': self.get_cube_sdf('red_cube', 0.05, (1.0, 0.0, 0.0)),
            'blue_cube': self.get_cube_sdf('blue_cube', 0.08, (0.0, 0.0, 1.0)),
            'green_cube': self.get_cube_sdf('green_cube', 0.06, (0.0, 1.0, 0.0)),
            'yellow_cylinder': self.get_cylinder_sdf('yellow_cylinder', 0.03, 0.08, (1.0, 1.0, 0.0)),
            'red_cylinder': self.get_cylinder_sdf('red_cylinder', 0.04, 0.06, (1.0, 0.0, 0.0)),
            'blue_cylinder': self.get_cylinder_sdf('blue_cylinder', 0.035, 0.07, (0.0, 0.0, 1.0))
        }
        
        self.get_logger().info("Object spawner initialized")
    
    def get_cube_sdf(self, name, size, color):
        """Generate SDF for a cube"""
        return f"""
        <sdf version='1.6'>
          <model name='{name}'>
            <pose>0 0 0 0 0 0</pose>
            <link name='link'>
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
              <collision name='collision'>
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>0.8</mu>
                      <mu2>0.8</mu2>
                    </ode>
                  </friction>
                </surface>
              </collision>
              <visual name='visual'>
                <geometry>
                  <box>
                    <size>{size} {size} {size}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{color[0]} {color[1]} {color[2]} 1</ambient>
                  <diffuse>{color[0]} {color[1]} {color[2]} 1</diffuse>
                  <specular>0.1 0.1 0.1 1</specular>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
    
    def get_cylinder_sdf(self, name, radius, length, color):
        """Generate SDF for a cylinder"""
        return f"""
        <sdf version='1.6'>
          <model name='{name}'>
            <pose>0 0 0 0 0 0</pose>
            <link name='link'>
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
              <collision name='collision'>
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{length}</length>
                  </cylinder>
                </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>0.8</mu>
                      <mu2>0.8</mu2>
                    </ode>
                  </friction>
                </surface>
              </collision>
              <visual name='visual'>
                <geometry>
                  <cylinder>
                    <radius>{radius}</radius>
                    <length>{length}</length>
                  </cylinder>
                </geometry>
                <material>
                  <ambient>{color[0]} {color[1]} {color[2]} 1</ambient>
                  <diffuse>{color[0]} {color[1]} {color[2]} 1</diffuse>
                  <specular>0.1 0.1 0.1 1</specular>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
    
    def spawn_random_objects_callback(self, request, response):
        """Spawn random objects in the workspace"""
        num_objects = random.randint(3, 6)
        
        for _ in range(num_objects):
            self.spawn_random_object()
        
        self.get_logger().info(f"Spawned {num_objects} random objects")
        return response
    
    def spawn_random_object(self):
        """Spawn a single random object"""
        # Choose random object type
        object_type = random.choice(list(self.object_definitions.keys()))
        
        # Generate unique name
        object_name = f"{object_type}_{self.object_counter}"
        self.object_counter += 1
        
        # Generate random pose within workspace
        pose = self.generate_random_pose()
        
        # Spawn object
        self.spawn_object(object_name, object_type, pose)
        self.spawned_objects.append(object_name)
    
    def generate_random_pose(self):
        """Generate random pose within robot workspace"""
        pose = Pose()
        
        # Define workspace boundaries (in front of robot)
        x_min, x_max = 0.3, 0.6
        y_min, y_max = -0.3, 0.3
        z_table = 0.01  # Table height
        
        # Random position
        pose.position.x = random.uniform(x_min, x_max)
        pose.position.y = random.uniform(y_min, y_max)
        pose.position.z = z_table
        
        # Random orientation (only rotation around Z-axis)
        yaw = random.uniform(0, 2 * math.pi)
        pose.orientation.z = math.sin(yaw / 2)
        pose.orientation.w = math.cos(yaw / 2)
        
        return pose
    
    def spawn_object(self, name, object_type, pose):
        """Spawn an object in Gazebo"""
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.object_definitions[object_type]
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"
        
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Successfully spawned {name}")
            else:
                self.get_logger().error(f"Failed to spawn {name}: {future.result().status_message}")
        else:
            self.get_logger().error(f"Failed to spawn {name}: Service call failed")
    
    def clear_objects_callback(self, request, response):
        """Clear all spawned objects"""
        for obj_name in self.spawned_objects:
            self.delete_object(obj_name)
        
        self.spawned_objects.clear()
        self.get_logger().info("Cleared all spawned objects")
        return response
    
    def delete_object(self, name):
        """Delete an object from Gazebo"""
        request = DeleteEntity.Request()
        request.name = name
        
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Successfully deleted {name}")
            else:
                self.get_logger().warn(f"Failed to delete {name}: {future.result().status_message}")
        else:
            self.get_logger().error(f"Failed to delete {name}: Service call failed")
    
    def spawn_specific_object(self, object_type, pose, name=None):
        """Spawn a specific object at a specific pose"""
        if object_type not in self.object_definitions:
            self.get_logger().error(f"Unknown object type: {object_type}")
            return False
        
        if name is None:
            name = f"{object_type}_{self.object_counter}"
            self.object_counter += 1
        
        self.spawn_object(name, object_type, pose)
        self.spawned_objects.append(name)
        return True


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