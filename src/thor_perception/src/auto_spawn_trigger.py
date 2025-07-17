#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

class AutoSpawnTrigger(Node):
    def __init__(self):
        super().__init__('auto_spawn_trigger')
        
        # Wait for object spawner service
        self.spawn_client = self.create_client(Empty, '/spawn_random_objects')
        
        # Wait for service and then spawn
        self.timer = self.create_timer(2.0, self.check_and_spawn)
        self.spawned = False
        
        self.get_logger().info("Auto spawn trigger initialized")
    
    def check_and_spawn(self):
        if not self.spawned and self.spawn_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("🎯 Auto-spawning random objects...")
            
            request = Empty.Request()
            future = self.spawn_client.call_async(request)
            
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info("✅ Successfully spawned random objects")
            else:
                self.get_logger().error("❌ Failed to spawn objects")
                
            self.spawned = True
            self.timer.cancel()
            
            # Shutdown this node after spawning
            self.get_logger().info("🔚 Auto-spawn trigger shutting down")
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutoSpawnTrigger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.context.ok():
            return
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()