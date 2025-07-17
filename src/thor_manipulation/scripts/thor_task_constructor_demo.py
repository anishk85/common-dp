#!/usr/bin/env python3
# filepath: /home/anish/dp_ws/Thor-ROS/ws_thor/src/thor_manipulation/scripts/thor_task_constructor_demo.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit.task_constructor import core, stages
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from tf_transformations import quaternion_from_euler
import numpy as np

class ThorTaskConstructorDemo(Node):
    def __init__(self):
        super().__init__('thor_task_constructor_demo')
        
        # Initialize MoveIt
        self.moveit = MoveItPy(node_name="thor_moveit_py")
        self.robot_model = self.moveit.get_robot_model()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/task_command', self.command_callback, 10)
        self.object_sub = self.create_subscription(
            PoseStamped, '/detected_objects', self.object_callback, 10)
        
        # Task Constructor parameters
        self.arm_group = "arm_group"
        self.eef_name = "electromagnet"
        self.eef_group = "electromagnet"  # If you have a separate group for electromagnet
        self.eef_frame = "electromagnet_plate"
        
        # Object information
        self.object_pose = None
        self.object_name = "target_object"
        
        # Task
        self.task = None
        
        self.get_logger().info("🤖 Thor Task Constructor Demo initialized")
        self.get_logger().info("Commands: 'demo', 'pick_place', 'plan_only'")

    def object_callback(self, msg):
        """Store detected object pose"""
        self.object_pose = msg.pose
        self.get_logger().info(f"📦 Object detected for task planning")

    def command_callback(self, msg):
        """Handle task commands"""
        command = msg.data.lower()
        
        if command == "demo" or command == "pick_place":
            if self.object_pose is None:
                self.get_logger().warn("⚠️ No object detected. Spawn and detect an object first.")
                return
            self.create_pick_place_task()
        elif command == "plan_only":
            if self.object_pose is None:
                self.get_logger().warn("⚠️ No object detected. Spawn and detect an object first.")
                return
            self.create_pick_place_task(plan_only=True)
        else:
            self.get_logger().warn(f"❌ Unknown command: {command}")

    def create_pick_place_task(self, plan_only=False):
        """Create a pick and place task using MoveIt Task Constructor"""
        try:
            self.get_logger().info("🚀 Creating pick and place task...")
            
            # Create the task
            task = core.Task("thor_pick_place")
            
            # Set the robot model
            task.setRobotModel(self.robot_model)
            
            # Create stages container
            task.stages = core.SerialContainer("pick_place_sequence")
            
            # Get current state
            current_state = stages.CurrentState("current_state")
            task.stages.add(current_state)
            
            # Stage 1: Move to ready position
            ready_stage = stages.MoveTo("move_to_ready", self.moveit.get_planning_component(self.arm_group))
            ready_stage.setGoal("ready")  # Assumes you have a "ready" named pose
            task.stages.add(ready_stage)
            
            # Stage 2: Open gripper (prepare electromagnet)
            open_gripper = stages.ModifyPlanningScene("open_gripper")
            task.stages.add(open_gripper)
            
            # Stage 3: Connect pick stages
            pick_stage_wrapper = core.SerialContainer("pick_object")
            
            # Approach object
            approach = stages.MoveRelative("approach_object", self.moveit.get_planning_component(self.arm_group))
            approach.properties.configureInitFrom(core.Stage.PARENT, ["group"])
            approach.setMinMaxDistance(0.1, 0.15)
            approach.setIKFrame(self.eef_frame)
            
            # Set approach direction
            direction = Vector3Stamped()
            direction.header.frame_id = self.eef_frame
            direction.vector.z = -1.0  # Approach from above
            approach.setDirection(direction)
            pick_stage_wrapper.add(approach)
            
            # Generate grasp pose
            generate_grasp_pose = stages.GenerateGraspPose("generate_grasp_pose")
            generate_grasp_pose.properties.configureInitFrom(core.Stage.PARENT)
            generate_grasp_pose.setPreGraspPose("open")  # Assumes you have an "open" pose
            generate_grasp_pose.setObject(self.object_name)
            generate_grasp_pose.setAngleDelta(np.pi / 12)  # 15 degree increments
            
            # Set grasp pose relative to object
            grasp_pose = self.create_grasp_pose_relative_to_object()
            generate_grasp_pose.setGraspPose(grasp_pose)
            pick_stage_wrapper.add(generate_grasp_pose)
            
            # Allow collision with object during grasp
            allow_collision = stages.ModifyPlanningScene("allow_collision_grasp")
            allow_collision.allowCollisions(
                self.object_name,
                task.getRobotModel().getJointModelGroup(self.eef_group).getLinkModelNames(),
                True
            )
            pick_stage_wrapper.add(allow_collision)
            
            # Close gripper (activate electromagnet)
            close_gripper = stages.MoveTo("close_gripper", self.moveit.get_planning_component(self.eef_group))
            close_gripper.setGoal("closed")  # Assumes you have a "closed" pose
            pick_stage_wrapper.add(close_gripper)
            
            # Attach object
            attach_object = stages.ModifyPlanningScene("attach_object")
            attach_object.attachObject(self.object_name, self.eef_frame)
            pick_stage_wrapper.add(attach_object)
            
            # Lift object
            lift = stages.MoveRelative("lift_object", self.moveit.get_planning_component(self.arm_group))
            lift.properties.configureInitFrom(core.Stage.PARENT, ["group"])
            lift.setMinMaxDistance(0.1, 0.3)
            lift.setIKFrame(self.eef_frame)
            
            # Set lift direction
            lift_direction = Vector3Stamped()
            lift_direction.header.frame_id = "base_link"
            lift_direction.vector.z = 1.0  # Lift upward
            lift.setDirection(lift_direction)
            pick_stage_wrapper.add(lift)
            
            # Add pick stages to main task
            task.stages.add(pick_stage_wrapper)
            
            # Stage 4: Move to place location
            place_stage_wrapper = core.SerialContainer("place_object")
            
            # Approach place location
            approach_place = stages.MoveTo("approach_place", self.moveit.get_planning_component(self.arm_group))
            place_pose = self.create_place_pose()
            approach_place.setGoal(place_pose)
            place_stage_wrapper.add(approach_place)
            
            # Generate place pose
            generate_place_pose = stages.GeneratePlacePose("generate_place_pose")
            generate_place_pose.properties.configureInitFrom(core.Stage.PARENT)
            generate_place_pose.setObject(self.object_name)
            
            # Set place location
            place_location = self.create_place_location()
            generate_place_pose.setPlacePose(place_location)
            place_stage_wrapper.add(generate_place_pose)
            
            # Lower object
            lower = stages.MoveRelative("lower_object", self.moveit.get_planning_component(self.arm_group))
            lower.properties.configureInitFrom(core.Stage.PARENT, ["group"])
            lower.setMinMaxDistance(0.05, 0.1)
            lower.setIKFrame(self.eef_frame)
            
            # Set lower direction
            lower_direction = Vector3Stamped()
            lower_direction.header.frame_id = "base_link"
            lower_direction.vector.z = -1.0  # Lower downward
            lower.setDirection(lower_direction)
            place_stage_wrapper.add(lower)
            
            # Detach object
            detach_object = stages.ModifyPlanningScene("detach_object")
            detach_object.detachObject(self.object_name, self.eef_frame)
            place_stage_wrapper.add(detach_object)
            
            # Open gripper (deactivate electromagnet)
            open_gripper_place = stages.MoveTo("open_gripper", self.moveit.get_planning_component(self.eef_group))
            open_gripper_place.setGoal("open")
            place_stage_wrapper.add(open_gripper_place)
            
            # Retreat
            retreat = stages.MoveRelative("retreat", self.moveit.get_planning_component(self.arm_group))
            retreat.properties.configureInitFrom(core.Stage.PARENT, ["group"])
            retreat.setMinMaxDistance(0.1, 0.2)
            retreat.setIKFrame(self.eef_frame)
            
            # Set retreat direction
            retreat_direction = Vector3Stamped()
            retreat_direction.header.frame_id = self.eef_frame
            retreat_direction.vector.z = 1.0  # Retreat upward
            retreat.setDirection(retreat_direction)
            place_stage_wrapper.add(retreat)
            
            # Add place stages to main task
            task.stages.add(place_stage_wrapper)
            
            # Stage 5: Return home
            return_home = stages.MoveTo("return_home", self.moveit.get_planning_component(self.arm_group))
            return_home.setGoal("home")  # Assumes you have a "home" named pose
            task.stages.add(return_home)
            
            # Store task
            self.task = task
            
            # Plan the task
            self.get_logger().info("📋 Planning task...")
            self.publish_status("Planning task...")
            
            success = task.plan(max_solutions=10)
            
            if success:
                self.get_logger().info(f"✅ Task planned successfully! Found {len(task.solutions)} solutions")
                self.publish_status(f"Task planned - {len(task.solutions)} solutions found")
                
                if not plan_only:
                    # Execute the best solution
                    self.get_logger().info("🎯 Executing task...")
                    self.publish_status("Executing task...")
                    
                    success = task.execute(task.solutions[0])
                    
                    if success:
                        self.get_logger().info("🎉 Task executed successfully!")
                        self.publish_status("Task completed successfully")
                    else:
                        self.get_logger().error("❌ Task execution failed")
                        self.publish_status("Task execution failed")
                else:
                    self.get_logger().info("📋 Planning complete (plan_only mode)")
                    self.publish_status("Planning complete (plan only)")
            else:
                self.get_logger().error("❌ Task planning failed")
                self.publish_status("Task planning failed")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error in task creation: {str(e)}")
            self.publish_status(f"Task error: {str(e)}")

    def create_grasp_pose_relative_to_object(self):
        """Create grasp pose relative to object"""
        # This would return a geometry_msgs/PoseStamped
        # relative to the object frame
        pose = PoseStamped()
        pose.header.frame_id = self.object_name
        
        # Position relative to object center
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.02  # Slightly above object
        
        # Orientation pointing down
        q = quaternion_from_euler(0, np.pi, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

    def create_place_pose(self):
        """Create place pose"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        
        # Place location
        pose.pose.position.x = -0.2
        pose.pose.position.y = 0.3
        pose.pose.position.z = 0.15
        
        # Orientation pointing down
        q = quaternion_from_euler(0, np.pi, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

    def create_place_location(self):
        """Create place location for object placement"""
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        
        # Place location on surface
        pose.pose.position.x = -0.2
        pose.pose.position.y = 0.3
        pose.pose.position.z = 0.02  # On table surface
        
        # Default orientation
        pose.pose.orientation.w = 1.0
        
        return pose

    def publish_status(self, status):
        """Publish task status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = ThorTaskConstructorDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()