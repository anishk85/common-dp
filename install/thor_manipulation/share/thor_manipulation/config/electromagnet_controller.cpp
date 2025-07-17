#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>

namespace gazebo
{
  class ElectromagnetController : public ModelPlugin
  {
    public:
      ElectromagnetController() : ModelPlugin()
      {
      }

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Store the model pointer
        this->model = _model;
        
        // Initialize ROS node
        this->ros_node = gazebo_ros::Node::Get(_sdf);
        
        // Get electromagnet parameters from SDF
        if (_sdf->HasElement("electromagnet_link"))
        {
          this->electromagnet_link_name = _sdf->Get<std::string>("electromagnet_link");
        }
        else
        {
          this->electromagnet_link_name = "electromagnet_plate";
        }
        
        if (_sdf->HasElement("max_force"))
        {
          this->max_force = _sdf->Get<double>("max_force");
        }
        else
        {
          this->max_force = 50.0;  // Default force in Newtons
        }
        
        if (_sdf->HasElement("max_distance"))
        {
          this->max_distance = _sdf->Get<double>("max_distance");
        }
        else
        {
          this->max_distance = 0.05;  // Default max distance in meters
        }
        
        // Get the electromagnet link
        this->electromagnet_link = this->model->GetLink(this->electromagnet_link_name);
        if (!this->electromagnet_link)
        {
          RCLCPP_ERROR(this->ros_node->get_logger(), 
                       "Electromagnet link '%s' not found", 
                       this->electromagnet_link_name.c_str());
          return;
        }
        
        // Subscribe to electromagnet control topic
        this->electromagnet_sub = this->ros_node->create_subscription<std_msgs::msg::Bool>(
          "/electromagnet_control", 10,
          std::bind(&ElectromagnetController::OnElectromagnetControl, this, std::placeholders::_1));
        
        // Connect to the world update event
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ElectromagnetController::OnUpdate, this));
        
        // Initialize state
        this->electromagnet_active = false;
        this->attached_object = nullptr;
        
        RCLCPP_INFO(this->ros_node->get_logger(), 
                    "Electromagnet controller initialized for link: %s", 
                    this->electromagnet_link_name.c_str());
      }
      
      void OnElectromagnetControl(const std_msgs::msg::Bool::SharedPtr msg)
      {
        this->electromagnet_active = msg->data;
        
        if (this->electromagnet_active)
        {
          RCLCPP_INFO(this->ros_node->get_logger(), "Electromagnet activated");
          this->AttachNearestObject();
        }
        else
        {
          RCLCPP_INFO(this->ros_node->get_logger(), "Electromagnet deactivated");
          this->DetachObject();
        }
      }
      
      void OnUpdate()
      {
        if (!this->electromagnet_active || !this->attached_object)
          return;
        
        // Apply attractive force to keep object attached
        this->ApplyMagneticForce();
      }
      
    private:
      void AttachNearestObject()
      {
        if (this->attached_object)
          return;  // Already attached to an object
        
        // Get electromagnet position
        ignition::math::Vector3d electromagnet_pos = 
          this->electromagnet_link->WorldPose().Pos();
        
        // Find nearest magnetic object
        physics::WorldPtr world = this->model->GetWorld();
        physics::Model_V models = world->Models();
        
        double min_distance = this->max_distance;
        physics::ModelPtr nearest_object = nullptr;
        
        for (auto& model : models)
        {
          // Skip if it's the same model as the arm
          if (model == this->model)
            continue;
          
          // Check if object has magnetic properties (simple check by name)
          std::string model_name = model->GetName();
          if (model_name.find("can") != std::string::npos ||
              model_name.find("box") != std::string::npos ||
              model_name.find("sphere") != std::string::npos ||
              model_name.find("cylinder") != std::string::npos)
          {
            ignition::math::Vector3d object_pos = model->WorldPose().Pos();
            double distance = electromagnet_pos.Distance(object_pos);
            
            if (distance < min_distance)
            {
              min_distance = distance;
              nearest_object = model;
            }
          }
        }
        
        if (nearest_object)
        {
          this->attached_object = nearest_object;
          RCLCPP_INFO(this->ros_node->get_logger(), 
                      "Attached to object: %s", 
                      nearest_object->GetName().c_str());
        }
        else
        {
          RCLCPP_WARN(this->ros_node->get_logger(), 
                      "No magnetic object found within range");
        }
      }
      
      void DetachObject()
      {
        if (this->attached_object)
        {
          RCLCPP_INFO(this->ros_node->get_logger(), 
                      "Detached from object: %s", 
                      this->attached_object->GetName().c_str());
          this->attached_object = nullptr;
        }
      }
      
      void ApplyMagneticForce()
      {
        if (!this->attached_object)
          return;
        
        // Get positions
        ignition::math::Vector3d electromagnet_pos = 
          this->electromagnet_link->WorldPose().Pos();
        ignition::math::Vector3d object_pos = 
          this->attached_object->WorldPose().Pos();
        
        // Calculate force direction (towards electromagnet)
        ignition::math::Vector3d force_direction = 
          electromagnet_pos - object_pos;
        
        double distance = force_direction.Length();
        
        if (distance > this->max_distance)
        {
          // Object is too far, detach it
          this->DetachObject();
          return;
        }
        
        // Normalize force direction
        force_direction.Normalize();
        
        // Calculate force magnitude (inverse square law, but clamped)
        double force_magnitude = this->max_force;
        if (distance > 0.001)  // Avoid division by zero
        {
          force_magnitude = std::min(this->max_force, 
                                   this->max_force / (distance * distance * 100));
        }
        
        // Apply force to object
        ignition::math::Vector3d force = force_direction * force_magnitude;
        
        // Get the first link of the attached object
        physics::LinkPtr object_link = this->attached_object->GetLinks()[0];
        if (object_link)
        {
          object_link->AddForce(force);
        }
        
        // Apply equal and opposite force to electromagnet (Newton's third law)
        this->electromagnet_link->AddForce(-force);
      }
      
      // Pointer to the model
      physics::ModelPtr model;
      
      // Pointer to the electromagnet link
      physics::LinkPtr electromagnet_link;
      
      // ROS node
      gazebo_ros::Node::SharedPtr ros_node;
      
      // ROS subscriber
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr electromagnet_sub;
      
      // Connection to the world update event
      event::ConnectionPtr update_connection;
      
      // Electromagnet parameters
      std::string electromagnet_link_name;
      double max_force;
      double max_distance;
      
      // State variables
      bool electromagnet_active;
      physics::ModelPtr attached_object;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ElectromagnetController)
}