#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <termios.h>
#include <unistd.h>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

// Key mapping for pose control
const std::map<char, std::pair<std::string, double>> KEY_MAP = {
    {'w', {"x", 0.02}}, {'s', {"x", -0.02}},
    {'a', {"y", 0.02}}, {'d', {"y", -0.02}},
    {'r', {"z", 0.02}}, {'f', {"z", -0.02}},
    {'q', {"roll", 0.1}}, {'e', {"roll", -0.1}},
    {'t', {"pitch", 0.1}}, {'g', {"pitch", -0.1}},
    {'y', {"yaw", 0.1}}, {'h', {"yaw", -0.1}}
};

class TeleopPoseControl : public rclcpp::Node
{
public:
    TeleopPoseControl() : Node("teleop_pose_control")
    {
        // ROS Communications
        move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/move_action");
        electromagnet_pub_ = this->create_publisher<std_msgs::msg::Bool>("/thor_arm/electromagnet/control", 10);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "🤖 Thor Pose (Cartesian) Teleop Controller Started.");
        print_instructions();

        // Start keyboard listener in a separate thread
        keyboard_thread_ = std::thread(&TeleopPoseControl::keyboard_loop, this);
    }

    ~TeleopPoseControl()
    {
        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
    }

private:
    // ROS Components
    rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr electromagnet_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_;
    std::atomic<bool> motion_in_progress_{false};
    std::atomic<bool> electromagnet_state_{false};
    std::mutex pose_mutex_;
    std::thread keyboard_thread_;

    // Parameters
    const std::string move_group_name_ = "arm_group";
    const std::string end_effector_link_ = "electromagnet_plate";
    const std::string base_frame_ = "world";

    void print_instructions()
    {
        RCLCPP_INFO(this->get_logger(), "\n"
            "==================================================\n"
            "        Thor Pose (Cartesian) Teleop Control\n"
            "==================================================\n"
            "  Position (XYZ):\n"
            "    Fwd/Bwd [w]/[s]   Left/Right [a]/[d]   Up/Down [r]/[f]\n"
            "\n"
            "  Orientation (Roll, Pitch, Yaw):\n"
            "    Roll: [q]/[e]   Pitch: [t]/[g]   Yaw: [y]/[h]\n"
            "\n"
            "  End Effector:\n"
            "    [spacebar]: Toggle Electromagnet\n"
            "\n"
            "  [Ctrl+C] to quit\n"
            "==================================================");
    }

    char get_key()
    {
        struct termios old_tio, new_tio;
        tcgetattr(STDIN_FILENO, &old_tio);
        new_tio = old_tio;
        new_tio.c_lflag &= (~ICANON & ~ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

        char c = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        return c;
    }

    void keyboard_loop()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt server...");
        move_group_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "✅ MoveIt server is ready.");

        get_current_pose();

        while (rclcpp::ok())
        {
            char key = get_key();
            if (key == '\x03') // Ctrl+C
            {
                rclcpp::shutdown();
                break;
            }

            if (motion_in_progress_.load())
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ Motion in progress, please wait.");
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (!current_pose_)
                {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Current pose not yet available.");
                    continue;
                }
            }
            
            motion_in_progress_.store(true);

            if (KEY_MAP.count(key))
            {
                const auto& [axis, increment] = KEY_MAP.at(key);
                update_pose(axis, increment);
                move_to_pose();
            }
            else if (key == ' ')
            {
                toggle_electromagnet();
                motion_in_progress_.store(false);
            }
            else
            {
                motion_in_progress_.store(false);
            }
        }
    }

    void get_current_pose()
    {
        RCLCPP_INFO(this->get_logger(), "Getting current end-effector pose...");
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = nullptr;
        }

        while (!current_pose_ && rclcpp::ok())
        {
            try
            {
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    base_frame_, end_effector_link_, tf2::TimePointZero);
                
                std::lock_guard<std::mutex> lock(pose_mutex_);
                current_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
                current_pose_->header = t.header;
                current_pose_->pose.position.x = t.transform.translation.x;
                current_pose_->pose.position.y = t.transform.translation.y;
                current_pose_->pose.position.z = t.transform.translation.z;
                current_pose_->pose.orientation = t.transform.rotation;

                RCLCPP_INFO(this->get_logger(), "✅ Pose acquired: P(%.2f, %.2f, %.2f)",
                    current_pose_->pose.position.x, current_pose_->pose.position.y, current_pose_->pose.position.z);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not get transform, retrying: %s", ex.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    void update_pose(const std::string& axis, double increment)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        auto& p = current_pose_->pose.position;
        auto& o = current_pose_->pose.orientation;

        if (axis == "x") p.x += increment;
        else if (axis == "y") p.y += increment;
        else if (axis == "z") p.z += increment;
        else if (axis == "roll" || axis == "pitch" || axis == "yaw")
        {
            tf2::Quaternion current_q;
            tf2::fromMsg(o, current_q);

            tf2::Quaternion delta_q;
            if (axis == "roll") delta_q.setRPY(increment, 0, 0);
            if (axis == "pitch") delta_q.setRPY(0, increment, 0);
            if (axis == "yaw") delta_q.setRPY(0, 0, increment);

            current_q = current_q * delta_q;
            current_q.normalize();
            o = tf2::toMsg(current_q);
        }
        RCLCPP_INFO(this->get_logger(), "➡️ New Target: P(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
    }

    void toggle_electromagnet()
    {
        electromagnet_state_ = !electromagnet_state_.load();
        auto msg = std_msgs::msg::Bool();
        msg.data = electromagnet_state_.load();
        electromagnet_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🧲 Electromagnet toggled %s", electromagnet_state_.load() ? "ON" : "OFF");
    }

    void move_to_pose()
    {
        auto goal_msg = moveit_msgs::action::MoveGroup::Goal();
        auto& request = goal_msg.request;
        request.group_name = move_group_name_;
        request.num_planning_attempts = 5;
        request.allowed_planning_time = 5.0;

        moveit_msgs::msg::Constraints constraints;
        
        // Position Constraint
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions.push_back(0.01); // 1cm radius

        moveit_msgs::msg::PositionConstraint pos_constraint;
        pos_constraint.header.frame_id = base_frame_;
        pos_constraint.link_name = end_effector_link_;
        pos_constraint.constraint_region.primitives.push_back(sphere);
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pos_constraint.constraint_region.primitive_poses.push_back(current_pose_->pose);
        }
        pos_constraint.weight = 1.0;
        constraints.position_constraints.push_back(pos_constraint);

        // Orientation Constraint
        moveit_msgs::msg::OrientationConstraint orient_constraint;
        orient_constraint.header.frame_id = base_frame_;
        orient_constraint.link_name = end_effector_link_;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            orient_constraint.orientation = current_pose_->pose.orientation;
        }
        orient_constraint.absolute_x_axis_tolerance = 0.2;
        orient_constraint.absolute_y_axis_tolerance = 0.2;
        orient_constraint.absolute_z_axis_tolerance = 0.2;
        orient_constraint.weight = 1.0;
        constraints.orientation_constraints.push_back(orient_constraint);

        request.goal_constraints.push_back(constraints);
        
        using namespace std::placeholders;
        auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TeleopPoseControl::goal_response_callback, this, _1);
        send_goal_options.result_callback = std::bind(&TeleopPoseControl::result_callback, this, _1);
        
        move_group_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr& goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ Goal was rejected by server");
            motion_in_progress_.store(false);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
        }
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "✅ Motion succeeded.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Motion failed with error code: %d", result.result->error_code.val);
        }

        RCLCPP_INFO(this->get_logger(), "Re-acquiring current pose from TF to re-sync.");
        get_current_pose();
        motion_in_progress_.store(false);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopPoseControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
