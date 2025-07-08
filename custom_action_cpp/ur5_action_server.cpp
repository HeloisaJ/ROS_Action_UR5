#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/ur5.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace custom_action_cpp
{
    class UR5ActionServer : public rclcpp::Node
    {
        public:
        using UR5 = custom_action_interfaces::action::UR5; // Using the action
        using GoalHandleUR5 = rclcpp_action::ServerGoalHandle<UR5>; // Handle the goal

        explicit UR5ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("ur5_action_server", options){ // Constuctor for the UR5ActionServer
            
            using namespace std::placeholders;

            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);

            auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const UR5::Goal> goal){
                std::vector<double> joints = goal->joints;

                for(long unsigned int i = 0; i < joints.size(); i++){
                    RCLCPP_INFO(this->get_logger(), "Received goal request with movement instuction %ld of %f", i, joints[i]);
                }
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto handle_cancel = [this](const std::shared_ptr<GoalHandleUR5> goal_handle){

                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto handle_accepted = [this](const std::shared_ptr<GoalHandleUR5> goal_handle){

                // this needs to return quickly to avoid blocking the executor,
                // so we declare a lambda function to be called inside a new thread
                auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
                std::thread{execute_in_thread}.detach();
            };

            this->action_server_ = rclcpp_action::create_server<UR5>(
                this,
                "ur5_move",
                handle_goal,
                handle_cancel,
                handle_accepted);
        }

        private:
            rclcpp_action::Server<UR5>::SharedPtr action_server_;
            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

            void execute(const std::shared_ptr<GoalHandleUR5> goal_handle){
                RCLCPP_INFO(this->get_logger(), "Executing goal");

                const auto goal = goal_handle->get_goal(); // Goal values
                auto feedback = std::make_shared<UR5::Feedback>(); 
                auto result = std::make_shared<UR5::Result>();
                trajectory_msgs::msg::JointTrajectory msg;
                trajectory_msgs::msg::JointTrajectoryPoint p;

                std::vector<double> joints = goal->joints;

                for(double j: joints){
                    if (goal_handle->is_canceling()) {
                        result->success = false;
                        feedback->status = "GOAL_CANCELED";
                        return;
                    }

                    if (std::isnan(j)){
                        RCLCPP_WARN(this->get_logger(), "Joint state contains NaN values");
                        result->success = false;
                        feedback->status = "GOAL_ABORTED";
                        return;
                    }

                    p.positions.push_back(j);
                }

                msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
                p.time_from_start = rclcpp::Duration::from_seconds(2.0);

                msg.points.push_back(p);
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Trajectory sent to robot");
            }

    }; // class UR5ActionServer

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::UR5ActionServer)