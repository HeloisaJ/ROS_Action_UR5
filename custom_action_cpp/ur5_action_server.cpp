#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/ur5.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"

namespace custom_action_cpp
{
    class UR5ActionServer : public rclcpp::Node
    {
        public:
        using UR5 = custom_action_interfaces::action::UR5; // Using the action
        using GoalHandleUR5 = rclcpp_action::ServerGoalHandle<UR5>; // Handle the goal

        CUSTOM_ACTION_CPP_PUBLIC // Constuctor for the UR5ActionServer
        explicit UR5ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("ur5_action_server", options){
            
            using namespace std::placeholders;

            auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const UR5::Goal> goal){

                RCLCPP_INFO(this->get_logger(), "Received goal request with movement instuctions of %f %f %f %f %f %f", goal->shoulder_pan_joint, goal->shoulder_lift_joint, goal->elbow_joint, goal->wrist_1_joint, goal->wrist_2_joint, goal->wrist_3_joint);
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

            void execute(const std::shared_ptr<GoalHandleUR5> goal_handle){
                
            }

    }; // class UR5ActionServer

} // namespace custom_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::UR5ActionServer)