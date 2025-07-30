#include <functional>
#include <memory>
#include <thread>
#include <map>

#include "custom_action_interfaces/action/ur5.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

namespace custom_action_server
{
    class UR5ActionServer : public rclcpp::Node
    {
        public:
        using UR5 = custom_action_interfaces::action::UR5; // Using the action
        using GoalHandleUR5 = rclcpp_action::ServerGoalHandle<UR5>; // Handle the goal

        explicit UR5ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("ur5_action_server", options){ // Constuctor for the UR5ActionServer
            
            using namespace std::placeholders;

            publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/scaled_joint_trajectory_controller/joint_trajectory", 10);
            joints_map_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>("/scaled_joint_trajectory_controller/controller_state", 10, std::bind(&UR5ActionServer::joint_state_callback, this, std::placeholders::_1));

            auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const UR5::Goal> goal){
                RCLCPP_INFO(this->get_logger(), "Received goal request with movement instruction");

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
            rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joints_map_;
            rclcpp::Time start_arm_move;
            rclcpp::Time goal_received;

            bool joint_callback_active = false;

            std::map<std::string, std::vector<double>> commands_map = {
                {"default", {-1.599998, -1.719986, -2.199983, -0.810036, 1.599995, -0.029998}},
                {"pick_up", {-1.41, -0.96, -1.8, -1.96, -1.6, 0.0}},
                {"pick_front", {0.1, -0.8, -1.6, -2.5, -0.5, -3.13}},
                {"pick_side", {-0.3, -0.96, -2.2, -1, -1.6, 1.57}}
            };

            std::shared_ptr<UR5::Result> result;
            std::shared_ptr<UR5::Feedback> feedback;
            std::shared_ptr<GoalHandleUR5> goal_handle;

            void init(const std::shared_ptr<GoalHandleUR5> gh){
                goal_handle = gh;
                feedback = std::make_shared<UR5::Feedback>();
                result = std::make_shared<UR5::Result>();
            }

            void execute(const std::shared_ptr<GoalHandleUR5> gh){
                
                init(gh);

                const auto goal = goal_handle->get_goal(); // Obtain goal

                goal_received = now();
                feedback->status = "GOAL_RECEIVED";
                goal_handle->publish_feedback(feedback);

                trajectory_msgs::msg::JointTrajectory msg;
                trajectory_msgs::msg::JointTrajectoryPoint p;

                RCLCPP_INFO(this->get_logger(), "Executing goal");

                msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
                p.positions = commands_map[goal->execute];
                p.time_from_start = rclcpp::Duration::from_seconds(2.0);

                msg.points.push_back(p);
                publisher_->publish(msg);
                start_arm_move = now();
                feedback->status = "ARM_MOVING";
                goal_handle->publish_feedback(feedback);
                joint_callback_active = true;

                RCLCPP_INFO(this->get_logger(), "Trajectory sent to robot");
            }

            void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState msg){

                if (!joint_callback_active){
                    return;
                }

                const auto& act = msg.feedback.positions; // Use ros2 interface show control_msgs/msg/JointTrajectoryControllerState to see more info
                const auto& des = msg.reference.positions;
                double max_error = 0;
                double error;

                for (long unsigned int i = 0; i < des.size(); i++){
                    error = abs(act[i] - des[i]);
                    if (error > max_error){
                        max_error = error;
                    }
                }

                if (max_error <= 0.01){
                    result->duration_move = (now() - start_arm_move).nanoseconds();
                    result->duration_goal_move = (start_arm_move - goal_received).nanoseconds();
                    joint_callback_active = false;

                    feedback->status = "GOAL_ENDED_SUCCESS";
                    goal_handle->publish_feedback(feedback);

                    result->success = true;
                    goal_handle->succeed(result);
                }
            }

    }; // class UR5ActionServer

} // namespace custom_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_server::UR5ActionServer)