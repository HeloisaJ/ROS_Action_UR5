#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "custom_action_interfaces/action/ur5.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/float64.hpp"

namespace custom_action_client
{
    class UR5ActionClient : public rclcpp::Node
    {
        public:
        using UR5 = custom_action_interfaces::action::UR5; // Using the action
        using GoalHandleUR5 = rclcpp_action::ClientGoalHandle<UR5>; // Handle the goal

        rclcpp::Time goal_sent;
        long int delay_client_server;

        explicit UR5ActionClient(const std::string & goal, const rclcpp::NodeOptions & options): Node("ur5_action_client", options){
            this->client_ptr_ = rclcpp_action::create_client<UR5>(
            this,
            "ur5_move");

            auto timer_callback_lambda = [this, goal](){ return this->send_goal(goal); };
            this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            timer_callback_lambda);
        }

        void send_goal(const std::string & goal){
            using namespace std::placeholders;

            this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = UR5::Goal();
            goal_msg.execute = goal;

            goal_sent = now();
            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<UR5>::SendGoalOptions();
            send_goal_options.goal_response_callback = [this](const GoalHandleUR5::SharedPtr & goal_handle)
            {
                if(!goal_handle){
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } 
                else{
                    delay_client_server = (now() - goal_sent).nanoseconds();

                    std::stringstream ss;
                    ss << "Goal accepted by server";
                    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                }
            };

            send_goal_options.feedback_callback = [this](
            GoalHandleUR5::SharedPtr,
            const std::shared_ptr<const UR5::Feedback> feedback)
            {
                std::stringstream ss;
                ss << "Status: " << feedback->status << "\n";
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            };

            send_goal_options.result_callback = [this](const GoalHandleUR5::WrappedResult & result)
            {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                    case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    return;
                    case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    return;
                    default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
                }

                std::stringstream ss;
                ss << "Duration of Arm Movement: " << result.result->duration_move << "\n";
                ss << "Duration between Goal Received and Start Movement: " << result.result->duration_goal_move << "\n";
                ss << "Command Delay Server to Client: " << delay_client_server << "\n";
            
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                rclcpp::shutdown();
            };
        }

        private:
        rclcpp_action::Client<UR5>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
    }; // class UR5ActionClient
} // namespace custom_action_client

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);

    std::string goal = argv[0];

    rclcpp::spin(std::make_shared<custom_action_client::UR5ActionClient>(goal, rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}