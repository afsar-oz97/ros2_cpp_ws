#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_navigation/action/navigation.hpp"
#include <iostream>

typedef robot_navigation::action::Navigation Navigation;
typedef rclcpp_action::ClientGoalHandle<Navigation> GoalHandleNavigation;
using geometry_msgs::msg::Point;

class NavigationActionClientNode : public rclcpp::Node
{
public:
    NavigationActionClientNode() : Node("navigation_action_client")
    {
        action_client_ = rclcpp_action::create_client<Navigation>(this, "navigate");
        std::cout << "Navigation action client started." << std::endl;
        prompt_for_goal();
    }

private:
    void prompt_for_goal()
    {
        Point goal_position;
        std::cout << "Enter goal position x: ";
        std::cin >> goal_position.x;
        std::cout << "Enter goal position y: ";
        std::cin >> goal_position.y;
        std::cout << "Enter goal position z: ";
        std::cin >> goal_position.z;

        auto goal_msg = Navigation::Goal();
        goal_msg.goal_position = goal_position;

        //--------------
        this->action_client_->wait_for_action_server();
        std::cout << "Action server is ready, sending goal..." << std::endl;

        auto send_goal_options = rclcpp_action::Client<Navigation>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&NavigationActionClientNode::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.feedback_callback = std::bind(
            &NavigationActionClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&NavigationActionClientNode::result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }
    void goal_response_callback(const GoalHandleNavigation::SharedPtr& goal_handle)
    {
        if (!goal_handle) { std::cout << "Goal was rejected by the action server." << std::endl; }
        else
        {
            std::cout << "Goal accepted by the action server." << std::endl;
        }
    }

    void feedback_callback(
        const GoalHandleNavigation::SharedPtr& goal_handle, const std::shared_ptr<const Navigation::Feedback> feedback)
    {
        if (goal_handle)
        {
            std::cout << "Feedback received: Distance to goal = " << feedback->distance_to_goal << std::endl;
        }
        else
        {
            std::cout << "Feedback received for an invalid goal handle." << std::endl;
        }
    }

    void result_callback(const GoalHandleNavigation::WrappedResult& result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                std::cout << "Time elapsed: " << result.result->time_elapsed << " seconds" << std::endl;
                std::cout << "Goal succeeded!" << std::endl;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                std::cout << "Goal was canceled." << std::endl;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                std::cout << "Goal was aborted." << std::endl;
                break;
            default:
                std::cout << "Unknown result code." << std::endl;
                break;
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<Navigation>::SharedPtr action_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationActionClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}