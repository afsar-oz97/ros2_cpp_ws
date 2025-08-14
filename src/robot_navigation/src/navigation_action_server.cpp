#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_navigation/action/navigation.hpp"
#include <iostream>

typedef robot_navigation::action::Navigation Navigation;
typedef rclcpp_action::ServerGoalHandle<Navigation> GoalHandleNavigation;
using geometry_msgs::msg::Point;
const float DIST_THRESHOLD = 0.1;    // Distance threshold to consider goal reached

class NavigationActionServerNode : public rclcpp::Node
{
public:
    NavigationActionServerNode() : Node("navigation_action_server")
    {
        robot_position_ = Point();
        robot_position_subcriber_ = this->create_subscription<Point>("robot_position", 10,
            std::bind(&NavigationActionServerNode::robotPositionCallback, this, std::placeholders::_1));
        action_server_ = rclcpp_action::create_server<Navigation>(this,
            "navigate",
            std::bind(&NavigationActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationActionServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&NavigationActionServerNode::handle_accepted, this, std::placeholders::_1));
        std::cout << "Navigation action server started." << std::endl;
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Navigation::Goal> goal)
    {
        (void)uuid;    // Suppress unused variable warning
        std::cout << "Received goal request, coordinate are: " << std::endl;
        std::cout << "X: " << goal->goal_position.x << ", Y: " << goal->goal_position.y
                  << ", Z: " << goal->goal_position.z << std::endl;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        (void)goal_handle;    // Suppress unused variable warning
        std::cout << "Received request to cancel goal." << std::endl;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        std::thread{std::bind(&NavigationActionServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigation> goal_handle)
    {
        std::cout << "Executing goal..." << std::endl;
        auto start_time = rclcpp::Clock().now();

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Navigation::Feedback>();
        auto result = std::make_shared<Navigation::Result>();
        rclcpp::Rate loop_rate(1);    // 1 Hz // Use rclcpp::Rate for reliable, consistent loop timing in ROS nodes.

        feedback->distance_to_goal = DIST_THRESHOLD;    // Just to enter the loop
        while (feedback->distance_to_goal >= DIST_THRESHOLD)
        {
            feedback->distance_to_goal = std::sqrt(std::pow(goal->goal_position.x - robot_position_.x, 2) +
                                                   std::pow(goal->goal_position.y - robot_position_.y, 2) +
                                                   std::pow(goal->goal_position.z - robot_position_.z, 2));
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        result->time_elapsed = (rclcpp::Clock().now() - start_time).seconds();
        std::cout << "Goal reached! Time elapsed: " << result->time_elapsed << " seconds." << std::endl;
        goal_handle->succeed(result);
    }

    void robotPositionCallback(const Point& msg) { robot_position_ = msg; }

    rclcpp_action::Server<Navigation>::SharedPtr action_server_;
    Point robot_position_;
    rclcpp::Subscription<Point>::SharedPtr robot_position_subcriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationActionServerNode>());
    rclcpp::shutdown();
    return 0;
}