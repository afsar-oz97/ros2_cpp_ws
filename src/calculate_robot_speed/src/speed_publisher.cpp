#include "rclcpp/rclcpp.hpp"    // ros client library
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <functional>
using namespace std::chrono_literals;

const double DEFAULT_WHEEL_RADIUS = 12.5 / 100;

class SpeedPublisherNode : public rclcpp::Node
{
public:
    SpeedPublisherNode() : Node("speed_publisher_node")
    {
        rpm_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "robot_rpm", 10, std::bind(&SpeedPublisherNode::calculate_pub_speed, this, std::placeholders::_1));
        this->declare_parameter<double>("wheel_radius", DEFAULT_WHEEL_RADIUS);
        speed_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("robot_speed", 10);    // robot_speed is the topic
    }

private:
    void calculate_pub_speed(const std_msgs::msg::Float64& rpm) const
    {
        rclcpp::Parameter wheel_radius_param = this->get_parameter("wheel_radius");
        auto speed = std_msgs::msg::Float64();
        speed.data = rpm.data * (2 * M_PI * wheel_radius_param.as_double()) / 60;    // Convert RPM to m/s
        speed_publisher_->publish(speed);
        std::cout << "Published Speed: " << speed.data << " m/s" << std::endl;
    }
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rpm_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPublisherNode>());
    rclcpp::shutdown();
    return 0;
}