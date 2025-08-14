#include "rclcpp/rclcpp.hpp"    // ros client library
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <functional>
using namespace std::chrono_literals;

const double RPM__DEFAULT_VALUE = 100.0;    // Example RPM value, to be replaced with actual logic

class RpmPublisherNode : public rclcpp::Node
{
public:
    RpmPublisherNode() : Node("rpm_publisher_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("robot_rpm", 10);    // robot_rpm is the topic
        // subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        //     "robot_speed", 10, std::bind(&RpmPublisherNode::rpm_callback, this, std::placeholders::_1));

        this->declare_parameter<double>("rpm_value", RPM__DEFAULT_VALUE);

        timer_ = this->create_wall_timer(1s, std::bind(&RpmPublisherNode::publish_rpm, this));

        std::cout << "RPM Publisher Node Initialized" << std::endl;
    }

private:
    void publish_rpm()
    {
        auto rpm = std_msgs::msg::Float64();
        rclcpp::Parameter rpm_value_param = this->get_parameter("rpm_value");
        rpm.data = rpm_value_param.as_double();
        publisher_->publish(rpm);
    }
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    // double rpm_value_parameter_ = RPM__DEFAULT_VALUE;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RpmPublisherNode>());
    rclcpp::shutdown();
    return 0;
}