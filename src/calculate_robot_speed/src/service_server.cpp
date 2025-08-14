#include "calculate_robot_speed/srv/odd_even_check.hpp"
#include "rclcpp/rclcpp.hpp"    // ros client library

typedef calculate_robot_speed::srv::OddEvenCheck OddEvenCheck;

class OddEvenCheckServiceNode : public rclcpp::Node
{
public:
    OddEvenCheckServiceNode() : Node("odd_even_check_service_node")
    {
        service_ = this->create_service<OddEvenCheck>("odd_even_check",
            std::bind(&OddEvenCheckServiceNode::check_odd_even, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<OddEvenCheck>::SharedPtr service_;
    void check_odd_even(
        const std::shared_ptr<OddEvenCheck::Request> request, std::shared_ptr<OddEvenCheck::Response> response)
    {
        if (std::abs(request->number) % 2 == 0) { response->decision = "even"; }
        else
        {
            response->decision = "odd";
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OddEvenCheckServiceNode>());
    rclcpp::shutdown();
    return 0;
}