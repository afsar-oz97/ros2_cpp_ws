#include "calculate_robot_speed/srv/odd_even_check.hpp"
#include "rclcpp/rclcpp.hpp"    // ros client library

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<rclcpp::Node>("odd_even_check_client_node");
    // client and server should use "odd_even_check" as the service name
    auto client = client_node->create_client<calculate_robot_speed::srv::OddEvenCheck>("odd_even_check");
    auto request = std::make_shared<calculate_robot_speed::srv::OddEvenCheck::Request>();
    std::cout << "Enter a number to check if it is odd or even: ";
    std::cin >> request->number;
    client->wait_for_service();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        std::cout << "The number " << request->number << " is " << response->decision << "." << std::endl;
    }
    else
    {
        std::cout << "Failed to call service odd_even_check." << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}