#include <opencv2/opencv.hpp>
#include "calculate_robot_speed/srv/image_angle_change.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"    // ros client library
#include "sensor_msgs/msg/image.hpp"

typedef calculate_robot_speed::srv::ImageAngleChange ImageAngleChange;

int main(int argc, char* argv[])

{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<rclcpp::Node>("image_angle_change_client_node");
    // client and server should use "image_angle_change" as the service name
    auto client = client_node->create_client<ImageAngleChange>("image_angle_change");
    auto request = std::make_shared<ImageAngleChange::Request>();
    std::cout << "Enter an angle to change the image: ";
    std::cin >> request->angle;
    client->wait_for_service();
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->image.data.empty())
        {
            std::cout << "No image data received for angle: " << request->angle << std::endl;
            rclcpp::shutdown();
            return 1;
        }
        auto cv_ptr = cv_bridge::toCvCopy(response->image, sensor_msgs::image_encodings::BGR8);
        auto cv_image = cv_ptr->image;
        cv::imshow("Robot Camera Image", cv_image);
        cv::waitKey(0);
    }
    else
    {
        std::cout << "Failed to call service image_angle_change." << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}