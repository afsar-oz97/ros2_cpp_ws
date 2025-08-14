#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "calculate_robot_speed/srv/image_angle_change.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"    // ros client library
#include "sensor_msgs/msg/image.hpp"

typedef calculate_robot_speed::srv::ImageAngleChange ImageAngleChange;
auto AVAILABLE_ANGLES = {-15, -30, 0, 15, 30};

class ImageAngleChangeServiceServerNode : public rclcpp::Node
{
public:
    ImageAngleChangeServiceServerNode(std::string exec_dir) : Node("image_angle_change_service_server_node")
    {
        service_ = this->create_service<ImageAngleChange>(
            "image_angle_change", std::bind(&ImageAngleChangeServiceServerNode::image_angle_change, this,
                                      std::placeholders::_1, std::placeholders::_2));
        set_image_directory(exec_dir);
    }

private:
    std::string image_directory_;
    rclcpp::Service<ImageAngleChange>::SharedPtr service_;

    void set_image_directory(std::string exec_dir)
    {
        // {workspace_dir}/install (i.e just before install)
        auto workspace_dir = exec_dir.substr(0, exec_dir.find("/install"));
        image_directory_ = workspace_dir + "/src/calculate_robot_speed/sample_images/";
    }

    void image_angle_change(
        const std::shared_ptr<ImageAngleChange::Request> request, std::shared_ptr<ImageAngleChange::Response> response)
    {
        // closest available angle with respect the requested angle
        int closest_angle = *std::min_element(AVAILABLE_ANGLES.begin(), AVAILABLE_ANGLES.end(),
            [request](int a, int b) { return std::abs(a - request->angle) < std::abs(b - request->angle); });

        RCLCPP_INFO(this->get_logger(), "Closest Available angle: %d", closest_angle);
        auto image_path = image_directory_ + std::to_string(closest_angle) + std::string(".png");
        // Expand tilde to home directory if present
        if (image_path[0] == '~')
        {
            const char* home = getenv("HOME");
            if (home) { image_path = std::string(home) + image_path.substr(1); }
        }
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) { RCLCPP_ERROR(this->get_logger(), "Image not found"); }
        else
        {
            auto image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            response->image = *image_ptr;
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageAngleChangeServiceServerNode>(argv[0]));
    rclcpp::shutdown();
    return 0;
}