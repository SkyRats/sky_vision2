#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/aruco.hpp>



class ArucoDetector : public rclcpp::Node
{
  public:
    ArucoDetector(): Node("aruco_detector") {
      
      this->declare_parameter("camera_topic", "/sky_vision/down_cam/img_raw");
      this->declare_parameter("debug", true);

      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      this->get_parameter("camera_topic").as_string(), 10, std::bind(&ArucoDetector::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      auto debug = this->get_parameter("debug").as_bool();
      if (debug) {
        RCLCPP_INFO(this->get_logger(), "Received image with width: %d, height: %d", msg->width, msg->height);
      }
      
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}