#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace rclcpp;


class ArucoDetector : public rclcpp::Node
{
  public:
    ArucoDetector(): Node("aruco_detector") {
      
      this->declare_parameter("camera_name", "down_cam");
      this->declare_parameter("debug", true);
      this->declare_parameter("aruco_type", 5); // 5x5 aruco
      this->declare_parameter("aruco_qtd_id", 250); // 0 - 249 ids
      this->declare_parameter("marker_size", 25); // 25 cm

      topic_camera = prefix + this->get_parameter("camera_name").as_string() + "/img_raw";
      topic_id = prefix + this->get_parameter("camera_name").as_string() + "/aruco_id";
      topic_pose = prefix + this->get_parameter("camera_name").as_string() + "/aruco_pose";


      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_camera, 
        10, 
        std::bind(&ArucoDetector::topic_callback, 
        this, 
        std::placeholders::_1)
      );

      publisher_id = this->create_publisher<std_msgs::msg::Int32>(topic_id, 10);
      publisher_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose, 10);
      publisher_debug_img = this->create_publisher<sensor_msgs::msg::Image>(prefix + this->get_parameter("camera_name").as_string() + "/debug_img", 10);

    }

  private:

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      bool debug = this->get_parameter("debug").as_bool();

      //if (debug) RCLCPP_INFO(this->get_logger(), "Received image");

      cv::Mat frame;

      try {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      }
      catch (cv_bridge::Exception& e) {
        if (debug) RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      
      if (frame.empty()) {
        if (debug) RCLCPP_ERROR(this->get_logger(), "Empty frame");
        return;
      }
      
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;

      cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
      cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250); // TODO: change to parameter 
      cv::aruco::ArucoDetector detector(dictionary, detectorParams);
      detector.detectMarkers(frame, corners, ids);
      if (debug) RCLCPP_INFO(this->get_logger(), "Detected %d markers", (int)ids.size());
      
      auto debug_img = frame.clone();
      cv::aruco::drawDetectedMarkers(debug_img, corners, ids);

      std_msgs::msg::Int32 id_msg;
      id_msg.data = ids.size() > 0 ? ids[0] : -1;
      //publisher_debug_img->publish(*cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg());
      publisher_id->publish(id_msg);
    }
    // subscribers
    Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // publishers
    Publisher<std_msgs::msg::Int32>::SharedPtr publisher_id;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;
    Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_debug_img;

    // parameters
    const std::string prefix = "/sky_vision/";
    std::string topic_camera;
    std::string topic_id;
    std::string topic_pose;
};

int main(int argc, char * argv[])
{
  init(argc, argv);
  spin(std::make_shared<ArucoDetector>());
  shutdown();
  return 0;
}