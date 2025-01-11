#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sky_vision2/camera_config/sim_cam_config.hpp"
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

      publisher_id = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic_id, 10);
      publisher_pose = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_pose, 10);
      publisher_debug_img = this->create_publisher<sensor_msgs::msg::Image>(prefix + this->get_parameter("camera_name").as_string() + "/debug_img", 10);

    }

  private:

    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      bool debug = this->get_parameter("debug").as_bool();
      std_msgs::msg::Header header = msg->header;
      header.frame_id = this->get_parameter("camera_name").as_string() + "_frame";

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
      if (!ids.empty()) cv::aruco::drawDetectedMarkers(debug_img, corners, ids);

      // publising a id list
      std_msgs::msg::Int32MultiArray id_msg;
      id_msg.data = ids;
      publisher_id->publish(id_msg);

      // calculating pose
      cv::Mat camera_matrix = CamConfig().camera_matrix;
      cv::Mat dist_coeffs = CamConfig().distortion_coefficients;
      float marker_size = this->get_parameter("marker_size").as_int();
      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
      cv::Mat coordinate_system(4, 1, CV_32FC3);
      coordinate_system.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size/2.f, marker_size/2.f, 0);
      coordinate_system.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_size/2.f, marker_size/2.f, 0); 
      coordinate_system.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_size/2.f, -marker_size/2.f, 0);
      coordinate_system.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size/2.f, -marker_size/2.f, 0);
      for (int i = 0; i < (int)ids.size(); ++i) {
        cv::solvePnP(coordinate_system, corners.at(i), camera_matrix, dist_coeffs, rvecs.at(i), tvecs.at(i));
        cv::drawFrameAxes(debug_img, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_size * 1.5f, 2);
      }

      // publishing pose
      geometry_msgs::msg::PoseArray pose_msg;
      header.stamp = now();
      pose_msg.header = header;
      // creating a pose element for each marker
      for (int i = 0; i < (int)ids.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        // pose translation/position in BODY_NED frame
        pose.position.x = tvecs[i][0];
        pose.position.y = tvecs[i][1];
        pose.position.z = tvecs[i][2];
        
        // pose rotation in BODY_NED frame
        cv::Mat rot_mat;
        cv::Rodrigues(rvecs[i], rot_mat);
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
        pose_msg.poses.push_back(pose); 
      }

      publisher_pose->publish(pose_msg);

      // publishing debug image
      if (debug) {
        // draw detected markers
        header.stamp = now();
        sensor_msgs::msg::Image debug_img_msg;
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, debug_img).toImageMsg(debug_img_msg);

        publisher_debug_img->publish(debug_img_msg);
      }

    }
    // subscribers
    Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // publishers
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_id;
    Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_pose;
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