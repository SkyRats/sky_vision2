#ifndef sim_cam_config
#define sim_cam_config

#include <opencv2/opencv.hpp>

class CamConfig {
  public:
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 536.60468864, 0.0, 336.71838244, 0.0, 478.13866264, 353.24213721, 0.0, 0.0, 1.0);
    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
};


#endif