#ifndef USB_STEREO_CAMERA_DRIVER_HPP
#define USB_STEREO_CAMERA_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace stereoCamera {

using ImageMsg = sensor_msgs::msg::Image;

class UsbStereoCameraDriver : public rclcpp::Node {
public:
    UsbStereoCameraDriver(const rclcpp::NodeOptions &options);

    ~UsbStereoCameraDriver ();

    void mainCameraProcessing();

private:

    bool initializeCamera();

    void splitStereoImages(cv::Mat &frame, cv::Mat &imgL, cv::Mat &imgR);

    ImageMsg createImageMsg(cv::Mat &img, const rclcpp::Time &stamp);

private:

    rclcpp::Clock::SharedPtr m_clock;
    const rclcpp::Logger m_logger;

    rclcpp::Publisher<ImageMsg>::SharedPtr m_leftImagePub;
    rclcpp::Publisher<ImageMsg>::SharedPtr m_rightImagePub;

    // only one capture because we expect one big image with the left and right frames side by side
    cv::VideoCapture m_cap;

    std::thread m_processingThread;

    int m_videoPort;
};
}  // namespace /* namespace_name */
#endif  // USB_STEREO_CAMERA_DRIVER_HPP
