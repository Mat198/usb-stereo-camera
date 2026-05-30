#ifndef USB_STEREO_CAMERA_DRIVER_HPP
#define USB_STEREO_CAMERA_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "image_transport/image_transport.hpp"

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <usb_stereo_camera_driver/stereo_camera_parameters.hpp>

namespace stereoCamera {

using ImageMsg = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using SetCameraInfo = sensor_msgs::srv::SetCameraInfo;
using ImagePublisher = image_transport::Publisher;

class UsbStereoCameraDriver : public rclcpp::Node {
public:
    UsbStereoCameraDriver(const rclcpp::NodeOptions &options);

    ~UsbStereoCameraDriver ();

    void mainCameraProcessing();

private:

    bool initializeCamera();

    void splitStereoImages(cv::Mat &frame, cv::Mat &imgL, cv::Mat &imgR);

    ImageMsg createImageMsg(cv::Mat &img, const rclcpp::Time &stamp);

    void loadCameraCalibration();

    void setLeftCameraInfoCallback(
        const SetCameraInfo::Request::SharedPtr request, SetCameraInfo::Response::SharedPtr response
    );

    void setRightCameraInfoCallback(
        const SetCameraInfo::Request::SharedPtr request, SetCameraInfo::Response::SharedPtr response
    );

private:

    rclcpp::Clock::SharedPtr m_clock;
    const rclcpp::Logger m_logger;

    ImagePublisher m_leftImagePub;
    ImagePublisher m_rightImagePub;

    rclcpp::Publisher<CameraInfo>::SharedPtr m_leftCameraInfoPub;
    rclcpp::Publisher<CameraInfo>::SharedPtr m_rightCameraInfoPub;

    // Services to set camera info
    rclcpp::Service<SetCameraInfo>::SharedPtr m_setLeftCameraInfoService;
    rclcpp::Service<SetCameraInfo>::SharedPtr m_setRightCameraInfoService;

    rclcpp::CallbackGroup::SharedPtr m_leftCameraCbGroup;
    rclcpp::CallbackGroup::SharedPtr m_rightCameraCbGroup;

    // Only one capture because we expect one big image with the left and right frames side by side
    cv::VideoCapture m_cap;

    std::thread m_processingThread;

    stereo_camera_driver::ParamListener m_paramListener;
    stereo_camera_driver::Params m_params;

    // Camera calibration data
    CameraInfo m_leftCameraInfo;
    CameraInfo m_rightCameraInfo;
};
}  // namespace stereoCamera
#endif  // USB_STEREO_CAMERA_DRIVER_HPP
