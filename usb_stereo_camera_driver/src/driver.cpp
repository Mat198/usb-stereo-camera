#include "usb_stereo_camera_driver/driver.hpp"

namespace stereoCamera {

UsbStereoCameraDriver::UsbStereoCameraDriver(const rclcpp::NodeOptions &options) :
    Node("usb_stereo_camera_driver", options),
    m_clock(this->get_clock()),
    m_logger(this->get_logger())
{

    this->declare_parameter<int>("video_port", 4);
    m_videoPort = this->get_parameter("video_port").as_int();

    m_leftImagePub = this->create_publisher<ImageMsg>("/image/left/color", 10);
    m_rightImagePub = this->create_publisher<ImageMsg>("/image/right/color", 10);

    if (!initializeCamera()) {
        rclcpp::shutdown();
        return;
    }

    // Start thread with camera processing
    m_processingThread = std::thread([this](){mainCameraProcessing();});
}

UsbStereoCameraDriver::~UsbStereoCameraDriver () {

    if (m_cap.isOpened()) {
        m_cap.release();
    }
    m_processingThread.join();
}

bool UsbStereoCameraDriver::initializeCamera() {
    m_cap = cv::VideoCapture(m_videoPort, cv::CAP_V4L2);

    if (!m_cap.isOpened()) {
        RCLCPP_ERROR_STREAM(m_logger, "Could not open camera on port " << m_videoPort);
        return false;
    }
    
    // Defining MJPG to have better FPS
    int MJPG_FOURCC = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    bool setSuccess = m_cap.set(cv::CAP_PROP_FOURCC, MJPG_FOURCC);
    if (setSuccess) {
        RCLCPP_INFO_STREAM(m_logger, "Format set to MJPG.");
    } else {
        RCLCPP_WARN_STREAM(
            m_logger, "Failed to set format property. Camera might be using the default format");
    }

    // TODO: Parametrizar
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    m_cap.set(cv::CAP_PROP_FPS, 30);

    // Read actual properties to verify
    int actualWidth = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int actualHeight = static_cast<int>(m_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    int actualFPS = static_cast<int>(m_cap.get(cv::CAP_PROP_FPS));
    
    RCLCPP_INFO_STREAM(m_logger, "Camera opened successfully!");
    RCLCPP_INFO_STREAM(m_logger, "Camera resolution: " << actualWidth << "x" << actualHeight);
    RCLCPP_INFO_STREAM(m_logger, "Camera FPS: " << actualFPS);
    
    return true;
}

void UsbStereoCameraDriver::mainCameraProcessing() {

    while (rclcpp::ok()) {
        cv::Mat frame;
        const bool frameReadResult = m_cap.read(frame);
        const rclcpp::Time frameStamp = m_clock->now();
    
        if (!frameReadResult) {
            RCLCPP_ERROR_STREAM_THROTTLE(
                m_logger, *m_clock, 1000, "Could not read the camera frame.");
            continue;
        }

        if (frame.empty()) {
            RCLCPP_ERROR_STREAM_THROTTLE(
                m_logger, *m_clock, 1000, "Frame is empty.");
            continue;
        }

        cv::Mat imgR, imgL;
        splitStereoImages(frame, imgL, imgR);

        ImageMsg leftImgMsg = createImageMsg(imgL, frameStamp);
        ImageMsg rightImgMsg = createImageMsg(imgR, frameStamp);

        m_leftImagePub->publish(leftImgMsg);
        m_rightImagePub->publish(rightImgMsg);
    }
}

void UsbStereoCameraDriver::splitStereoImages(cv::Mat &frame, cv::Mat &imgL, cv::Mat &imgR) {
    
    // Defines the regions for each image
    // TODO: Parametrizar com a resolução
    cv::Rect roiL(0,0, 1280, frame.rows);
    cv::Rect roiR(1280,0, frame.cols - 1280, frame.rows);

    imgL = frame(roiL);
    imgR = frame(roiR);
}

ImageMsg UsbStereoCameraDriver::createImageMsg(cv::Mat &img, const rclcpp::Time &stamp) {
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = stamp; // Set timestamp
    cv_image.header.frame_id = "camera_frame"; // Set frame ID
    cv_image.encoding = sensor_msgs::image_encodings::BGR8; // Set encoding (e.g., BGR8 for color images)
    cv_image.image = img; // Assign your cv::Mat
    ImageMsg::SharedPtr msg = cv_image.toImageMsg();
    return *msg;
}

}  // namespace stereoCamera