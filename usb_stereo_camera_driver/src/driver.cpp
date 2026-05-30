#include "usb_stereo_camera_driver/driver.hpp"

namespace stereoCamera {

UsbStereoCameraDriver::UsbStereoCameraDriver(const rclcpp::NodeOptions &options) :
    Node("usb_stereo_camera_driver", options),
    m_clock(this->get_clock()),
    m_logger(this->get_logger()),
    m_paramListener(this->get_node_parameters_interface(), this->get_logger()),
    m_params(m_paramListener.get_params())
{

    m_leftImagePub = this->create_publisher<ImageMsg>(
        m_params.left_camera.name + "/image_raw", rclcpp::QoS(1).best_effort()
    );
    m_rightImagePub = this->create_publisher<ImageMsg>(
        m_params.right_camera.name + "/image_raw", rclcpp::QoS(1).best_effort()
    );

    m_leftCameraInfoPub = this->create_publisher<CameraInfo>(
        m_params.left_camera.name + "/camera_info", rclcpp::QoS(1).best_effort()
    );
    m_rightCameraInfoPub = this->create_publisher<CameraInfo>(
        m_params.right_camera.name + "/camera_info", rclcpp::QoS(1).best_effort()
    );

    // Create services to set camera info
    using std::placeholders::_1;
    using std::placeholders::_2;
    m_setLeftCameraInfoService = this->create_service<SetCameraInfo>(
        m_params.left_camera.name + "/set_camera_info", 
        std::bind(&UsbStereoCameraDriver::setLeftCameraInfoCallback, this, _1, _2)
    );

    m_setRightCameraInfoService = this->create_service<SetCameraInfo>(
        m_params.right_camera.name + "/set_camera_info",
        std::bind(&UsbStereoCameraDriver::setRightCameraInfoCallback, this, _1, _2)
    );

    // Load camera calibration from parameters
    loadCameraCalibration();

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
    m_cap = cv::VideoCapture(m_params.camera_port, cv::CAP_V4L2);

    if (!m_cap.isOpened()) {
        RCLCPP_ERROR_STREAM(m_logger, "Could not open camera on port " << m_params.camera_port);
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

    // Side by side image, so width is doubled
    m_cap.set(cv::CAP_PROP_FRAME_WIDTH, m_params.resolution[0] * 2);
    m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_params.resolution[1]);
    m_cap.set(cv::CAP_PROP_FPS, m_params.fps);

    // Read actual properties to verify
    const double actualWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actualHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actualFPS = m_cap.get(cv::CAP_PROP_FPS);
    
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

        if (!m_cap.isOpened()) {
            break;
        }
    
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

        // Update and publish camera info
        m_leftCameraInfo.header.stamp = frameStamp;
        m_rightCameraInfo.header.stamp = frameStamp;

        m_leftImagePub->publish(leftImgMsg);
        m_rightImagePub->publish(rightImgMsg);
        m_leftCameraInfoPub->publish(m_leftCameraInfo);
        m_rightCameraInfoPub->publish(m_rightCameraInfo);
    }
}

void UsbStereoCameraDriver::splitStereoImages(cv::Mat &frame, cv::Mat &imgL, cv::Mat &imgR) {
    
    // Defines the regions for each image
    cv::Rect roiL(0, 0, frame.cols / 2, frame.rows);
    cv::Rect roiR(frame.cols / 2, 0, frame.cols / 2, frame.rows);

    imgL = frame(roiL);
    imgR = frame(roiR);
}

ImageMsg UsbStereoCameraDriver::createImageMsg(cv::Mat &img, const rclcpp::Time &stamp) {
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = stamp;
    cv_image.header.frame_id = "camera_frame";
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = img;
    ImageMsg::SharedPtr msg = cv_image.toImageMsg();
    return *msg;
}

void UsbStereoCameraDriver::loadCameraCalibration() {

    // Load left camera calibration
    m_leftCameraInfo.header.frame_id = m_params.left_camera.name;
    m_leftCameraInfo.width = m_params.resolution[0];
    m_leftCameraInfo.height = m_params.resolution[1];
    m_leftCameraInfo.distortion_model = m_params.left_camera.distortion_model;
    std::copy(
        m_params.left_camera.camera_matrix.begin(), 
        m_params.left_camera.camera_matrix.end(), 
        m_leftCameraInfo.k.begin()
    );
    m_leftCameraInfo.d = m_params.left_camera.distortion_coefficients;
    std::copy(
        m_params.left_camera.rectification_matrix.begin(), 
        m_params.left_camera.rectification_matrix.end(), 
        m_leftCameraInfo.r.begin()
    );
    std::copy(
        m_params.left_camera.projection_matrix.begin(), 
        m_params.left_camera.projection_matrix.end(), 
        m_leftCameraInfo.p.begin()
    );

    // Load right camera calibration
    m_rightCameraInfo.header.frame_id = m_params.right_camera.name;
    m_rightCameraInfo.width = m_params.resolution[0];
    m_rightCameraInfo.height = m_params.resolution[1];
    m_rightCameraInfo.distortion_model = m_params.right_camera.distortion_model;
    std::copy(
        m_params.right_camera.camera_matrix.begin(), 
        m_params.right_camera.camera_matrix.end(), 
        m_rightCameraInfo.k.begin()
    );
    m_rightCameraInfo.d = m_params.right_camera.distortion_coefficients;
    std::copy(
        m_params.right_camera.rectification_matrix.begin(), 
        m_params.right_camera.rectification_matrix.end(), 
        m_rightCameraInfo.r.begin()
    );
    std::copy(
        m_params.right_camera.projection_matrix.begin(), 
        m_params.right_camera.projection_matrix.end(), 
        m_rightCameraInfo.p.begin()
    );

    RCLCPP_INFO_STREAM(m_logger, "Camera calibration loaded successfully");
}

void UsbStereoCameraDriver::setLeftCameraInfoCallback(
    const SetCameraInfo::Request::SharedPtr request, SetCameraInfo::Response::SharedPtr response
) {

    m_leftCameraInfo = request->camera_info;
    response->success = true;
    RCLCPP_INFO_STREAM(m_logger, "Left camera info updated");
}

void UsbStereoCameraDriver::setRightCameraInfoCallback(
    const SetCameraInfo::Request::SharedPtr request, SetCameraInfo::Response::SharedPtr response
) {

    m_rightCameraInfo = request->camera_info;
    response->success = true;
    RCLCPP_INFO_STREAM(m_logger, "Right camera info updated");
}

}  // namespace stereoCamera