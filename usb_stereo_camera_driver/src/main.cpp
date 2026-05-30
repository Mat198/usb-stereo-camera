#include "rclcpp/rclcpp.hpp"
#include "usb_stereo_camera_driver/driver.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<stereoCamera::UsbStereoCameraDriver>(options);
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}