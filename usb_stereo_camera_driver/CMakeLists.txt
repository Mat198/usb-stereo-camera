cmake_minimum_required(VERSION 3.8)
project(usb_stereo_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  include
)

link_directories(${OpenCV_LIBRARY_DIRS})

generate_parameter_library(
  stereo_camera_parameters # cmake target name for the parameter library
  config/stereo_camera_parameters.yaml # path to input yaml file
)

add_executable(stereo_camera_driver src/main.cpp src/driver.cpp)
ament_target_dependencies(stereo_camera_driver rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(stereo_camera_driver ${OpenCV_LIBRARIES} stereo_camera_parameters)

install(TARGETS stereo_camera_driver stereo_camera_parameters
  EXPORT ${PROJECT_NAME}Targets)

add_library(stereo_camera_driver_component SHARED src/composable_main.cpp src/driver.cpp)
ament_target_dependencies(stereo_camera_driver_component rclcpp rclcpp_components sensor_msgs cv_bridge image_transport)
target_link_libraries(stereo_camera_driver_component ${OpenCV_LIBRARIES} stereo_camera_parameters)

rclcpp_components_register_node(
  stereo_camera_driver_component
  PLUGIN "stereoCamera::UsbStereoCameraDriver"
  EXECUTABLE composable_driver
)

install(TARGETS stereo_camera_driver_component
  EXPORT export_stereo_camera_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/cloud_saver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
