# USB Stere Camera ROS 2 packages


#### Testing

Run camera:

```
ros2 launch usb_stereo_camera_driver driver.launch.py camera_port:=0
```

View Image:

```
ros2 run image_view stereo_view --ros-args -r /stereo/disparity:=/disparity -r /stereo/left/image:=/left_camera/image_rect -r /stereo/right/image:=/right_camera/image_rect
```
