#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

import numpy as np
import open3d as o3d
import struct
import time
import os

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('point_cloud_saver')
        
        # Parameters (configurable via CLI or launch files)
        self.declare_parameter('cloud_topic', '/points2')
        self.declare_parameter('output_dir', './output_meshes')
        
        topic = self.get_parameter('cloud_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscriber to the PointCloud2 stream
        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.cloud_callback,
            10
        )
        
        # Service to trigger the file save
        self.srv = self.create_service(Trigger, 'save_point_cloud', self.save_cloud_callback)
        
        self.latest_msg = None
        self.get_logger().info(f"Point Cloud Saver initialized. Listening on: {topic}")
        self.get_logger().info("Call service '/save_point_cloud' to save the latest frame.")

    def cloud_callback(self, msg):
        # Continually cache the latest message frame
        self.latest_msg = msg

    def save_cloud_callback(self, request, response):
        if self.latest_msg is None:
            response.success = False
            response.message = "Failed: No point cloud message received yet!"
            return response

        try:
            self.get_logger().info("Extracting point cloud coordinates...")
            pts, colors = self.unpack_ros_point_cloud(self.latest_msg)
            
            if len(pts) == 0:
                response.success = False
                response.message = "Failed: Extracted point cloud is empty."
                return response

            # Create Open3D PointCloud structure
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)
            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)

            # Generate filename based on timestamp
            filename = os.path.join(self.output_dir, f"ros_cloud_{int(time.time())}.ply")
            o3d.io.write_point_cloud(filename, pcd)

            response.success = True
            response.message = f"Successfully saved point cloud to {filename}"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Error saving file: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def unpack_ros_point_cloud(self, msg):
        """Unpacks standard XYZ/RGB profiles from binary PointCloud2 data stream."""
        # Fast byte offset unpacking via numpy frombuffer
        data = np.frombuffer(msg.data, dtype=np.uint8)
        
        # Standard step sizes
        point_step = msg.point_step
        num_points = msg.width * msg.height
        
        # Reshape to easily index per-point fields
        reshaped_data = data.reshape(num_points, point_step)
        
        # Read X, Y, Z coordinates (assuming standard float32 offsets 0, 4, 8)
        xyz = reshaped_data[:, 0:12].view(dtype=np.float32).reshape(num_points, 3)
        
        # Filter out NaN/invalid values typical in structured light/stereo depth sensors
        valid_mask = ~np.isnan(xyz).any(axis=1)
        xyz = xyz[valid_mask]
        
        # Optional: Attempt to pull RGB properties if they exist at offset 16
        colors = None
        if point_step >= 32: 
            try:
                # RGB data in PointCloud2 is packed as a single float32 or uint32
                rgb_bytes = reshaped_data[valid_mask, 16:19]  # extract R, G, B channels directly
                colors = rgb_bytes.astype(np.float32) / 255.0  # Open3D expects range [0, 1]
            except Exception:
                pass

        return xyz, colors

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()