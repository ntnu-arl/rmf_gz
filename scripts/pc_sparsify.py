#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')
        
        # Parameters
        self.declare_parameter('input_topic', '/rmf/lidar/points')
        self.declare_parameter('output_topic', '/rmf/lidar/points_downsampled')
        self.declare_parameter('voxel_size', 0.1)  # 5cm voxel grid
        self.declare_parameter('skip_points', 10)     # Keep every Nth point (alternative method)
        self.declare_parameter('method', 'skip')    # 'voxel' or 'skip'
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.skip_points = self.get_parameter('skip_points').value
        self.method = self.get_parameter('method').value
        
        # Subscriber and Publisher
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10
        )
        
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        
        self.get_logger().info(f'Downsampling point clouds from {input_topic} to {output_topic}')
        self.get_logger().info(f'Method: {self.method}, Voxel size: {self.voxel_size}, Skip: {self.skip_points}')
    
    def pointcloud_callback(self, msg):
        if self.method == 'voxel':
            downsampled_msg = self.voxel_downsample(msg)
        else:
            downsampled_msg = self.skip_downsample(msg)
        
        self.pub.publish(downsampled_msg)
    
    def skip_downsample(self, msg):
        """Simple downsampling by keeping every Nth point"""
        points_list = list(pc2.read_points(msg, skip_nans=True))
        
        # Keep every Nth point
        downsampled_points = points_list[::self.skip_points]
        
        # Create new PointCloud2 message
        downsampled_msg = pc2.create_cloud(msg.header, msg.fields, downsampled_points)
        
        return downsampled_msg
    
    def voxel_downsample(self, msg):
        """Voxel grid downsampling - keeps one point per voxel"""
        # Read points
        points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        
        if len(points_list) == 0:
            return msg
        
        # Convert to numpy array
        points = np.array(points_list)
        
        # Compute voxel indices
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        
        # Get unique voxels (keeps first occurrence)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        
        # Get downsampled points
        downsampled_points = points[unique_indices]
        
        # Convert back to list of tuples
        downsampled_points_list = [tuple(p) for p in downsampled_points]
        
        # Create new PointCloud2 message
        downsampled_msg = pc2.create_cloud_xyz32(msg.header, downsampled_points_list)
        
        original_count = len(points_list)
        downsampled_count = len(downsampled_points_list)
        reduction = (1 - downsampled_count/original_count) * 100
        
        self.get_logger().info(
            f'Downsampled: {original_count} -> {downsampled_count} points ({reduction:.1f}% reduction)',
            throttle_duration_sec=2.0
        )
        
        return downsampled_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDownsampler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()