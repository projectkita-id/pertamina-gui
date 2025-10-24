import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg

class DummyLidarPublisher(Node):
    def __init__(self):
        super().__init__('dummy_lidar_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.timer = self.create_timer(0.1, self.publish_point_cloud)  # 10 Hz
        self.get_logger().info("✅ Dummy LiDAR Publisher started, publishing to /livox/lidar")

    def publish_point_cloud(self):
        # Randomize cube size (0.8m to 1.2m) for dynamic updates
        cube_size = np.random.uniform(0.8, 1.2)
        num_points_per_side = 50  # ~6000 points total
        points = []

        # Generate points on cube surfaces
        for x in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
            for y in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
                # Top and bottom faces
                points.append([x, y, cube_size/2])
                points.append([x, y, -cube_size/2])
        for x in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
            for z in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
                # Front and back faces
                points.append([x, cube_size/2, z])
                points.append([x, -cube_size/2, z])
        for y in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
            for z in np.linspace(-cube_size/2, cube_size/2, num_points_per_side):
                # Left and right faces
                points.append([cube_size/2, y, z])
                points.append([-cube_size/2, y, z])

        points = np.array(points, dtype=np.float32)

        # Randomly shift cube center within ROI bounds
        roi_center = np.array([0.161, -1.230, 1.095], dtype=np.float32)
        roi_half_extent = np.array([1.5, 1.5, 1.5], dtype=np.float32) / 2
        # Allow center to shift within 20% of ROI extent
        center_shift = np.random.uniform(-0.2 * roi_half_extent, 0.2 * roi_half_extent)
        cube_center = roi_center + center_shift
        points += cube_center

        # Add random noise to simulate real LiDAR data
        noise = np.random.normal(0, 0.01, points.shape).astype(np.float32)
        points += noise

        # Generate random intensity values (0 to 100)
        intensities = np.random.uniform(0, 100, len(points)).astype(np.float32)

        # Combine x, y, z, and intensity into a point cloud
        cloud_points = np.column_stack((points, intensities))

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lidar_frame'

        # Define point cloud fields
        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
        ]

        # Create and publish PointCloud2 message
        point_cloud = pc2.create_cloud(header, fields, cloud_points)
        self.publisher.publish(point_cloud)
        self.get_logger().info(f"Published dummy point cloud with {len(cloud_points)} points, cube size: {cube_size:.3f}m")
        print("this is the end")

rclpy.init()
publisher = DummyLidarPublisher()
rclpy.spin(publisher)