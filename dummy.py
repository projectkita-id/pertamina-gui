#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg

class DummyLidarFloorBag(Node):
    def __init__(self):
        super().__init__('dummy_lidar_floorbag')
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.timer = self.create_timer(0.1, self.publish_point_cloud)  # 10 Hz
        self.get_logger().info("✅ Dummy LiDAR Publisher — floor + sack (63×36×11.5 cm, occluded)")

    # ---------- Geometry generators ----------
    def make_plane(self, size=5.0, res=0.05):
        """Generate dense flat floor (≈ 10k pts)"""
        half = size / 2
        xs = np.linspace(-half, half, int(size / res))
        ys = np.linspace(-half, half, int(size / res))
        X, Y = np.meshgrid(xs, ys)
        Z = np.zeros_like(X)
        pts = np.column_stack((X.ravel(), Y.ravel(), Z.ravel()))
        return pts.astype(np.float32)

    def make_bag(self, center, size=(1.0, 0.70, 0.45), res=0.02):
        """Generate a rectangular 'bag' (not a perfect cube, with slightly curved top)."""
        lx, ly, lz = size
        hx, hy, hz = lx / 2, ly / 2, lz / 2
        pts = []

        # Top & bottom surfaces
        xs = np.linspace(-hx, hx, int(lx / res))
        ys = np.linspace(-hy, hy, int(ly / res))
        for x in xs:
            for y in ys:
                # Top surface: add slight curve
                z_top = hz + 0.01 * np.cos(np.pi * x / lx) * np.cos(np.pi * y / ly)
                pts.append([center[0]+x, center[1]+y, center[2]+z_top])
                pts.append([center[0]+x, center[1]+y, center[2]-hz])

        # Front & back
        zs = np.linspace(-hz, hz, int(lz / res))
        for x in xs:
            for z in zs:
                pts.append([center[0]+x, center[1]+hy, center[2]+z])
                pts.append([center[0]+x, center[1]-hy, center[2]+z])

        # Left & right sides
        for y in ys:
            for z in zs:
                pts.append([center[0]+hx, center[1]+y, center[2]+z])
                pts.append([center[0]-hx, center[1]+y, center[2]+z])

        return np.array(pts, dtype=np.float32)

    # ---------- Publisher ----------
    def publish_point_cloud(self):
        # Dense floor
        floor = self.make_plane(size=5.0, res=0.05)

        # Bag parameters (center, size)
        bag_center = np.array([0.8, -0.8, 0.0575])  # center at half height above ground
        bag_size = (1.0, 0.70, 0.45)

        # Remove floor points under the bag (occlusion)
        hx, hy, hz = bag_size[0]/2, bag_size[1]/2, bag_size[2]/2
        mask = ~(
            (floor[:, 0] > (bag_center[0] - hx)) &
            (floor[:, 0] < (bag_center[0] + hx)) &
            (floor[:, 1] > (bag_center[1] - hy)) &
            (floor[:, 1] < (bag_center[1] + hy))
        )
        floor_visible = floor[mask]

        # Generate the bag
        bag = self.make_bag(center=bag_center, size=bag_size)

        # Combine visible floor + bag
        points = np.vstack([floor_visible, bag])

        # Add small LiDAR noise
        noise = np.random.normal(0, 0.002, points.shape).astype(np.float32)
        points += noise

        # Different intensity for floor vs bag
        floor_intensity = np.full(len(floor_visible), 30.0, dtype=np.float32)  # darker
        bag_intensity = np.full(len(bag), 90.0, dtype=np.float32)  # brighter
        intensities = np.concatenate([floor_intensity, bag_intensity])

        cloud_points = np.column_stack((points, intensities))

        # Create PointCloud2 message
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'lidar_frame'

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='intensity', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        msg = pc2.create_cloud(header, fields, cloud_points)
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published {len(cloud_points)} pts — floor (dark) + bag (bright, occluded below)"
        )

def main():
    rclpy.init()
    node = DummyLidarFloorBag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
