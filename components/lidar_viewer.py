import json
import time
import rclpy
import numpy as np
import open3d as o3d
import pages.home as home
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LivoxOpen3DViewer(Node):
    def __init__(self):
        super().__init__('livox_open3d_viewer')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            10
        )

        self.get_logger().info("✅ Livox Open3D Viewer (Manual ROI + Accumulation) started")

        # ==== Open3D setup ====
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window("Livox MID360 - Open3D Viewer", width=960, height=540)
        self.vis.register_key_callback(ord("G"), self.toggle_ground_plane)
        self.pcd = o3d.geometry.PointCloud()
        self.detected = o3d.geometry.PointCloud()
        self.plane_mesh = None
        self.show_ground_plane = True
        self.roi_obb = None

        self.vis.register_key_callback(256, lambda v: self.destroy_node())  # ESC key

        # ==== Keyboard control setup ====
        self.step_move = 0.01  # meter
        self.step_rot  = 2.0   # derajat

        # Geser ROI
        self.vis.register_key_callback(ord('Q'), lambda v: self._nudge_roi(np.array([ self.step_move, 0, 0])))
        self.vis.register_key_callback(ord('E'), lambda v: self._nudge_roi(np.array([-self.step_move, 0, 0])))
        self.vis.register_key_callback(ord('A'), lambda v: self._nudge_roi(np.array([0, -self.step_move, 0])))
        self.vis.register_key_callback(ord('D'), lambda v: self._nudge_roi(np.array([0,  self.step_move, 0])))
        self.vis.register_key_callback(ord('W'), lambda v: self._nudge_roi(np.array([0, 0,  self.step_move])))
        self.vis.register_key_callback(ord('S'), lambda v: self._nudge_roi(np.array([0, 0, -self.step_move])))

        # Rotasi ROI
        self.vis.register_key_callback(ord('U'), lambda v: self._rotate_roi('pitch',  self.step_rot))
        self.vis.register_key_callback(ord('Y'), lambda v: self._rotate_roi('pitch', -self.step_rot))
        self.vis.register_key_callback(ord('O'), lambda v: self._rotate_roi('roll',   self.step_rot))
        self.vis.register_key_callback(ord('P'), lambda v: self._rotate_roi('roll',  -self.step_rot))
        self.vis.register_key_callback(ord('K'), lambda v: self._rotate_roi('yaw',    self.step_rot))
        self.vis.register_key_callback(ord('L'), lambda v: self._rotate_roi('yaw',   -self.step_rot))
        self.vis.register_key_callback(ord('R'), lambda v: self._print_roi())

        self.get_logger().info(
            "🎮 Kontrol ROI: Q/E=Maju/Mundur, A/D=Kiri/Kanan, W/S=Naik/Turun, U/Y=Pitch, O/P=Roll, K/L=Yaw, R=Cetak Posisi"
        )

        self.is_initialized = False
        self.roi_added = False
        self.dim_history = []

        # ==== Frame accumulation ====
        self.accumulate_frames = True
        self.max_history = 5
        self.history_points = []
        self.history_colors = []

        # ==== ROI manual ====
        self.use_manual_roi = True
        self.roi_center_man = np.array([0.161, -1.230, 1.095], dtype=np.float64)
        self.roi_extent_man = np.array([1.5, 1.50, 1.50], dtype=np.float64)
        self.roi_pitch_deg = 4.0
        self.roi_roll_deg  = 24.0
        self.roi_yaw_deg   = 0.0

        # ==== ROI manual box ====
        Rman = self._rotation_matrix_zyx(self.roi_roll_deg, self.roi_pitch_deg, self.roi_yaw_deg)
        self.roi_obb = o3d.geometry.OrientedBoundingBox(self.roi_center_man, Rman, self.roi_extent_man)
        self.roi_obb.color = (1, 0, 0)

    def _rotation_matrix_zyx(self, roll_deg, pitch_deg, yaw_deg):
        rx = np.deg2rad(roll_deg)
        ry = np.deg2rad(pitch_deg)
        rz = np.deg2rad(yaw_deg)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(rx), -np.sin(rx)],
                       [0, np.sin(rx),  np.cos(rx)]])
        Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                       [0, 1, 0],
                       [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                       [np.sin(rz),  np.cos(rz), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx

    def toggle_ground_plane(self, vis):
        self.show_ground_plane = not self.show_ground_plane
        state = "ON" if self.show_ground_plane else "OFF"
        return False

    def _nudge_roi(self, delta):
        self.roi_center_man += delta
        self._update_roi_box()
        x, y, z = self.roi_center_man
        self.get_logger().info(f"📦 ROI Posisi: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    def _rotate_roi(self, axis, delta_deg):
        if axis == 'pitch':
            self.roi_pitch_deg = np.clip(self.roi_pitch_deg + delta_deg, -89.9, 89.9)
        elif axis == 'roll':
            self.roi_roll_deg = (self.roi_roll_deg + delta_deg) % 360
        elif axis == 'yaw':
            self.roi_yaw_deg = (self.roi_yaw_deg + delta_deg) % 360
        self._update_roi_box()
        self.get_logger().info(
            f"↩️ Rotasi ROI: Pitch={self.roi_pitch_deg:.1f}°, Roll={self.roi_roll_deg:.1f}°, Yaw={self.roi_yaw_deg:.1f}°"
        )

    def _update_roi_box(self):
        R = self._rotation_matrix_zyx(self.roi_roll_deg, self.roi_pitch_deg, self.roi_yaw_deg)
        self.roi_obb.center = self.roi_center_man
        self.roi_obb.extent = self.roi_extent_man
        self.roi_obb.R = R
        try:
            self.vis.update_geometry(self.roi_obb)
        except Exception:
            pass

    def _print_roi(self):
        x, y, z = self.roi_center_man
        self.get_logger().info(
            f"🛰️ ROI Sekarang → Posisi: X={x:.3f}, Y={y:.3f}, Z={z:.3f} | "
            f"Pitch={self.roi_pitch_deg:.1f}°, Roll={self.roi_roll_deg:.1f}°, Yaw={self.roi_yaw_deg:.1f}°"
        )
        with open("roi_position.txt", "w") as f:
            json.dump({
                "center": self.roi_center_man.tolist(),
                "size": self.roi_extent_man.tolist(),
                "rotation_deg": {
                    "pitch": self.roi_pitch_deg,
                    "roll": self.roi_roll_deg,
                    "yaw": self.roi_yaw_deg
                }
            }, f, indent=4)

    def _measure_object_dimensions(self):
        pts = np.asarray(self.detected.points)
        if pts.shape[0] < 50:
            return None

        pcd_roi = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))

        plane_model, inliers = pcd_roi.segment_plane(
            distance_threshold=0.015,
            ransac_n=3,
            num_iterations=200
        )
        ground = pcd_roi.select_by_index(inliers)
        non_ground = pcd_roi.select_by_index(inliers, invert=True)

        if self.show_ground_plane:
            if hasattr(self, "plane_mesh") and self.plane_mesh is not None:
                self.vis.remove_geometry(self.plane_mesh, reset_bounding_box=False)

            plane_center = ground.get_center()
            plane_extent = ground.get_axis_aligned_bounding_box().get_extent()
            plane_mesh = o3d.geometry.TriangleMesh.create_box(
                width=max(plane_extent[0], 0.1),
                height=max(plane_extent[1], 0.1),
                depth=0.002
            )
            plane_mesh.paint_uniform_color([1.0, 1.0, 0.0])
            plane_mesh.compute_vertex_normals()

            normal = np.array(plane_model[:3])
            normal /= np.linalg.norm(normal)
            z_axis = np.array([0, 0, 1])
            v = np.cross(z_axis, normal)
            c_ = np.dot(z_axis, normal)
            if np.linalg.norm(v) > 1e-6:
                vx = np.array([[0, -v[2], v[1]],
                               [v[2], 0, -v[0]],
                               [-v[1], v[0], 0]])
                R = np.eye(3) + vx + vx @ vx * ((1 - c_) / (np.linalg.norm(v)**2))
                plane_mesh.rotate(R, center=np.array([0, 0, 0]))
            plane_mesh.translate(plane_center - plane_mesh.get_center())

            self.plane_mesh = plane_mesh
            self.vis.add_geometry(self.plane_mesh, reset_bounding_box=False)
        else:
            if hasattr(self, "plane_mesh") and self.plane_mesh is not None:
                self.vis.remove_geometry(self.plane_mesh, reset_bounding_box=False)
                self.plane_mesh = None

        if non_ground.is_empty():
            self.get_logger().info("⚠️ Tidak ada titik di atas lantai ROI")
            return None

        non_ground, _ = non_ground.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        labels = np.array(non_ground.cluster_dbscan(eps=0.12, min_points=6))
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        if n_clusters == 0:
            pcd_object = non_ground
        else:
            mask = labels >= 0
            obj_points = np.asarray(non_ground.points)[mask]
            pcd_object = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(obj_points))

        points = np.asarray(pcd_object.points)
        if points.shape[0] < 4:
            self.get_logger().info(f"⚠️ Titik terlalu sedikit untuk buat bounding box ({points.shape[0]} titik)")
            if hasattr(self, "obj_box") and self.obj_box is not None:
                self.vis.remove_geometry(self.obj_box, reset_bounding_box=False)
                self.obj_box = None
            return None

        points += np.random.normal(0, 0.0001, points.shape)
        mask = (points[:,2] > np.percentile(points[:,2], 1)) & (points[:,2] < np.percentile(points[:,2], 99))
        points = points[mask]
        if points.shape[0] < 4:
            self.get_logger().info(f"⚠️ Setelah filter, titik masih terlalu sedikit ({points.shape[0]})")
            return None

        pcd_object.points = o3d.utility.Vector3dVector(points)

        try:
            obb = pcd_object.get_oriented_bounding_box()
            obb.color = (0, 1, 0)
        except Exception as e:
            self.get_logger().warn(f"Gagal membuat bounding box: {e}")
            return None

        if hasattr(self, "obj_box") and self.obj_box is not None:
            self.vis.remove_geometry(self.obj_box, reset_bounding_box=False)
        self.obj_box = obb
        self.vis.add_geometry(self.obj_box, reset_bounding_box=False)

        extent = obb.extent
        center = obb.center

        self.dim_history.append(extent)
        if len(self.dim_history) > 5:
            self.dim_history.pop(0)
        avg_extent = np.mean(self.dim_history, axis=0)

        if not hasattr(self, "center_history"):
            self.center_history = []
        self.center_history.append(center)
        if len(self.center_history) > 5:
            self.center_history.pop(0)
        avg_center = np.mean(self.center_history, axis=0)

        alpha = 0.3
        if hasattr(self, "last_center"):
            avg_center = alpha * avg_center + (1 - alpha) * self.last_center
        if hasattr(self, "last_extent"):
            avg_extent = alpha * avg_extent + (1 - alpha) * self.last_extent

        self.last_center = avg_center
        self.last_extent = avg_extent

        obb.center = avg_center
        obb.extent = avg_extent

        if hasattr(self, "obj_box") and self.obj_box is not None:
            self.vis.remove_geometry(self.obj_box, reset_bounding_box=False)
        self.obj_box = obb
        self.vis.add_geometry(self.obj_box, reset_bounding_box=False)

        p, l, t = avg_extent
        self.get_logger().info(
            f"Dimensi: Panjang={p:.3f} m, Lebar={l:.3f} m, Tinggi={t:.3f} m"
        )
        # Send dimensions to CTk via global queue
        home.DIMENSION_QUEUE.put((p, l, t))

        return extent

    def pointcloud_callback(self, msg):
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not cloud_points:
            return

        points = np.array([(p[0], p[1], p[2]) for p in cloud_points], dtype=np.float64)
        intensity = np.array([p[3] for p in cloud_points], dtype=np.float64)

        i_min, i_max = np.min(intensity), np.max(intensity)
        if i_max - i_min < 1e-6:
            i_max = i_min + 1e-6
        i_norm = (intensity - i_min) / (i_max - i_min)
        cmap = plt.get_cmap("turbo")
        colors = cmap(i_norm)[:, :3]

        if self.accumulate_frames:
            self.history_points.append(points)
            self.history_colors.append(colors)
            if len(self.history_points) > self.max_history:
                self.history_points.pop(0)
                self.history_colors.pop(0)
            merged_points = np.vstack(self.history_points)
            merged_colors = np.vstack(self.history_colors)
        else:
            merged_points = points
            merged_colors = colors

        self.pcd.points = o3d.utility.Vector3dVector(merged_points)
        self.pcd.colors = o3d.utility.Vector3dVector(merged_colors)

        if self.use_manual_roi:
            R = self.roi_obb.R
            c = self.roi_obb.center
            half = self.roi_obb.extent / 2.0
            local = (points - c) @ R.T
            mask = np.all((local >= -half) & (local <= half), axis=1)
            roi_points = points[mask]

            if len(roi_points) > 0:
                self.detected.points = o3d.utility.Vector3dVector(roi_points)
                self.detected.paint_uniform_color([1.0, 0.0, 0.0])
                self._measure_object_dimensions()
            else:
                self.detected.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))

        if not self.is_initialized:
            self.vis.add_geometry(self.pcd)
            self.vis.add_geometry(self.roi_obb)
            self.vis.add_geometry(self.detected)

            opt = self.vis.get_render_option()
            opt.background_color = np.asarray([0, 0, 0])
            opt.point_size = 4.0
            opt.light_on = True

            ctr = self.vis.get_view_control()
            bbox = self.pcd.get_axis_aligned_bounding_box()
            ctr.set_lookat(bbox.get_center())
            ctr.set_front([0, 0, -1])
            ctr.set_up([0, 1, 0])
            ctr.set_zoom(0.3)
            self.is_initialized = True
        else:
            self.vis.update_geometry(self.pcd)
            self.vis.update_geometry(self.roi_obb)
            self.vis.update_geometry(self.detected)

        self.vis.poll_events()
        self.vis.update_renderer()

        # if hasattr(self, "_should_close") and self._should_close:
        #     self.get_logger().info("🔴 Closing Open3D Viewer...")
        #     self.vis.destroy_window()
        #     rclpy.shutdown()

    def destroy_node(self):
        # self.vis.destroy_window()
        # super().destroy_node()
        self._should_close = True
        print(self._should_close)