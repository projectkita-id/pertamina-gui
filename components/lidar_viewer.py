#!/usr/bin/env python3
from rclpy.node import Node
from datetime import datetime
from multiprocessing import Queue
from sensor_msgs.msg import PointCloud2
from components.db_config import get_conf

import time
import queue
import pymysql
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import rclpy, threading, numpy as np, json, os, matplotlib.pyplot as plt

GUI_INSTANCE = None
LAST_SEND = 0

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "roi_config.json")
GROUND_MODEL_PATH = os.path.join(os.path.dirname(__file__), "ground_plane.npy")

def upload(p, l, t):
    global LAST_SEND
    current_time = time.time()
    if current_time - LAST_SEND < 5:
        print("[INFO] Melewati upload data untuk menghindari pengiriman berlebih.")
        return

    config = get_conf(["address", "username", "password", "port", "db_name"])
    if not all(config):
        print("[ERROR] Database configuration is incomplete.")
        return
    
    address, username, password, port, db_name = config
    port = int(port) if port else 3306

    conn = None
    try:
        conn = pymysql.connect(
            host=address,
            user=username,
            password=password,
            port=port,
            database=db_name,
            charset='utf8mb4',
            cursorclass=pymysql.cursors.DictCursor
        )
        cursor = conn.cursor()

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS data (
                id INT AUTO_INCREMENT PRIMARY KEY,
                panjang FLOAT NOT NULL,
                lebar FLOAT NOT NULL,
                tinggi FLOAT NOT NULL,
                timestamp DATETIME NOT NULL
            )
        """)
        conn.commit()

        timestamp = datetime.now()
        cursor.execute("""
            INSERT INTO data (panjang, lebar, tinggi, timestamp)
            VALUES (%s, %s, %s, %s)
        """, (round(p, 2), round(l, 2), round(t, 2), timestamp))
        conn.commit()
        print(f"[INFO] Data dikirim ke DB: P={round(p, 2)}, L={round(l, 2)}, T={round(t, 2)}, Timestamp={timestamp}")
        LAST_SEND = current_time
    except pymysql.MySQLError as e:
        print(f"[ERROR] Gagal mengirim data ke database: {str(e)}")
    finally:
        if conn:
            conn.close()

class LivoxGUI(Node):
    def __init__(self, app, dim_queue=None):
        global GUI_INSTANCE
        super().__init__("livox_open3d_gui")
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, 10)

        self.dim_queue = dim_queue if dim_queue is not None else Queue

        self.app = app
        self.window = app.create_window("Livox MID360 - Open3D GUI", 1280, 600)
        
        flag_path = "/tmp/lidar_visible"
        if os.path.exists(flag_path):
            os.remove(flag_path)
        self.window.show(False)

        def on_close():
            self.hide_window()
            flag_path = "/tmp/lidar_visible"
            if os.path.exists(flag_path):
                os.remove(flag_path)
            return False
        
        GUI_INSTANCE = self
        self.window.set_on_close(on_close)

        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)
        self.dim_label = gui.Label("Belum ada objek terdeteksi")

        self.scene.scene.set_background([0, 0, 0, 1])
        self.scene.scene.show_axes(True)

        self.pcd = o3d.geometry.PointCloud()
        self._camera_set = False

        # === Geometry names to manage on scene ===
        self._name_cloud = "PointCloud"
        self._name_roi = "ROI"
        self._name_obb = "StableOBB"

        # Flags & params
        self.show_ground_color = True         # hanya mematikan warna, bukan deteksi
        self.ground_model = None
        self.is_calibrating = False
        self.ground_thresh = 0.03             # <— lebih ketat dari 0.08, bisa disesuaikan
        self.calib_thresh = 0.02              # RANSAC distance saat kalibrasi

        # Smoothing state
        self._dim_hist_height = []
        self._last_height = None
        self._dim_hist_xy = []
        self._last_xy = None

        # ==== ROI default ====
        self.roi_center = np.array([0.0, 0.0, 0.0])
        self.roi_size   = np.array([2.0, 2.0, 1.0])
        self.roi_rpy    = np.array([0.0, 0.0, 0.0])

        # ==== TOAST ====
        self.toast_label = gui.Label("")
        self._toast_counter = 0

        # ==== Orientasi LiDAR ====
        self.set_lidar_orientation(tilt_deg=30.0, invert_z=True)

        # ==== Load ROI config ====
        self.load_roi_from_file()

        # ==== ROI geometry ====
        self.roi_box = self.create_roi_box()
        self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())

        self.window.set_on_layout(self.on_layout)

        # ==== Load ground model kalau ada ====
        if os.path.exists(GROUND_MODEL_PATH):
            self.load_ground_model()

        self.monitoring = threading.Thread(target=self.monitor_window_flag, daemon=True)
        self.monitoring.start()

        self.get_logger().info("✅ GUI aktif + ROI + Ground Calibration + Stable Dimension Measure")

    def monitor_window_flag(self):
        flag_path = "/tmp/lidar_visible"

        while True:
            should_be_visible = os.path.exists(flag_path)
            current_visible = self.window.is_visible

            if should_be_visible and not current_visible:
                def do_show():
                    self.show_window()
                    self.get_logger().info("Window SHOW command sent")
                    gui.Application.instance.post_to_main_thread(
                        self.window,
                        lambda: self.get_logger().info(f"Window NOW visible: {self.window.is_visible}")
                    )

                gui.Application.instance.post_to_main_thread(self.window, do_show)

            elif not should_be_visible and current_visible:
                def do_hide():
                    self.hide_window()
                    self.get_logger().info("Window HIDE command sent")

                gui.Application.instance.post_to_main_thread(self.window, do_hide)

            threading.Event().wait(0.3)

    # ---------------------- UI util ----------------------
    def show_toast(self, text, duration=0.8):
        self._toast_counter += 1
        my_id = self._toast_counter

        def _show():
            self.toast_label.text = text

        def _clear_if_same():
            if my_id == self._toast_counter:
                self.toast_label.text = ""

        gui.Application.instance.post_to_main_thread(self.window, _show)
        t = threading.Timer(duration, lambda: gui.Application.instance.post_to_main_thread(self.window, _clear_if_same))
        t.daemon = True
        t.start()

    def make_plus_minus_row(self, label_text, target_attr, idx, step):
        h = gui.Horiz()
        label = gui.Label(label_text)
        value = gui.Label(f"{getattr(self, target_attr)[idx]:.3f}")
        minus_btn = gui.Button("_")
        plus_btn = gui.Button("+")
        minus_btn.horizontal_padding_em = 0.4
        plus_btn.horizontal_padding_em = 0.4

        def adjust(val):
            arr = getattr(self, target_attr).copy()
            arr[idx] += val
            setattr(self, target_attr, arr)
            value.text = f"{arr[idx]:.3f}"
            self.update_roi_box()
            self.get_logger().info(f"{label_text}: {arr[idx]:.3f}")
            self.show_toast(f"{label_text} diubah: {arr[idx]:.3f}")

        minus_btn.set_on_clicked(lambda: adjust(-step))
        plus_btn.set_on_clicked(lambda: adjust(step))
        h.add_child(label)

        h.add_stretch()
        h.add_child(value)
        
        h.add_stretch()
        h.add_child(minus_btn)
        h.add_child(plus_btn)
        return h

    # ----------------- Lidar orientation -----------------
    def set_lidar_orientation(self, tilt_deg=30.0, invert_z=True):
        self.lidar_tilt_deg = tilt_deg
        self.lidar_invert_z = invert_z

    def get_lidar_rotation_matrix(self):
        tilt_rad = np.deg2rad(self.lidar_tilt_deg)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
            [0, np.sin(tilt_rad),  np.cos(tilt_rad)],
        ])
        Rflip = np.eye(3)
        if self.lidar_invert_z:
            Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        return Rx @ Rflip

    def toggle_ground_color(self):
        self.show_ground_color = not self.show_ground_color
        state = "ON" if self.show_ground_color else "OFF"
        self.get_logger().info(f"🌍 Ground color display {state}")
        self.show_toast(f"Ground color {state}")

    # ----------------- ROI helpers -----------------
    def roi_material(self):
        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.base_color = [1.0, 0.0, 0.0, 1.0]
        return mat

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        r, p, y = np.deg2rad([roll, pitch, yaw])
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[ np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[ np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def create_roi_box(self):
        R = self.euler_to_rotation_matrix(*self.roi_rpy)
        bbox = o3d.geometry.OrientedBoundingBox(self.roi_center, R, self.roi_size)
        bbox.color = (1, 0, 0)
        return bbox

    def update_roi_box(self):
        try:
            self.scene.scene.remove_geometry(self._name_roi)
        except Exception:
            pass
        self.roi_box = self.create_roi_box()
        self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())
        self.scene.force_redraw()

    # ----------------- Ground Calibration -----------------
    def calibrate_ground(self):
        self.is_calibrating = True
        self.get_logger().info("📏 Calibrating ground plane... (tunggu beberapa frame)")
        self.show_toast("Calibrating ground...")

    def save_ground_model(self, model):
        np.save(GROUND_MODEL_PATH, model)
        self.get_logger().info(f"💾 Ground model saved to {GROUND_MODEL_PATH}")
        self.show_toast("Ground model disimpan")

    def load_ground_model(self):
        if os.path.exists(GROUND_MODEL_PATH):
            self.ground_model = np.load(GROUND_MODEL_PATH)
            self.get_logger().info(f"📂 Ground model loaded from {GROUND_MODEL_PATH}")
            self.show_toast("Ground model dimuat")
        else:
            self.get_logger().warn("❌ Belum ada model ground, lakukan kalibrasi dahulu!")
            self.show_toast("Belum ada ground model")

    # ----------------- ROI persistence -----------------
    def save_roi_to_file(self):
        data = {"center": self.roi_center.tolist(), "size": self.roi_size.tolist(), "rpy": self.roi_rpy.tolist()}
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"💾 ROI saved to {CONFIG_PATH}")
        self.show_toast("ROI disimpan")

    def load_roi_from_file(self):
        # os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
            self.roi_center = np.array(data.get("center", [0, 0, 0]), dtype=float)
            self.roi_size   = np.array(data.get("size",   [2, 2, 1]), dtype=float)
            self.roi_rpy    = np.array(data.get("rpy",    [0, 0, 0]), dtype=float)
            print(f"[INFO] ROI loaded from {CONFIG_PATH}")
        else:
            print("[INFO] No saved ROI config found, using defaults")

    def reset_roi(self):
        self.roi_center = np.array([0.0, 0.0, 0.0])
        self.roi_size   = np.array([2.0, 2.0, 1.0])
        self.roi_rpy    = np.array([0.0, 0.0, 0.0])
        self.update_roi_box()
        self.get_logger().info("🔄 ROI reset ke default")
        self.show_toast("ROI di-reset")

    # ----------------- Stable dimension measure -----------------
    def measure_dims_stable(self, pts_in, assume_non_ground=False):
        """
        Dimensi stabil:
        - (Opsional) segment ground ketat
        - Buang noise
        - Cluster DBSCAN (eps lebih ketat), gabungkan label valid
        - P & L: PCA 2D pada XY (bukan axis-aligned)
        - T: range persentil di sepanjang normal ground (jika ada), else Z
        - Smoothing: median guard + EMA
        Return: (P, L, T, obb or None) atau None.
        """
        if pts_in.shape[0] < 50:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_in)

        # 1) Segment ground bila diminta
        work = pcd
        if not assume_non_ground:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=300)
            if len(inliers) >= 100:
                work = pcd.select_by_index(inliers, invert=True)

        if work.is_empty():
            return None

        # 2) Noise removal
        work, _ = work.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if work.is_empty():
            return None

        # 3) Cluster & kumpulkan semua label valid (eps lebih ketat)
        labels = np.array(work.cluster_dbscan(eps=0.1, min_points=6, print_progress=False))
        if (labels >= 0).any():
            obj_points = np.asarray(work.points)[labels >= 0]
        else:
            obj_points = np.asarray(work.points)

        if obj_points.shape[0] < 20:
            return None

        # Anti-QHull degeneracy
        obj_points = obj_points + np.random.normal(0, 1e-4, obj_points.shape)

        # --- T (tinggi) di sepanjang normal ground (kalau ada) ---
        if self.ground_model is not None:
            a, b, c, d = self.ground_model
            n = np.array([a, b, c], dtype=float)
            n_norm = np.linalg.norm(n) + 1e-12
            # Signed distance (positif di atas bidang)
            h = (obj_points @ n + d) / n_norm
            # Buang ekor 2% teratas & terbawah
            h2, h98 = np.percentile(h, [0.1, 99.99])
            true_height = float(h98 - h2)
        else:
            # fallback: pakai Z dunia tapi ketat
            z2, z98 = np.percentile(obj_points[:, 2], [0.1, 99.99])
            true_height = float(z98 - z2)

        # --- P & L dari PCA 2D (XY) ---
            # --- P & L dari PCA 2D (XY) ---
        XY = obj_points[:, :2]
        mu = XY.mean(axis=0, keepdims=True)
        Xc = XY - mu

        # Tambahkan sedikit regularisasi supaya orientasi PCA tidak meloncat antar frame
        cov = np.cov(Xc.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        # Urutkan eigenvalue menurun
        order = np.argsort(eigvals)[::-1]
        eigvecs = eigvecs[:, order]

        # Simpan orientasi PCA agar bisa dismoothing antar frame
        if not hasattr(self, "_last_pca_vecs"):
            self._last_pca_vecs = eigvecs.copy()
        else:
            # Exponential smoothing pada arah sumbu utama
            self._last_pca_vecs = 0.2 * eigvecs + 0.8 * self._last_pca_vecs
            # Orthonormalize ulang
            u, _, vt = np.linalg.svd(self._last_pca_vecs, full_matrices=False)
            self._last_pca_vecs = u @ vt
        V = self._last_pca_vecs  # gunakan orientasi yang dismooth

        # Proyeksikan ke sumbu PCA yang sudah stabil
        proj = Xc @ V
        # Persentil lebih ketat untuk tahan outlier
        p_low, p_high = np.percentile(proj[:, 0], [3.0, 97.0])
        q_low, q_high = np.percentile(proj[:, 1], [3.0, 97.0])
        true_length = float(p_high - p_low)
        true_width  = float(q_high - q_low)

        # Filter ukuran ekstrem antar frame
        if hasattr(self, "_last_dims_raw"):
            last_L, last_W = self._last_dims_raw
            # Batasi lonjakan per frame maksimal 10%
            true_length = np.clip(true_length, 0.9 * last_L, 1.1 * last_L)
            true_width  = np.clip(true_width, 0.9 * last_W, 1.1 * last_W)
        self._last_dims_raw = (true_length, true_width)


        # --- Smoothing & guard seperti sebelumnya ---
        # T
        self._dim_hist_height.append(true_height)
        if len(self._dim_hist_height) > 12:
            self._dim_hist_height.pop(0)
        median_h = float(np.median(self._dim_hist_height))
        if true_height > 1.25 * median_h:   # guard outlier ke atas
            true_height = median_h
        if self._last_height is not None:
            true_height = 0.28 * true_height + 0.75 * self._last_height
        self._last_height = true_height

        # P & L
        self._dim_hist_xy.append([true_length, true_width])
        if len(self._dim_hist_xy) > 12:
            self._dim_hist_xy.pop(0)
        xy_med = np.median(self._dim_hist_xy, axis=0)
        p_corr, l_corr = true_length, true_width
        # guard 15% deviasi
        if abs(p_corr - xy_med[0]) > 0.15 * max(1e-6, xy_med[0]):
            p_corr = float(xy_med[0])
        if abs(l_corr - xy_med[1]) > 0.15 * max(1e-6, xy_med[1]):
            l_corr = float(xy_med[1])
        if self._last_xy is not None:
            p_corr = 0.25 * p_corr + 0.75 * float(self._last_xy[0])
            l_corr = 0.20 * l_corr + 0.75 * float(self._last_xy[1])
        self._last_xy = np.array([p_corr, l_corr], dtype=np.float32)

        # --- OBB hanya visual (tak dipakai ukur) ---
        obb = None
        try:
            pcd_obj = o3d.geometry.PointCloud()
            pcd_obj.points = o3d.utility.Vector3dVector(obj_points)
            obb = pcd_obj.get_oriented_bounding_box()
            obb.color = (1.0, 1.0, 0.0)
        except Exception:
            obb = None

        return p_corr, l_corr, true_height, obb

    # ----------------- Callback ROS -----------------
    def cb(self, msg):
        cloud = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not cloud:
            return

        pts = np.asarray([(p[0], p[1], p[2]) for p in cloud], dtype=np.float32)
        inten = np.asarray([p[3] for p in cloud], dtype=np.float32)

        # Transform LiDAR -> world
        R_lidar = self.get_lidar_rotation_matrix()
        pts = (R_lidar @ pts.T).T

        # Colorize by intensity
        cmap = plt.get_cmap("turbo")
        inten_n = (inten - inten.min()) / (inten.ptp() + 1e-6)
        cols = cmap(inten_n)[:, :3].astype(np.float32)

        # ROI mask
        bbox = self.create_roi_box()
        roi_indices = bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(pts))
        roi_indices = np.array(roi_indices, dtype=int)
        dim_text = "Belum ada objek terdeteksi"

        # Prepare for OBB redraw
        def remove_old_obb():
            try:
                self.scene.scene.remove_geometry(self._name_obb)
            except Exception:
                pass

        remove_old_obb()

        if len(roi_indices) > 50:
            pts_roi = pts[roi_indices]

            # Kalibrasi ground saat diminta
            if self.is_calibrating:
                try:
                    pcd_roi = o3d.geometry.PointCloud()
                    pcd_roi.points = o3d.utility.Vector3dVector(pts_roi)
                    plane_model, inliers = pcd_roi.segment_plane(
                        distance_threshold=self.calib_thresh, ransac_n=3, num_iterations=300
                    )
                    self.save_ground_model(plane_model)
                    self.is_calibrating = False
                    self.ground_model = plane_model
                    a, b, c, d = plane_model
                    self.get_logger().info(
                        f"✅ Ground calibrated: a={a:.3f}, b={b:.3f}, c={c:.3f}, d={d:.3f}"
                    )
                    self.show_toast("Kalibrasi ground selesai")
                except Exception as e:
                    self.is_calibrating = False
                    self.get_logger().warn(f"Kalibrasi gagal: {e}")
                    self.show_toast("Kalibrasi gagal")

            # Color ground (opsional) + non-ground extraction untuk measure
            assume_non_ground = False
            pts_for_measure = pts_roi
            if self.ground_model is not None:
                a, b, c, d = self.ground_model
                denom = np.sqrt(a ** 2 + b ** 2 + c ** 2) + 1e-9
                heights = np.abs((a * pts_roi[:, 0] + b * pts_roi[:, 1] + c * pts_roi[:, 2] + d) / denom)
                ground_mask = heights < self.ground_thresh
                non_ground_mask = ~ground_mask
                if self.show_ground_color:
                    # hanya warnai ground jadi hijau, tidak mematikan deteksi
                    cols[roi_indices[ground_mask]] = [0.0, 1.0, 0.0]
                # Gunakan hanya non-ground untuk pengukuran
                if non_ground_mask.sum() > 20:
                    pts_for_measure = pts_roi[non_ground_mask]
                    assume_non_ground = True

            # Ukur dimensi stabil
            result = self.measure_dims_stable(pts_for_measure, assume_non_ground=assume_non_ground)
            if result is not None:
                P, L, T, obb = result
                dim_text = f"Objek: P={P:.3f} m, L={L:.3f} m, T={T:.3f} m"
                print(dim_text)

                try:
                    print("sending to gui")
                    self.dim_queue.put_nowait((P, L, T))
                    upload(P, L, T)
                except queue.Full:
                    print("nope, queue full - skip data")  # Kalau full
                except Exception as e:
                    print(f"nope, sending error: {type(e).__name__} - {str(e)}")  # Detail error
                    pass

                if obb is not None:
                    mat_box = rendering.MaterialRecord()
                    mat_box.shader = "unlitLine"
                    mat_box.line_width = 4.0
                    self.scene.scene.add_geometry(self._name_obb, obb, mat_box)

        # Update label di UI
        gui.Application.instance.post_to_main_thread(
            self.window, lambda: setattr(self.dim_label, "text", dim_text)
        )

        # Update point cloud
        self.pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd.colors = o3d.utility.Vector3dVector(cols)

        def update_scene():
            # Clear hanya geometri dinamis, lalu re-add
            self.scene.scene.clear_geometry()
            self.scene.scene.add_geometry(self._name_cloud, self.pcd, rendering.MaterialRecord())
            self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())
            # OBB jika ada sudah ditambahkan di atas (akan hilang saat clear, jadi re-add bila perlu)
            # Untuk kesederhanaan, bila obb dibuat pada frame ini sudah ditambahkan; kalau ingin persist,
            # bisa simpan dan re-add di sini.

            if not self._camera_set:
                bbox = self.pcd.get_axis_aligned_bounding_box()
                center = bbox.get_center()
                radius = np.linalg.norm(bbox.get_extent()) * 0.5 + 1.0
                eye = center + np.array([radius, radius, radius])
                self.scene.scene.camera.look_at(center.tolist(), eye.tolist(), [0, 0, 1])
                self._camera_set = True
            self.scene.force_redraw()

        gui.Application.instance.post_to_main_thread(self.window, update_scene)

    # ----------------- Layout -----------------
    def on_layout(self, ctx):
        r = self.window.content_rect
        self.scene.frame = gui.Rect(r.x, r.y, r.width, r.height)

    def show_window(self):
        self.window.show(True)
        self.get_logger().info("A window has appear")
        self.scene.force_redraw()

    def hide_window(self):
        self.window.show(False)
        self.get_logger().info("The window is hidden")
        self.scene.force_redraw()

    def toggle_window(self):
        if self.window.is_visible:
            self.hide_window()
        else:
            self.show_window()

class LivoxCalib(Node):
    def __init__(self, app):
        global GUI_INSTANCE
        super().__init__("livox_open3d_gui")
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, 10)

        self.app = app
        self.window = app.create_window("Livox MID360 - Open3D GUI", 1600, 900)
        
        # flag_path = "/tmp/lidar_calib_visible"
        # if os.path.exists(flag_path):
        #     os.remove(flag_path)
        # self.window.show(False)

        # def on_close():
        #     self.hide_window()
        #     flag_path = "/tmp/lidar_calib_visible"
        #     if os.path.exists(flag_path):
        #         os.remove(flag_path)
        #     return False
        
        GUI_INSTANCE = self
        # self.window.set_on_close(on_close)

        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self.scene)
        self.dim_label = gui.Label("Belum ada objek terdeteksi")

        self.scene.scene.set_background([0, 0, 0, 1])
        self.scene.scene.show_axes(True)

        self.pcd = o3d.geometry.PointCloud()
        self._camera_set = False

        # === Geometry names to manage on scene ===
        self._name_cloud = "PointCloud"
        self._name_roi = "ROI"
        self._name_obb = "StableOBB"

        # Flags & params
        self.show_ground_color = True         # hanya mematikan warna, bukan deteksi
        self.ground_model = None
        self.is_calibrating = False
        self.ground_thresh = 0.03             # <— lebih ketat dari 0.08, bisa disesuaikan
        self.calib_thresh = 0.02              # RANSAC distance saat kalibrasi

        # Smoothing state
        self._dim_hist_height = []
        self._last_height = None
        self._dim_hist_xy = []
        self._last_xy = None

        # ==== ROI default ====
        self.roi_center = np.array([0.0, 0.0, 0.0])
        self.roi_size   = np.array([2.0, 2.0, 1.0])
        self.roi_rpy    = np.array([0.0, 0.0, 0.0])

        # ==== TOAST ====
        self.toast_label = gui.Label("")
        self._toast_counter = 0

        # ==== Orientasi LiDAR ====
        self.set_lidar_orientation(tilt_deg=30.0, invert_z=True)

        # ==== Load ROI config ====
        self.load_roi_from_file()

        # ==== ROI geometry ====
        self.roi_box = self.create_roi_box()
        self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())

        # ==== Panel GUI ====
        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(margin, gui.Margins(margin, margin, margin, margin))

        self.panel.add_child(gui.Label("ROI Box Controls"))
        self.panel.add_child(gui.Label("Position (m)"))
        self.panel.add_child(self.make_plus_minus_row("X", "roi_center", 0, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Y", "roi_center", 1, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Z", "roi_center", 2, 0.01))
        self.panel.add_child(gui.Label("Size (m)"))
        self.panel.add_child(self.make_plus_minus_row("Size X", "roi_size", 0, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Size Y", "roi_size", 1, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Size Z", "roi_size", 2, 0.01))
        self.panel.add_child(gui.Label("Rotation (deg)"))
        self.panel.add_child(self.make_plus_minus_row("Roll",  "roi_rpy", 0, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Pitch", "roi_rpy", 1, 0.01))
        self.panel.add_child(self.make_plus_minus_row("Yaw",   "roi_rpy", 2, 0.01))

        # Buttons utama
        save_btn = gui.Button("Save ROI")
        save_btn.set_on_clicked(self.save_roi_to_file)
        reset_btn = gui.Button("Reset ROI")
        reset_btn.set_on_clicked(self.reset_roi)
        toggle_color_btn = gui.Button("Toggle Ground Color")
        toggle_color_btn.set_on_clicked(self.toggle_ground_color)
        calibrate_btn = gui.Button("Calibrate Ground")
        calibrate_btn.set_on_clicked(self.calibrate_ground)
        load_model_btn = gui.Button("Load Ground Model")
        load_model_btn.set_on_clicked(self.load_ground_model)
        self.panel.add_child(save_btn)
        self.panel.add_child(reset_btn)
        self.panel.add_child(toggle_color_btn)
        self.panel.add_child(calibrate_btn)
        self.panel.add_child(load_model_btn)

        # Label hasil pengukuran
        self.panel.add_child(gui.Label(" "))
        self.panel.add_child(gui.Label("Object Dimensions (m):"))
        self.panel.add_child(self.dim_label)

        # Toast area
        self.panel.add_child(gui.Label(" "))
        self.panel.add_child(gui.Label("Notifikasi cepat:"))
        self.panel.add_child(self.toast_label)

        self.window.add_child(self.panel)
        self.window.set_on_layout(self.on_layout)

        # ==== Load ground model kalau ada ====
        if os.path.exists(GROUND_MODEL_PATH):
            self.load_ground_model()

        self.get_logger().info("✅ GUI aktif + ROI + Ground Calibration + Stable Dimension Measure")

    # ---------------------- UI util ----------------------
    def show_toast(self, text, duration=0.8):
        self._toast_counter += 1
        my_id = self._toast_counter

        def _show():
            self.toast_label.text = text

        def _clear_if_same():
            if my_id == self._toast_counter:
                self.toast_label.text = ""

        gui.Application.instance.post_to_main_thread(self.window, _show)
        t = threading.Timer(duration, lambda: gui.Application.instance.post_to_main_thread(self.window, _clear_if_same))
        t.daemon = True
        t.start()

    def make_plus_minus_row(self, label_text, target_attr, idx, step):
        h = gui.Horiz()
        label = gui.Label(label_text)
        value = gui.Label(f"{getattr(self, target_attr)[idx]:.3f}")
        minus_btn = gui.Button("_")
        plus_btn = gui.Button("+")
        minus_btn.horizontal_padding_em = 0.4
        plus_btn.horizontal_padding_em = 0.4

        def adjust(val):
            arr = getattr(self, target_attr).copy()
            arr[idx] += val
            setattr(self, target_attr, arr)
            value.text = f"{arr[idx]:.3f}"
            self.update_roi_box()
            self.get_logger().info(f"{label_text}: {arr[idx]:.3f}")
            self.show_toast(f"{label_text} diubah: {arr[idx]:.3f}")

        minus_btn.set_on_clicked(lambda: adjust(-step))
        plus_btn.set_on_clicked(lambda: adjust(step))
        h.add_child(label)

        h.add_stretch()
        h.add_child(value)
        
        h.add_stretch()
        h.add_child(minus_btn)
        h.add_child(plus_btn)
        return h

    # ----------------- Lidar orientation -----------------
    def set_lidar_orientation(self, tilt_deg=30.0, invert_z=True):
        self.lidar_tilt_deg = tilt_deg
        self.lidar_invert_z = invert_z

    def get_lidar_rotation_matrix(self):
        tilt_rad = np.deg2rad(self.lidar_tilt_deg)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
            [0, np.sin(tilt_rad),  np.cos(tilt_rad)],
        ])
        Rflip = np.eye(3)
        if self.lidar_invert_z:
            Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        return Rx @ Rflip

    def toggle_ground_color(self):
        self.show_ground_color = not self.show_ground_color
        state = "ON" if self.show_ground_color else "OFF"
        self.get_logger().info(f"🌍 Ground color display {state}")
        self.show_toast(f"Ground color {state}")

    # ----------------- ROI helpers -----------------
    def roi_material(self):
        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.base_color = [1.0, 0.0, 0.0, 1.0]
        return mat

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        r, p, y = np.deg2rad([roll, pitch, yaw])
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[ np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[ np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def create_roi_box(self):
        R = self.euler_to_rotation_matrix(*self.roi_rpy)
        bbox = o3d.geometry.OrientedBoundingBox(self.roi_center, R, self.roi_size)
        bbox.color = (1, 0, 0)
        return bbox

    def update_roi_box(self):
        try:
            self.scene.scene.remove_geometry(self._name_roi)
        except Exception:
            pass
        self.roi_box = self.create_roi_box()
        self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())
        self.scene.force_redraw()

    # ----------------- Ground Calibration -----------------
    def calibrate_ground(self):
        self.is_calibrating = True
        self.get_logger().info("📏 Calibrating ground plane... (tunggu beberapa frame)")
        self.show_toast("Calibrating ground...")

    def save_ground_model(self, model):
        np.save(GROUND_MODEL_PATH, model)
        self.get_logger().info(f"💾 Ground model saved to {GROUND_MODEL_PATH}")
        self.show_toast("Ground model disimpan")

    def load_ground_model(self):
        if os.path.exists(GROUND_MODEL_PATH):
            self.ground_model = np.load(GROUND_MODEL_PATH)
            self.get_logger().info(f"📂 Ground model loaded from {GROUND_MODEL_PATH}")
            self.show_toast("Ground model dimuat")
        else:
            self.get_logger().warn("❌ Belum ada model ground, lakukan kalibrasi dahulu!")
            self.show_toast("Belum ada ground model")

    # ----------------- ROI persistence -----------------
    def save_roi_to_file(self):
        data = {"center": self.roi_center.tolist(), "size": self.roi_size.tolist(), "rpy": self.roi_rpy.tolist()}
        with open(CONFIG_PATH, "w") as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f"💾 ROI saved to {CONFIG_PATH}")
        self.show_toast("ROI disimpan")

    def load_roi_from_file(self):
        # os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH, "r") as f:
                data = json.load(f)
            self.roi_center = np.array(data.get("center", [0, 0, 0]), dtype=float)
            self.roi_size   = np.array(data.get("size",   [2, 2, 1]), dtype=float)
            self.roi_rpy    = np.array(data.get("rpy",    [0, 0, 0]), dtype=float)
            print(f"[INFO] ROI loaded from {CONFIG_PATH}")
        else:
            print("[INFO] No saved ROI config found, using defaults")

    def reset_roi(self):
        self.roi_center = np.array([0.0, 0.0, 0.0])
        self.roi_size   = np.array([2.0, 2.0, 1.0])
        self.roi_rpy    = np.array([0.0, 0.0, 0.0])
        self.update_roi_box()
        self.get_logger().info("🔄 ROI reset ke default")
        self.show_toast("ROI di-reset")

    # ----------------- Stable dimension measure -----------------
    def measure_dims_stable(self, pts_in, assume_non_ground=False):
        """
        Dimensi stabil:
        - (Opsional) segment ground ketat
        - Buang noise
        - Cluster DBSCAN (eps lebih ketat), gabungkan label valid
        - P & L: PCA 2D pada XY (bukan axis-aligned)
        - T: range persentil di sepanjang normal ground (jika ada), else Z
        - Smoothing: median guard + EMA
        Return: (P, L, T, obb or None) atau None.
        """
        if pts_in.shape[0] < 50:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_in)

        # 1) Segment ground bila diminta
        work = pcd
        if not assume_non_ground:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.015, ransac_n=3, num_iterations=300)
            if len(inliers) >= 100:
                work = pcd.select_by_index(inliers, invert=True)

        if work.is_empty():
            return None

        # 2) Noise removal
        work, _ = work.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        if work.is_empty():
            return None

        # 3) Cluster & kumpulkan semua label valid (eps lebih ketat)
        labels = np.array(work.cluster_dbscan(eps=0.1, min_points=6, print_progress=False))
        if (labels >= 0).any():
            obj_points = np.asarray(work.points)[labels >= 0]
        else:
            obj_points = np.asarray(work.points)

        if obj_points.shape[0] < 20:
            return None

        # Anti-QHull degeneracy
        obj_points = obj_points + np.random.normal(0, 1e-4, obj_points.shape)

        # --- T (tinggi) di sepanjang normal ground (kalau ada) ---
        if self.ground_model is not None:
            a, b, c, d = self.ground_model
            n = np.array([a, b, c], dtype=float)
            n_norm = np.linalg.norm(n) + 1e-12
            # Signed distance (positif di atas bidang)
            h = (obj_points @ n + d) / n_norm
            # Buang ekor 2% teratas & terbawah
            h2, h98 = np.percentile(h, [0.1, 99.99])
            true_height = float(h98 - h2)
        else:
            # fallback: pakai Z dunia tapi ketat
            z2, z98 = np.percentile(obj_points[:, 2], [0.1, 99.99])
            true_height = float(z98 - z2)

        # --- P & L dari PCA 2D (XY) ---
            # --- P & L dari PCA 2D (XY) ---
        XY = obj_points[:, :2]
        mu = XY.mean(axis=0, keepdims=True)
        Xc = XY - mu

        # Tambahkan sedikit regularisasi supaya orientasi PCA tidak meloncat antar frame
        cov = np.cov(Xc.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        # Urutkan eigenvalue menurun
        order = np.argsort(eigvals)[::-1]
        eigvecs = eigvecs[:, order]

        # Simpan orientasi PCA agar bisa dismoothing antar frame
        if not hasattr(self, "_last_pca_vecs"):
            self._last_pca_vecs = eigvecs.copy()
        else:
            # Exponential smoothing pada arah sumbu utama
            self._last_pca_vecs = 0.2 * eigvecs + 0.8 * self._last_pca_vecs
            # Orthonormalize ulang
            u, _, vt = np.linalg.svd(self._last_pca_vecs, full_matrices=False)
            self._last_pca_vecs = u @ vt
        V = self._last_pca_vecs  # gunakan orientasi yang dismooth

        # Proyeksikan ke sumbu PCA yang sudah stabil
        proj = Xc @ V
        # Persentil lebih ketat untuk tahan outlier
        p_low, p_high = np.percentile(proj[:, 0], [3.0, 97.0])
        q_low, q_high = np.percentile(proj[:, 1], [3.0, 97.0])
        true_length = float(p_high - p_low)
        true_width  = float(q_high - q_low)

        # Filter ukuran ekstrem antar frame
        if hasattr(self, "_last_dims_raw"):
            last_L, last_W = self._last_dims_raw
            # Batasi lonjakan per frame maksimal 10%
            true_length = np.clip(true_length, 0.9 * last_L, 1.1 * last_L)
            true_width  = np.clip(true_width, 0.9 * last_W, 1.1 * last_W)
        self._last_dims_raw = (true_length, true_width)


        # --- Smoothing & guard seperti sebelumnya ---
        # T
        self._dim_hist_height.append(true_height)
        if len(self._dim_hist_height) > 12:
            self._dim_hist_height.pop(0)
        median_h = float(np.median(self._dim_hist_height))
        if true_height > 1.25 * median_h:   # guard outlier ke atas
            true_height = median_h
        if self._last_height is not None:
            true_height = 0.28 * true_height + 0.75 * self._last_height
        self._last_height = true_height

        # P & L
        self._dim_hist_xy.append([true_length, true_width])
        if len(self._dim_hist_xy) > 12:
            self._dim_hist_xy.pop(0)
        xy_med = np.median(self._dim_hist_xy, axis=0)
        p_corr, l_corr = true_length, true_width
        # guard 15% deviasi
        if abs(p_corr - xy_med[0]) > 0.15 * max(1e-6, xy_med[0]):
            p_corr = float(xy_med[0])
        if abs(l_corr - xy_med[1]) > 0.15 * max(1e-6, xy_med[1]):
            l_corr = float(xy_med[1])
        if self._last_xy is not None:
            p_corr = 0.25 * p_corr + 0.75 * float(self._last_xy[0])
            l_corr = 0.20 * l_corr + 0.75 * float(self._last_xy[1])
        self._last_xy = np.array([p_corr, l_corr], dtype=np.float32)

        # --- OBB hanya visual (tak dipakai ukur) ---
        obb = None
        try:
            pcd_obj = o3d.geometry.PointCloud()
            pcd_obj.points = o3d.utility.Vector3dVector(obj_points)
            obb = pcd_obj.get_oriented_bounding_box()
            obb.color = (1.0, 1.0, 0.0)
        except Exception:
            obb = None

        return p_corr, l_corr, true_height, obb

    # ----------------- Callback ROS -----------------
    def cb(self, msg):
        cloud = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not cloud:
            return

        pts = np.asarray([(p[0], p[1], p[2]) for p in cloud], dtype=np.float32)
        inten = np.asarray([p[3] for p in cloud], dtype=np.float32)

        # Transform LiDAR -> world
        R_lidar = self.get_lidar_rotation_matrix()
        pts = (R_lidar @ pts.T).T

        # Colorize by intensity
        cmap = plt.get_cmap("turbo")
        inten_n = (inten - inten.min()) / (inten.ptp() + 1e-6)
        cols = cmap(inten_n)[:, :3].astype(np.float32)

        # ROI mask
        bbox = self.create_roi_box()
        roi_indices = bbox.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(pts))
        roi_indices = np.array(roi_indices, dtype=int)
        dim_text = "Belum ada objek terdeteksi"

        # Prepare for OBB redraw
        def remove_old_obb():
            try:
                self.scene.scene.remove_geometry(self._name_obb)
            except Exception:
                pass

        remove_old_obb()

        if len(roi_indices) > 50:
            pts_roi = pts[roi_indices]

            # Kalibrasi ground saat diminta
            if self.is_calibrating:
                try:
                    pcd_roi = o3d.geometry.PointCloud()
                    pcd_roi.points = o3d.utility.Vector3dVector(pts_roi)
                    plane_model, inliers = pcd_roi.segment_plane(
                        distance_threshold=self.calib_thresh, ransac_n=3, num_iterations=300
                    )
                    self.save_ground_model(plane_model)
                    self.is_calibrating = False
                    self.ground_model = plane_model
                    a, b, c, d = plane_model
                    self.get_logger().info(
                        f"✅ Ground calibrated: a={a:.3f}, b={b:.3f}, c={c:.3f}, d={d:.3f}"
                    )
                    self.show_toast("Kalibrasi ground selesai")
                except Exception as e:
                    self.is_calibrating = False
                    self.get_logger().warn(f"Kalibrasi gagal: {e}")
                    self.show_toast("Kalibrasi gagal")

            # Color ground (opsional) + non-ground extraction untuk measure
            assume_non_ground = False
            pts_for_measure = pts_roi
            if self.ground_model is not None:
                a, b, c, d = self.ground_model
                denom = np.sqrt(a ** 2 + b ** 2 + c ** 2) + 1e-9
                heights = np.abs((a * pts_roi[:, 0] + b * pts_roi[:, 1] + c * pts_roi[:, 2] + d) / denom)
                ground_mask = heights < self.ground_thresh
                non_ground_mask = ~ground_mask
                if self.show_ground_color:
                    # hanya warnai ground jadi hijau, tidak mematikan deteksi
                    cols[roi_indices[ground_mask]] = [0.0, 1.0, 0.0]
                # Gunakan hanya non-ground untuk pengukuran
                if non_ground_mask.sum() > 20:
                    pts_for_measure = pts_roi[non_ground_mask]
                    assume_non_ground = True

            # Ukur dimensi stabil
            result = self.measure_dims_stable(pts_for_measure, assume_non_ground=assume_non_ground)
            if result is not None:
                P, L, T, obb = result
                dim_text = f"Objek: P={P:.3f} m, L={L:.3f} m, T={T:.3f} m"
                print(dim_text)

                if obb is not None:
                    mat_box = rendering.MaterialRecord()
                    mat_box.shader = "unlitLine"
                    mat_box.line_width = 4.0
                    self.scene.scene.add_geometry(self._name_obb, obb, mat_box)

        # Update label di UI
        gui.Application.instance.post_to_main_thread(
            self.window, lambda: setattr(self.dim_label, "text", dim_text)
        )

        # Update point cloud
        self.pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd.colors = o3d.utility.Vector3dVector(cols)

        def update_scene():
            # Clear hanya geometri dinamis, lalu re-add
            self.scene.scene.clear_geometry()
            self.scene.scene.add_geometry(self._name_cloud, self.pcd, rendering.MaterialRecord())
            self.scene.scene.add_geometry(self._name_roi, self.roi_box, self.roi_material())
            # OBB jika ada sudah ditambahkan di atas (akan hilang saat clear, jadi re-add bila perlu)
            # Untuk kesederhanaan, bila obb dibuat pada frame ini sudah ditambahkan; kalau ingin persist,
            # bisa simpan dan re-add di sini.

            if not self._camera_set:
                bbox = self.pcd.get_axis_aligned_bounding_box()
                center = bbox.get_center()
                radius = np.linalg.norm(bbox.get_extent()) * 0.5 + 1.0
                eye = center + np.array([radius, radius, radius])
                self.scene.scene.camera.look_at(center.tolist(), eye.tolist(), [0, 0, 1])
                self._camera_set = True
            self.scene.force_redraw()

        gui.Application.instance.post_to_main_thread(self.window, update_scene)

    # ----------------- Layout -----------------
    def on_layout(self, ctx):
        r = self.window.content_rect
        panel_width = 360
        self.scene.frame = gui.Rect(r.x, r.y, r.width, r.height)
        self.panel.frame = gui.Rect(r.get_right() - panel_width, r.y, panel_width, r.height)

    def show_window(self):
        self.window.show(True)
        self.get_logger().info("A window has appear")
        self.scene.force_redraw()

    def hide_window(self):
        self.window.show(False)
        self.get_logger().info("The window is hidden")
        self.scene.force_redraw()

    def toggle_window(self):
        if self.window.is_visible:
            self.hide_window()
        else:
            self.show_window()

# ----------------- Main -----------------
# def main(args=None):
#     global GUI_INSTANCE
#     rclpy.init(args=args)
#     app = gui.Application.instance
#     app.initialize()

#     node = LivoxGUI(app)
#     ros_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     ros_t.start()

#     app.run()

#     node.save_roi_to_file()
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
