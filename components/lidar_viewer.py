#!/usr/bin/env python3
from rclpy.node import Node
from datetime import datetime
from multiprocessing import Queue
from sensor_msgs.msg import PointCloud2
from components.db_config import get_conf
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

# ==== Global config paths (agar sinkron antar proses) ====
BASE_DIR = os.path.abspath(os.getcwd())  # Folder tempat main.py dijalankan
CONFIG_PATH = os.path.join(BASE_DIR, "roi_config.json")
GROUND_MODEL_PATH = os.path.join(BASE_DIR, "ground_plane.npy")
MEASURE_CONFIG_PATH = os.path.join(BASE_DIR, "measure_config.json")

def load_measure_config():
    """Muat konfigurasi dari measure_config.json dengan fallback default."""
    default = {
        "general": {
            "frame_interval": 0.1,
            "point_size": 5.0,
            "lidar_tilt_deg": 30.0,
            "lidar_invert_z": True
        },
        "ground": {
            "ground_thresh": 0.03,
            "calib_thresh": 0.02,
            "distance_threshold": 0.015,
            "ransac_n": 3,
            "num_iterations": 300
        },
        "noise_filter": {"nb_neighbors": 20, "std_ratio": 2.0},
        "cluster": {"eps": 0.1, "min_points": 6},
        "adaptive_filter": {"min_thresh": 0.02, "max_thresh": 0.06, "scale_factor": 3.0},
        "height_smoothing": {
            "max_history": 20,
            "guard_high": 1.3,
            "guard_low": 0.7,
            "ema_factor_new": 0.3,
            "ema_factor_old": 0.7
        },
        "dimension_smoothing": {
            "max_history": 20,
            "deviation_guard": 0.15,
            "ema_factor_length_new": 0.25,
            "ema_factor_length_old": 0.75,
            "ema_factor_width_new": 0.25,
            "ema_factor_width_old": 0.75
        }
    }

    if not os.path.exists(MEASURE_CONFIG_PATH):
        with open(MEASURE_CONFIG_PATH, "w") as f:
            json.dump(default, f, indent=2)
        return default

    try:
        with open(MEASURE_CONFIG_PATH, "r") as f:
            data = json.load(f)
        for k, v in data.items():
            if isinstance(v, dict) and k in default:
                default[k].update(v)
            else:
                default[k] = v
        print(f"[INFO] Loaded measure_config.json from {MEASURE_CONFIG_PATH}")
        return default
    except Exception as e:
        print(f"[WARN] Gagal load measure_config.json: {e}")
        return default


def watch_measure_config(node):
    """Pantau perubahan file konfigurasi dan reload otomatis."""
    def mtime():
        return os.path.getmtime(MEASURE_CONFIG_PATH) if os.path.exists(MEASURE_CONFIG_PATH) else 0
    last = mtime()
    while True:
        time.sleep(2)
        new = mtime()
        if new != last:
            last = new
            print("[INFO] measure_config.json updated, reloading...")
            node.measure_cfg = load_measure_config()


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
    retries = 3  # Max retry kalau timeout
    while retries > 0:
        try:
            conn = pymysql.connect(
                host=address,
                user=username,
                password=password,
                port=port,
                database=db_name,
                charset='utf8mb4',
                cursorclass=pymysql.cursors.DictCursor,
                connect_timeout=10  # Timeout connect 10 detik (default infinite, bikin lelet)
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
            break  # Sukses, keluar loop
        except pymysql.MySQLError as e:
            retries -= 1
            print(f"[ERROR] Gagal mengirim data ke database (retry {3-retries}): {str(e)}")
            if retries == 0:
                print("[ERROR] Max retry reached, skipping upload")
            time.sleep(1)  # Delay sedikit sebelum retry
        finally:
            if conn:
                conn.close()

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
    retries = 3  # Max retry kalau timeout
    while retries > 0:
        try:
            conn = pymysql.connect(
                host=address,
                user=username,
                password=password,
                port=port,
                database=db_name,
                charset='utf8mb4',
                cursorclass=pymysql.cursors.DictCursor,
                connect_timeout=10  # Timeout connect 10 detik (default infinite, bikin lelet)
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
            break  # Sukses, keluar loop
        except pymysql.MySQLError as e:
            retries -= 1
            print(f"[ERROR] Gagal mengirim data ke database (retry {3-retries}): {str(e)}")
            if retries == 0:
                print("[ERROR] Max retry reached, skipping upload")
            time.sleep(1)  # Delay sedikit sebelum retry
        finally:
            if conn:
                conn.close()

class LivoxGUI(Node):
    def __init__(self, app, p_val=None, l_val=None, t_val=None, data_event=None):
        global GUI_INSTANCE
        super().__init__("livox_open3d_gui")
        # ==== Load konfigurasi JSON ====
        self.measure_cfg = load_measure_config()

        # Terapkan parameter umum
        g = self.measure_cfg["general"]
        self.frame_interval = g["frame_interval"]
        self.point_size = g["point_size"]
        self.set_lidar_orientation(g["lidar_tilt_deg"], g["lidar_invert_z"])

        gr = self.measure_cfg["ground"]
        self.ground_thresh = gr["ground_thresh"]
        self.calib_thresh = gr["calib_thresh"]

        # Jalankan watcher agar reload otomatis bila file JSON berubah
        threading.Thread(target=watch_measure_config, args=(self,), daemon=True).start()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, qos)

        self.p_val = p_val
        self.l_val = l_val
        self.t_val = t_val
        self.data_event = data_event

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
         # ==== Frame & point size control ====
        self.last_update_time = self.get_clock().now()
        self.frame_interval = 0.1   # detik per frame (0.1 = 10 Hz)
        self.point_size = 5.0       # ukuran titik point cloud

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

         # ==== ROI & Ground auto-reload watcher ====
        self._config_watch_thread = threading.Thread(target=self._watch_config_files, daemon=True)
        self._config_watch_thread.start()

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

    def _watch_config_files(self):
        """Pantau perubahan file ROI & ground plane, reload otomatis jika berubah"""
        def safe_mtime(path):
            try:
                return os.path.getmtime(path) if os.path.exists(path) else 0
            except Exception:
                return 0

        roi_mtime = safe_mtime(CONFIG_PATH)
        ground_mtime = safe_mtime(GROUND_MODEL_PATH)

        while True:
            time.sleep(2)
            new_roi_mtime = safe_mtime(CONFIG_PATH)
            new_ground_mtime = safe_mtime(GROUND_MODEL_PATH)

            # Jika ROI berubah
            if new_roi_mtime != roi_mtime:
                roi_mtime = new_roi_mtime
                print("[INFO] ROI config file changed, reloading in Home viewer...")
                gui.Application.instance.post_to_main_thread(
                    self.window,
                    lambda: (self.load_roi_from_file(), self.update_roi_box())
                )

            # Jika ground model berubah
            if new_ground_mtime != ground_mtime:
                ground_mtime = new_ground_mtime
                print("[INFO] Ground model file changed, reloading in Home viewer...")
                gui.Application.instance.post_to_main_thread(
                    self.window,
                    lambda: self.load_ground_model()
                )


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
        """Mengukur dimensi objek dengan auto-adaptive parameter (tanpa tuning manual)."""
        cfg = self.measure_cfg
        gnd = cfg["ground"]
        nf  = cfg["noise_filter"]
        clu = cfg["cluster"]

        if pts_in.shape[0] < 50:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_in)

        # --- 1) Segmentasi ground (RANSAC) ---
        work = pcd
        ground_plane = None
        if not assume_non_ground:
            try:
                plane_model, inliers = pcd.segment_plane(
                    distance_threshold=gnd["distance_threshold"],
                    ransac_n=gnd["ransac_n"],
                    num_iterations=gnd["num_iterations"]
                )
                if len(inliers) >= 100:
                    ground_plane = plane_model
                    work = pcd.select_by_index(inliers, invert=True)
            except Exception as e:
                self.get_logger().warn(f"Gagal segment ground: {e}")

        if work.is_empty():
            return None

        # --- 2) Noise filter ---
        work, _ = work.remove_statistical_outlier(
            nb_neighbors=nf["nb_neighbors"], std_ratio=nf["std_ratio"]
        )
        if work.is_empty():
            return None

        # --- 3) Cluster ---
        labels = np.array(work.cluster_dbscan(
            eps=clu["eps"], min_points=clu["min_points"], print_progress=False
        ))
        if (labels >= 0).any():
            obj_points = np.asarray(work.points)[labels >= 0]
        else:
            obj_points = np.asarray(work.points)
        if obj_points.shape[0] < 30:
            return None

        # --- 4) Estimasi kasar dimensi (tanpa smoothing dulu) ---
        # Tinggi kasar (persentil robust)
        z2, z98   = np.percentile(obj_points[:, 2], [1, 99])
        height_raw = float(z98 - z2)

        # PCA 2D pada XY (sudah di-center)
        XY = obj_points[:, :2]
        Xc = XY - XY.mean(axis=0, keepdims=True)
        cov = np.cov(Xc.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvecs = eigvecs[:, np.argsort(eigvals)[::-1]]

        # ---- PCA-lock anti-flip TANPA perlu ubah __init__
        if not hasattr(self, "_eigvecs_prev"):
            self._eigvecs_prev = None
        if self._eigvecs_prev is not None:
            if np.dot(eigvecs[:, 0], self._eigvecs_prev[:, 0]) < 0:
                eigvecs[:, 0] *= -1
            if np.dot(eigvecs[:, 1], self._eigvecs_prev[:, 1]) < 0:
                eigvecs[:, 1] *= -1
        self._eigvecs_prev = eigvecs.copy()

        proj = Xc @ eigvecs
        p_low, p_high = np.percentile(proj[:, 0], [1, 99])
        q_low, q_high = np.percentile(proj[:, 1], [1, 99])
        length_raw = float(p_high - p_low)
        width_raw  = float(q_high - q_low)

        # Anti-swap label jika minor > major
        if width_raw > length_raw:
            length_raw, width_raw = width_raw, length_raw

        # --- 5) Skala adaptif (pakai skema existing, tetap kompatibel)
        scale_ref   = max(length_raw, width_raw, height_raw)
        scale_factor = np.clip(scale_ref / 5.0, 0.5, 2.0)

        base_hs = cfg["height_smoothing"]
        base_ds = cfg["dimension_smoothing"]

        ema_height_new = np.clip(base_hs["ema_factor_new"] * scale_factor, 0.1, 0.6)
        ema_height_old = 1.0 - ema_height_new
        ema_length_new = np.clip(base_ds["ema_factor_length_new"] * scale_factor, 0.1, 0.6)
        ema_length_old = 1.0 - ema_length_new
        ema_width_new  = np.clip(base_ds["ema_factor_width_new"] * scale_factor, 0.1, 0.6)
        ema_width_old  = 1.0 - ema_width_new

        guard_high = np.clip(base_hs["guard_high"] + 0.2 * (scale_factor - 1.0), 1.0, 1.6)
        guard_low  = np.clip(base_hs["guard_low"]  - 0.1 * (scale_factor - 1.0), 0.5, 0.9)

        # --- 6) Hitung tinggi sebenarnya (pakai model ground bila ada) ---
        if ground_plane is not None:
            a, b, c, d = ground_plane
            n = np.array([a, b, c], dtype=float)
            n_norm = np.linalg.norm(n) + 1e-12
            hvals = (obj_points @ n + d) / n_norm
            h2, h98 = np.percentile(hvals, [1, 99])
            true_height = float(h98 - h2)
        else:
            true_height = height_raw

        # --- 7) Smoothing T (pakai buffer & EMA yang SUDAH ada) ---
        self._dim_hist_height.append(true_height)
        if len(self._dim_hist_height) > base_hs["max_history"]:
            self._dim_hist_height.pop(0)
        median_h = float(np.median(self._dim_hist_height))
        if true_height > guard_high * median_h or true_height < guard_low * median_h:
            true_height = median_h
        if self._last_height is not None:
            true_height = ema_height_new * true_height + ema_height_old * self._last_height
        self._last_height = true_height

        # --- 8) Smoothing P & L (pakai buffer & EMA yang SUDAH ada) ---
        self._dim_hist_xy.append([length_raw, width_raw])
        if len(self._dim_hist_xy) > base_ds["max_history"]:
            self._dim_hist_xy.pop(0)
        xy_med = np.median(self._dim_hist_xy, axis=0)
        p_corr, l_corr = length_raw, width_raw
        if abs(p_corr - xy_med[0]) > base_ds["deviation_guard"] * max(1e-6, xy_med[0]):
            p_corr = float(xy_med[0])
        if abs(l_corr - xy_med[1]) > base_ds["deviation_guard"] * max(1e-6, xy_med[1]):
            l_corr = float(xy_med[1])
        if self._last_xy is not None:
            p_corr = ema_length_new * p_corr + ema_length_old * self._last_xy[0]
            l_corr = ema_width_new  * l_corr + ema_width_old  * self._last_xy[1]
        self._last_xy = np.array([p_corr, l_corr], dtype=np.float32)

        # --- 9) Koreksi linier L & T (biarkan P apa adanya) ---
        # Ambil dari config bila ada, kalau tidak fallback ke konstanta kalibrasi
        cm = cfg.get("correction_matrix", {})
        cL = cm.get("L", [1.73118631, -0.99775355, -0.00802691])
        cT = cm.get("T", [-0.94557996,  1.70870925,  0.22837981])

        L_det, T_det = float(l_corr), float(self._last_height if self._last_height is not None else true_height)
        L_corr = cL[0]*L_det + cL[1]*T_det + cL[2]
        T_corr = cT[0]*L_det + cT[1]*T_det + cT[2]
        L_corr = float(np.clip(L_corr, 0.05, 50.0))
        T_corr = float(np.clip(T_corr, 0.05, 50.0))

        # --- 10) (Optional) OBB untuk visualisasi (tetap seperti semula) ---
        obb = None
        try:
            pcd_obj = o3d.geometry.PointCloud()
            pcd_obj.points = o3d.utility.Vector3dVector(obj_points)
            obb = pcd_obj.get_oriented_bounding_box()
            obb.color = (1.0, 1.0, 0.0)
        except Exception:
            pass

        return float(p_corr), float(L_corr), float(T_corr), obb



    # ----------------- Callback ROS -----------------
    def cb(self, msg):
        # --- Batasi update rate sesuai frame_interval ---
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt < self.frame_interval:
            return  # skip frame, terlalu cepat
        self.last_update_time = now

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
                    print("sending to gui directly via shared mem")
                    if self.p_val is not None and self.data_event is not None:
                        self.p_val.value = P
                        self.l_val.value = L
                        self.t_val.value = T
                        self.data_event.set()  # Signal ke GUI bahwa data ready
                    threading.Thread(target=upload, args=(P, L, T), daemon=True).start()
                except Exception as e:
                    print(f"nope, sending error: {type(e).__name__} - {str(e)}")
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
    def __init__(self, app, p_val=None, l_val=None, t_val=None, data_event=None):
        global GUI_INSTANCE
        super().__init__("livox_open3d_calib")
        # ==== Load konfigurasi JSON ====
        self.measure_cfg = load_measure_config()

        # Terapkan parameter umum
        g = self.measure_cfg["general"]
        self.frame_interval = g["frame_interval"]
        self.point_size = g["point_size"]
        self.set_lidar_orientation(g["lidar_tilt_deg"], g["lidar_invert_z"])

        gr = self.measure_cfg["ground"]
        self.ground_thresh = gr["ground_thresh"]
        self.calib_thresh = gr["calib_thresh"]

        # Jalankan watcher agar reload otomatis bila file JSON berubah
        threading.Thread(target=watch_measure_config, args=(self,), daemon=True).start()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, qos)

        self.p_val = p_val
        self.l_val = l_val
        self.t_val = t_val
        self.data_event = data_event

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
        # ==== Frame & point size control ====
        self.last_update_time = self.get_clock().now()
        self.frame_interval = 0.1   # detik per frame (0.1 = 10 Hz)
        self.point_size = 5.0       # ukuran titik point cloud

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
        self.panel.add_child(self.make_plus_minus_row("X", "roi_center", 0, 0.05))
        self.panel.add_child(self.make_plus_minus_row("Y", "roi_center", 1, 0.05))
        self.panel.add_child(self.make_plus_minus_row("Z", "roi_center", 2, 0.05))
        self.panel.add_child(gui.Label("Size (m)"))
        self.panel.add_child(self.make_plus_minus_row("Size X", "roi_size", 0, 0.1))
        self.panel.add_child(self.make_plus_minus_row("Size Y", "roi_size", 1, 0.1))
        self.panel.add_child(self.make_plus_minus_row("Size Z", "roi_size", 2, 0.1))
        self.panel.add_child(gui.Label("Rotation (deg)"))
        self.panel.add_child(self.make_plus_minus_row("Roll",  "roi_rpy", 0, 0.5))
        self.panel.add_child(self.make_plus_minus_row("Pitch", "roi_rpy", 1, 0.5))
        self.panel.add_child(self.make_plus_minus_row("Yaw",   "roi_rpy", 2, 0.5))

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
        """Mengukur dimensi objek dengan auto-adaptive parameter (tanpa tuning manual)."""
        cfg = self.measure_cfg
        gnd = cfg["ground"]
        nf  = cfg["noise_filter"]
        clu = cfg["cluster"]

        if pts_in.shape[0] < 50:
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_in)

        # --- 1) Segmentasi ground (RANSAC) ---
        work = pcd
        ground_plane = None
        if not assume_non_ground:
            try:
                plane_model, inliers = pcd.segment_plane(
                    distance_threshold=gnd["distance_threshold"],
                    ransac_n=gnd["ransac_n"],
                    num_iterations=gnd["num_iterations"]
                )
                if len(inliers) >= 100:
                    ground_plane = plane_model
                    work = pcd.select_by_index(inliers, invert=True)
            except Exception as e:
                self.get_logger().warn(f"Gagal segment ground: {e}")

        if work.is_empty():
            return None

        # --- 2) Noise filter ---
        work, _ = work.remove_statistical_outlier(
            nb_neighbors=nf["nb_neighbors"], std_ratio=nf["std_ratio"]
        )
        if work.is_empty():
            return None

        # --- 3) Cluster ---
        labels = np.array(work.cluster_dbscan(
            eps=clu["eps"], min_points=clu["min_points"], print_progress=False
        ))
        if (labels >= 0).any():
            obj_points = np.asarray(work.points)[labels >= 0]
        else:
            obj_points = np.asarray(work.points)
        if obj_points.shape[0] < 30:
            return None

        # --- 4) Estimasi kasar dimensi (tanpa smoothing dulu) ---
        # Tinggi kasar (persentil robust)
        z2, z98   = np.percentile(obj_points[:, 2], [1, 99])
        height_raw = float(z98 - z2)

        # PCA 2D pada XY (sudah di-center)
        XY = obj_points[:, :2]
        Xc = XY - XY.mean(axis=0, keepdims=True)
        cov = np.cov(Xc.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvecs = eigvecs[:, np.argsort(eigvals)[::-1]]

        # ---- PCA-lock anti-flip TANPA perlu ubah __init__
        if not hasattr(self, "_eigvecs_prev"):
            self._eigvecs_prev = None
        if self._eigvecs_prev is not None:
            if np.dot(eigvecs[:, 0], self._eigvecs_prev[:, 0]) < 0:
                eigvecs[:, 0] *= -1
            if np.dot(eigvecs[:, 1], self._eigvecs_prev[:, 1]) < 0:
                eigvecs[:, 1] *= -1
        self._eigvecs_prev = eigvecs.copy()

        proj = Xc @ eigvecs
        p_low, p_high = np.percentile(proj[:, 0], [1, 99])
        q_low, q_high = np.percentile(proj[:, 1], [1, 99])
        length_raw = float(p_high - p_low)
        width_raw  = float(q_high - q_low)

        # Anti-swap label jika minor > major
        if width_raw > length_raw:
            length_raw, width_raw = width_raw, length_raw

        # --- 5) Skala adaptif (pakai skema existing, tetap kompatibel)
        scale_ref   = max(length_raw, width_raw, height_raw)
        scale_factor = np.clip(scale_ref / 5.0, 0.5, 2.0)

        base_hs = cfg["height_smoothing"]
        base_ds = cfg["dimension_smoothing"]

        ema_height_new = np.clip(base_hs["ema_factor_new"] * scale_factor, 0.1, 0.6)
        ema_height_old = 1.0 - ema_height_new
        ema_length_new = np.clip(base_ds["ema_factor_length_new"] * scale_factor, 0.1, 0.6)
        ema_length_old = 1.0 - ema_length_new
        ema_width_new  = np.clip(base_ds["ema_factor_width_new"] * scale_factor, 0.1, 0.6)
        ema_width_old  = 1.0 - ema_width_new

        guard_high = np.clip(base_hs["guard_high"] + 0.2 * (scale_factor - 1.0), 1.0, 1.6)
        guard_low  = np.clip(base_hs["guard_low"]  - 0.1 * (scale_factor - 1.0), 0.5, 0.9)

        # --- 6) Hitung tinggi sebenarnya (pakai model ground bila ada) ---
        if ground_plane is not None:
            a, b, c, d = ground_plane
            n = np.array([a, b, c], dtype=float)
            n_norm = np.linalg.norm(n) + 1e-12
            hvals = (obj_points @ n + d) / n_norm
            h2, h98 = np.percentile(hvals, [1, 99])
            true_height = float(h98 - h2)
        else:
            true_height = height_raw

        # --- 7) Smoothing T (pakai buffer & EMA yang SUDAH ada) ---
        self._dim_hist_height.append(true_height)
        if len(self._dim_hist_height) > base_hs["max_history"]:
            self._dim_hist_height.pop(0)
        median_h = float(np.median(self._dim_hist_height))
        if true_height > guard_high * median_h or true_height < guard_low * median_h:
            true_height = median_h
        if self._last_height is not None:
            true_height = ema_height_new * true_height + ema_height_old * self._last_height
        self._last_height = true_height

        # --- 8) Smoothing P & L (pakai buffer & EMA yang SUDAH ada) ---
        self._dim_hist_xy.append([length_raw, width_raw])
        if len(self._dim_hist_xy) > base_ds["max_history"]:
            self._dim_hist_xy.pop(0)
        xy_med = np.median(self._dim_hist_xy, axis=0)
        p_corr, l_corr = length_raw, width_raw
        if abs(p_corr - xy_med[0]) > base_ds["deviation_guard"] * max(1e-6, xy_med[0]):
            p_corr = float(xy_med[0])
        if abs(l_corr - xy_med[1]) > base_ds["deviation_guard"] * max(1e-6, xy_med[1]):
            l_corr = float(xy_med[1])
        if self._last_xy is not None:
            p_corr = ema_length_new * p_corr + ema_length_old * self._last_xy[0]
            l_corr = ema_width_new  * l_corr + ema_width_old  * self._last_xy[1]
        self._last_xy = np.array([p_corr, l_corr], dtype=np.float32)

        # --- 9) Koreksi linier L & T (biarkan P apa adanya) ---
        # Ambil dari config bila ada, kalau tidak fallback ke konstanta kalibrasi
        cm = cfg.get("correction_matrix", {})
        cL = cm.get("L", [1.73118631, -0.99775355, -0.00802691])
        cT = cm.get("T", [-0.94557996,  1.70870925,  0.22837981])

        L_det, T_det = float(l_corr), float(self._last_height if self._last_height is not None else true_height)
        L_corr = cL[0]*L_det + cL[1]*T_det + cL[2]
        T_corr = cT[0]*L_det + cT[1]*T_det + cT[2]
        L_corr = float(np.clip(L_corr, 0.05, 50.0))
        T_corr = float(np.clip(T_corr, 0.05, 50.0))

        # --- 10) (Optional) OBB untuk visualisasi (tetap seperti semula) ---
        obb = None
        try:
            pcd_obj = o3d.geometry.PointCloud()
            pcd_obj.points = o3d.utility.Vector3dVector(obj_points)
            obb = pcd_obj.get_oriented_bounding_box()
            obb.color = (1.0, 1.0, 0.0)
        except Exception:
            pass

        return float(p_corr), float(L_corr), float(T_corr), obb



    # ----------------- Callback ROS -----------------
    def cb(self, msg):
        # --- Batasi update rate sesuai frame_interval ---
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        if dt < self.frame_interval:
            return  # skip frame, terlalu cepat
        self.last_update_time = now
        
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
                    print("sending to gui directly via shared mem")
                    if self.p_val is not None and self.data_event is not None:
                        self.p_val.value = P
                        self.l_val.value = L
                        self.t_val.value = T
                        self.data_event.set()  # Signal ke GUI bahwa data ready
                    threading.Thread(target=upload, args=(P, L, T), daemon=True).start()
                except Exception as e:
                    print(f"nope, sending error: {type(e).__name__} - {str(e)}")
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
