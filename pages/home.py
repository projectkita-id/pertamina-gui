import os
import time
import signal
import threading
import customtkinter as ctk
import open3d.visualization.gui as gui
import rclpy

import components.header as header
import components.connection_check as connection_check

from multiprocessing import Process, Value, Event
import atexit
from components.lidar_viewer import LivoxCalib

# ================= GLOBAL =================

PROCESS = []

P_VAL = Value("d", 0.0)
L_VAL = Value("d", 0.0)
T_VAL = Value("d", 0.0)
DATA_EVENT = Event()

VISIBLE_FLAG_PATH = "/tmp/lidar_visible"
CALIB_FLAG_PATH = "/tmp/lidar_calib_mode"


# ================= PROCESS CONTROL ================

def kill_viewer():
    """Matikan child viewer secara keras + bersih."""
    print("[HOME] Killing viewer process...")

    for p in PROCESS:
        if p.is_alive():
            try:
                os.kill(p.pid, signal.SIGTERM)
                p.join(timeout=0.5)
            except Exception:
                pass

            if p.is_alive():
                try:
                    os.kill(p.pid, signal.SIGKILL)
                except Exception:
                    pass

    PROCESS.clear()


def run_viewer(p_val, l_val, t_val, data_event):
    rclpy.init()
    app = gui.Application.instance
    app.initialize()

    node = LivoxCalib(app, p_val, l_val, t_val, data_event)

    def safe_spin(n):
        try:
            rclpy.spin(n)
        except Exception:
            pass  # <--- Telan semua error, termasuk ExternalShutdownException

    ros_thread = threading.Thread(target=safe_spin, args=(node,), daemon=True)

    ros_thread.start()

    app.run()

    # Setelah window Open3D ditutup → hentikan ROS & keluar cepat
    try:
        node.save_roi_to_file()
    except:
        pass

    try:
        node.destroy_node()
    except:
        pass

    try:
        rclpy.shutdown()
    except:
        pass

    try:
        ros_thread.join(timeout=0.5)
    except:
        pass

    # Pastikan proses benar-benar mati
    os._exit(0)


def _cleanup_child():
    """Dipanggil saat proses GUI mati normal / crash, untuk jaga-jaga."""
    try:
        kill_viewer()
    except Exception:
        pass

atexit.register(_cleanup_child)


def start_viewer_once():
    """Start viewer jika belum berjalan."""
    if PROCESS and PROCESS[0].is_alive():
        return PROCESS[0]

    p = Process(target=run_viewer, args=(P_VAL, L_VAL, T_VAL, DATA_EVENT))
    p.daemon = True  # child mati kalau parent (GUI) mati
    PROCESS.clear()
    PROCESS.append(p)
    p.start()
    return p


# ================= GUI PAGE =================

class Home(ctk.CTkFrame):
    def __init__(self, parent, show_page):
        super().__init__(parent, fg_color="white")
        header.Header(self, show_page).pack(fill="x")

        # HOME MODE ⇒ hapus panel CALIB
        if os.path.exists(CALIB_FLAG_PATH):
            os.remove(CALIB_FLAG_PATH)

        # Start viewer sekali
        start_viewer_once()

        # When GUI closed, kill viewer
        def on_close():
            print("[GUI] Closing, killing viewer...")
            kill_viewer()
            if os.path.exists(VISIBLE_FLAG_PATH):
                os.remove(VISIBLE_FLAG_PATH)
            parent.destroy()

        parent.protocol("WM_DELETE_WINDOW", on_close)
        parent._on_close_callback = on_close  # supaya bisa dipanggil dari main.py

        # Thread consumer shared memory
        self.stop_flag = threading.Event()
        threading.Thread(target=self.consume_dimension, daemon=True).start()

        self.bind("<Destroy>", lambda e: self.stop_flag.set())

        # UI
        fontSmall = ctk.CTkFont(family="Verdana", size=14, weight="bold")
        font = ctk.CTkFont(family="Verdana", size=24, weight="bold")
        fontBig = ctk.CTkFont(family="Verdana", size=48, weight="bold")

        container = ctk.CTkFrame(self, fg_color="white")
        container.pack(fill="both", expand=True, padx=20, pady=20)

        # tombol viewer
        self.openWindow = ctk.CTkButton(
            container,
            text="Open 3D Lidar Viewer",
            font=fontSmall,
            fg_color="#F01382",
            command=self.toggle_lidar,
            height=40
        )
        self.openWindow.pack(fill="x", pady=(0, 20))

        # nilai dimensi
        valueFrame = ctk.CTkFrame(container, fg_color="white")
        valueFrame.pack(fill="both", expand=True)

        self.lengthValue = self._build_box(valueFrame, "Panjang", font, fontBig)
        self.widthValue  = self._build_box(valueFrame, "Lebar", font, fontBig)
        self.heightValue = self._build_box(valueFrame, "Tinggi", font, fontBig)

        # auto update button text
        self.sync_button_text()
        self.last_update = 0

    def _build_box(self, parent, title, font, fontBig):
        box = ctk.CTkFrame(
            parent, fg_color="white",
            border_width=2, border_color="darkgrey",
            corner_radius=20,
        )
        box.pack(fill="x", pady=10)
        ctk.CTkLabel(box, text=title, font=font, text_color="#F01382").pack(pady=(40, 0))
        val = ctk.CTkLabel(box, text="0", font=fontBig, text_color="#F01382")
        val.pack(pady=40)
        return val

    # =============== BUTTON VIEWER ===============

    def toggle_lidar(self):
        if os.path.exists(VISIBLE_FLAG_PATH):
            os.remove(VISIBLE_FLAG_PATH)
        else:
            open(VISIBLE_FLAG_PATH, "w").close()

    def sync_button_text(self):
        if os.path.exists(VISIBLE_FLAG_PATH):
            self.openWindow.configure(text="Hide 3D Viewer")
        else:
            self.openWindow.configure(text="Open 3D Viewer")
        self.after(300, self.sync_button_text)

    # =============== SHARED MEMORY ===============

    def consume_dimension(self):
        while not self.stop_flag.is_set():
            DATA_EVENT.wait()
            if self.stop_flag.is_set():
                break

            p = P_VAL.value
            l = L_VAL.value
            t = T_VAL.value

            DATA_EVENT.clear()

            self.after(0, self.update_dim, p, l, t)

    def update_dim(self, p, l, t):
        now = time.time()
        if now - self.last_update < 0.1:
            return
        self.lengthValue.configure(text=f"{p:.2f}")
        self.widthValue.configure(text=f"{l:.2f}")
        self.heightValue.configure(text=f"{t:.2f}")
        self.last_update = now
