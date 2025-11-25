import os
import time
import queue
import rclpy
import signal
import pymysql
import threading
import customtkinter as ctk
import components.header as header
import open3d.visualization.gui as gui
import components.connection_check as connection_check

from components.lidar_viewer import LivoxGUI
from rclpy.executors import MultiThreadedExecutor
from multiprocessing import Process, Value, Event

# DIMENSION_QUEUE = Queue(maxsize=1000)
PROCESS = []

P_VAL = Value('d', 0.0)
L_VAL = Value('d', 0.0)
T_VAL = Value('d', 0.0)
DATA_EVENT = Event()

def kill():
    print("Killing processes...")
    os.system("pkill -f 'livox_ros_driver2'")
    for p in PROCESS:
        if p.is_alive():
            print(f"Terminating process {p.pid}...")
            os.kill(p.pid, signal.SIGTERM)
            p.join(timeout=2)
            if p.is_alive():
                print(f"Process {p.pid} did not terminate, killing...")
                os.kill(p.pid, signal.SIGKILL)
                p.join()
    PROCESS.clear()

def run_open3d_viewer(p_val, l_val, t_val, data_event):
    rclpy.init()
    app = gui.Application.instance
    app.initialize()

    node = LivoxGUI(app, p_val=p_val, l_val=l_val, t_val=t_val, data_event=data_event)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    app.run()
    node.save_roi_to_file()
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

def open3d_thread():
    p = Process(target=run_open3d_viewer, args=(P_VAL, L_VAL, T_VAL, DATA_EVENT))
    PROCESS.append(p)
    p.start()
    return p

class Home(ctk.CTkFrame) :
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="white")
        header.Header(self, show_page).pack(fill="x")

        if not PROCESS:  # hanya buat viewer pertama kali
            open3d_thread()
        else:
            print("[INFO] Open3D viewer sudah berjalan, tidak membuat ulang.")


        def on_close():
            print("Closing application...")
            # Setelah GUI ditutup, hentikan semua proses ros2 launch
            os.system("pkill -f 'livox_ros_driver2'")
            kill()
            if os.path.exists("/tmp/lidar_visible"):
                os.remove("/tmp/lidar_visible")
            parent.destroy()

        self.stop_consumer = threading.Event()
        self.dimension_consumer_thread = threading.Thread(target=self.dimension_consumer, daemon=True)
        self.dimension_consumer_thread.start()

        self.bind("<Destroy>", self.on_destroy)

        parent.protocol("WM_DELETE_WINDOW", on_close)
        self.last_update = 0

        # Font Template
        fontSmall = ctk.CTkFont(family="Verdana", size=14, weight="bold")
        font = ctk.CTkFont(family="Verdana", size=24, weight="bold")
        fontBig = ctk.CTkFont(family="Verdana", size=48, weight="bold")

        # Check Connection Database and Lidar
        self.after(3000, lambda: connection_check.connection_check(self))

        container = ctk.CTkFrame(self, fg_color="white", height=100)
        container.pack(fill="both", expand=True, padx=20, pady=20)

        self.openWindow = ctk.CTkButton(
            container,
            text="Open 3D Lidar Viewer",
            font=fontSmall,
            fg_color="#F01382",
            command=self.toggle_lidar,
            height=40,
            hover=False,
            cursor="hand2"
        )
        self.openWindow.pack(fill="x")

        # Data
        valueFrame = ctk.CTkFrame(container, fg_color="white", height=100)
        valueFrame.pack(fill="both", expand=True, pady=(20, 0))

        valueFrame.columnconfigure((0, 1, 2), weight=1, uniform="a")
        valueFrame.rowconfigure(0, weight=1)
        valueFrame.grid_propagate(False)

        # Data Length
        self.lengthFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        # self.lengthFrame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.lengthFrame.pack(fill="x", pady=(0, 10))

        self.lengthLabel = ctk.CTkLabel(self.lengthFrame, text="Panjang", font=font, text_color="#F01382")
        self.lengthLabel.pack(pady=(50, 0))

        self.lengthValue = ctk.CTkLabel(self.lengthFrame, text="12", font=fontBig, text_color="#F01382")
        self.lengthValue.pack(pady=50)

        # Data Width
        self.widthFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        # self.widthFrame.grid(row=0, column=1, sticky="nsew", padx=10)
        self.widthFrame.pack(fill="x", pady=10)

        self.widthLabel = ctk.CTkLabel(self.widthFrame, text="Lebar", font=font, text_color="#F01382")
        self.widthLabel.pack(pady=(50, 0))

        self.widthValue = ctk.CTkLabel(self.widthFrame, text="2.5", font=fontBig, text_color="#F01382")
        self.widthValue.pack(pady=50)

        # Data Height
        self.heightFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        # self.heightFrame.grid(row=0, column=2, sticky="nsew", padx=(10, 0))
        self.heightFrame.pack(fill="x", pady=(10, 0))

        self.heightLabel = ctk.CTkLabel(self.heightFrame, text="Tinggi", font=font, text_color="#F01382")
        self.heightLabel.pack(pady=(50, 0))

        self.heightValue = ctk.CTkLabel(self.heightFrame, text="4", font=fontBig, text_color="#F01382")
        self.heightValue.pack(pady=50)

        self.sync_button_text()
        self.dimension_consumer_thread = threading.Thread(target=self.dimension_consumer, daemon=True)
        self.dimension_consumer_thread.start()
    
    def toggle_lidar(self):
        flag_window = "/tmp/lidar_visible"
        if os.path.exists(flag_window):
            os.remove(flag_window)
            self.openWindow.configure(text="Open 3D Lidar Viewer")
        else:
            open(flag_window, "a").close()
            self.openWindow.configure(text="Close 3D Lidar Viewer")
    
    def sync_button_text(self):
        flag_path = "/tmp/lidar_visible"
        if os.path.exists(flag_path):
            self.openWindow.configure(text="Hide 3D Lidar Viewer")
        else:
            self.openWindow.configure(text="Open 3D Lidar Viewer")
        self.after(300, self.sync_button_text)

    def on_destroy(self, event=None):
        print("[DEBUG] Home frame destroyed (page switched)")
        self.stop_consumer.set()

    def dimension_consumer(self):
        """Thread ini tunggu event, ambil latest shared value, lalu schedule update UI."""
        while not self.stop_consumer.is_set():
            DATA_EVENT.wait()
            if self.stop_consumer.is_set():
                break
            p = P_VAL.value
            l = L_VAL.value
            t = T_VAL.value
            print(f"[DEBUG] Dapat data langsung via shared mem: P={p:.2f}, L={l:.2f}, T={t:.2f}")
            DATA_EVENT.clear()
            if not self.winfo_exists():
                break
            self.after(0, self.update_dimension, p, l, t)

    def update_dimension(self, p, l, t):
        if not self.winfo_exists():
            print("[DEBUG] Frame sudah dihancurkan, abaikan update_dimension")
            return

        current_time = time.time()
        if current_time - self.last_update < 0.1:
            print("[DEBUG] Skipping UI update (debounce)")
            return

        try:
            self.lengthValue.configure(text=f"{p:.2f}")
            self.widthValue.configure(text=f"{l:.2f}")
            self.heightValue.configure(text=f"{t:.2f}")
            self.last_update = current_time
            print("[DEBUG] UI updated")
        except Exception as e:
            print(f"[DEBUG] Gagal update UI: {type(e).__name__} - {e}")