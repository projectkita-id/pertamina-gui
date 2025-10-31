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

from multiprocessing import Process, Queue
from components.lidar_viewer import LivoxGUI

DIMENSION_QUEUE = Queue(maxsize=1000)
PROCESS = []

def kill():
    print("Killing processes...")
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

def run_open3d_viewer(dim_queue):
    rclpy.init()
    app = gui.Application.instance
    app.initialize()

    node = LivoxGUI(app, dim_queue=dim_queue)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    app.run()
    node.save_roi_to_file()
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

def open3d_thread():
    p = Process(target=run_open3d_viewer, args=(DIMENSION_QUEUE,))
    PROCESS.append(p)
    p.start()
    return p

class Home(ctk.CTkFrame) :
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="white")
        header.Header(self, show_page).pack(fill="x")

        open3d_thread()

        def on_close():
            print("Closing application...")
            kill()
            if os.path.exists("/tmp/lidar_visible"):
                os.remove("/tmp/lidar_visible")
            parent.destroy()

        parent.protocol("WM_DELETE_WINDOW", on_close)

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
        self.update_dimension()
    
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

    def update_dimension(self) :
        try :
            while True :
                p, l ,t = DIMENSION_QUEUE.get_nowait()
                print(f"[DEBUG] Dapat data: P={p:.2f}, L={l:.2f}, T={t:.2f}")
                self.lengthValue.configure(text=f"{p:.2f}")
                self.widthValue.configure(text=f"{l:.2f}")
                self.heightValue.configure(text=f"{t:.2f}")
        except queue.Empty :
            print("[DEBUG] Queue kosong")
            pass
        self.after(100, self.update_dimension)