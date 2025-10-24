import queue
import rclpy
import threading
import customtkinter as ctk
import components.header as header
import components.lidar_viewer as lidar
import components.connection_check as connection_check
from components.lidar_viewer import LivoxOpen3DViewer

DIMENSION_QUEUE = queue.Queue()

def run_open3d_viewer():
        try:
            rclpy.init()
            viewer = LivoxOpen3DViewer()
            viewer.get_logger().info("Starting LivoxOpen3DViewer")
            rclpy.spin(viewer)
        except Exception as e:
            print(f"Error in run_open3d_viewer: {str(e)}")
        finally:
            print("Shutting down LivoxOpen3DViewer")
            rclpy.shutdown()

def open3d_thread():
    viewer_thread = threading.Thread(target=run_open3d_viewer, daemon=True)
    viewer_thread.start()

class Home(ctk.CTkFrame) :
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="white")
        header.Header(self, show_page).pack(fill="x")

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=24, weight="bold")
        fontBig = ctk.CTkFont(family="Verdana", size=48, weight="bold")

        container = ctk.CTkFrame(self, fg_color="white", height=100)
        container.pack(fill="both", expand=True, padx=20, pady=20)

        # Lidar
        lidarFrame = ctk.CTkFrame(container, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20, height=300)
        lidarFrame.pack(fill="both", expand=True)

        openWindow = ctk.CTkButton(
            lidarFrame,
            text="Open 3D Lidar Viewer",
            fg_color="#F01382",
            command=open3d_thread
        )
        openWindow.pack()


        # Data
        valueFrame = ctk.CTkFrame(container, fg_color="white", height=100)
        valueFrame.pack(fill="both", expand=True, pady=(20, 0))

        valueFrame.columnconfigure((0, 1, 2), weight=1, uniform="a")
        valueFrame.rowconfigure(0, weight=1)
        valueFrame.grid_propagate(False)

        # Data Length
        lengthFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        lengthFrame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        lengthLabel = ctk.CTkLabel(lengthFrame, text="Panjang", font=font, text_color="#F01382")
        lengthLabel.pack(pady=(20, 0))

        lengthValue = ctk.CTkLabel(lengthFrame, text="12", font=fontBig, text_color="#F01382")
        lengthValue.pack(pady=(35, 0))

        # Data Width
        widthFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        widthFrame.grid(row=0, column=1, sticky="nsew", padx=10)

        widthLabel = ctk.CTkLabel(widthFrame, text="Lebar", font=font, text_color="#F01382")
        widthLabel.pack(pady=(20, 0))

        widthValue = ctk.CTkLabel(widthFrame, text="2.5", font=fontBig, text_color="#F01382")
        widthValue.pack(pady=(35, 0))

        # Data Height
        heightFrame = ctk.CTkFrame(valueFrame, fg_color="white", border_width=2, border_color="darkgrey", corner_radius=20)
        heightFrame.grid(row=0, column=2, sticky="nsew", padx=(10, 0))

        heightLabel = ctk.CTkLabel(heightFrame, text="Tinggi", font=font, text_color="#F01382")
        heightLabel.pack(pady=(20, 0))

        heightValue = ctk.CTkLabel(heightFrame, text="4", font=fontBig, text_color="#F01382")
        heightValue.pack(pady=(35, 0))

        
        # Check Connection Database and Lidar
        self.after(3000, lambda: connection_check.connection_check(self))
    
    def update_dimension(self) :
        try :
            while True :
                p, l ,t = DIMENSION_QUEUE.get_nowait()
                self.lengthValue.configure(f"{p:.3f}")
                self.widthValue.configure(f"{l:.3f}")
                self.heightValue.configure(f"{t:.3f}")
        except queue.Empty :
            pass
        self.after(100, self.update_dimension)