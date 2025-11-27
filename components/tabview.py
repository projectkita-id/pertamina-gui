import os
import rclpy
import threading
import customtkinter as ctk
import open3d.visualization.gui as gui

from PIL import Image
from multiprocessing import Value, Event
from components.db_config import init, get_conf, save_conf
from components.lidar_viewer import LivoxCalib    # Viewer utama 1 proses
import components.message_box as message
import components.calibrate_tab_content as right
import pages.home as home


# ======================================
# Shared memory (SAMA seperti Home.py)
# ======================================
P_VAL = Value('d', 0.0)
L_VAL = Value('d', 0.0)
T_VAL = Value('d', 0.0)
DATA_EVENT = Event()

VISIBLE_FLAG_PATH = "/tmp/lidar_visible"
CALIB_MODE_FLAG_PATH = "/tmp/lidar_calib_mode"

PROCESS = []   # Tetapi TABVIEW TIDAK PERNAH start viewer


# ======================================
# TABVIEW KODE
# ======================================
class Tabview(ctk.CTkFrame):
    def __init__(self, parent, show_page):
        super().__init__(parent, fg_color="white")
        self.show_page = show_page

        init()
        default = get_conf(["address", "username", "password", "port", "db_name"])
        address_val, username_val, password_val, port_val, dbname_val = (
            default if default else ("", "", "", "", "")
        )

        # ============================
        # Fonts (dari desain kode 1)
        # ============================
        font = ctk.CTkFont(family="Verdana", size=16, weight="bold")
        fontSmall = ctk.CTkFont(family="Verdana", size=14, weight="bold")

        # ============================
        # Tab Frame (header tab)
        # ============================
        tabFrame = ctk.CTkFrame(self, fg_color="transparent")
        tabFrame.pack(fill="x", side="top", pady=(10, 5))

        self.db_btn = ctk.CTkButton(
            tabFrame,
            text="Database",
            font=font,
            width=150,
            height=50,
            corner_radius=15,
            border_width=2,
            cursor="hand2",
            border_color="darkgrey",
            fg_color="#E0E0E0",
            hover_color="white",
            text_color="#F01382",
            command=lambda: self.show_tab("db")
        )
        self.db_btn.pack(side="left", fill="y", padx=10)

        self.calibrate_btn = ctk.CTkButton(
            tabFrame,
            text="Calibrate Lidar",
            font=font,
            width=150,
            height=50,
            corner_radius=15,
            border_width=2,
            cursor="hand2",
            border_color="darkgrey",
            fg_color="#E0E0E0",
            hover_color="white",
            text_color="#F01382",
            command=lambda: self.show_tab("calibrate")
        )
        self.calibrate_btn.pack(side="left", fill="y", padx=10)

        # Tombol back pakai handler back_to_home (LOGIKA KODE 2)
        close = Image.open('assets/icon/close.png')
        close = ctk.CTkImage(light_image=close, size=(16, 16))
        back = ctk.CTkButton(
            tabFrame,
            image=close,
            text="",
            width=40,
            height=40,
            font=font,
            fg_color="transparent",
            hover=False,
            text_color="#F01382",
            cursor="hand2",
            command=lambda: self.back_to_home(show_page)
        )
        back.pack(side="right", padx=10)

        # ============================
        # Database Tab (desain kode 1)
        # ============================
        self.db_tab = ctk.CTkFrame(
            self,
            fg_color="white",
            corner_radius=20,
            border_width=2,
            border_color="darkgrey"
        )

        ctk.CTkLabel(self.db_tab, text="Address", font=font, text_color="#F01382").pack(pady=(80, 5))
        self.addr = ctk.CTkEntry(
            self.db_tab,
            width=400,
            height=40,
            font=font,
            text_color="#3b3b3b",
            placeholder_text="192.168.1.1",
            placeholder_text_color="darkgrey",
            justify="center"
        )
        self.addr.pack(pady=5)

        ctk.CTkLabel(self.db_tab, text="Username", font=font, text_color="#F01382").pack(pady=5)
        self.usn = ctk.CTkEntry(
            self.db_tab,
            width=400,
            height=40,
            font=font,
            text_color="#3b3b3b",
            placeholder_text="root",
            placeholder_text_color="darkgrey",
            justify="center"
        )
        self.usn.pack(pady=5)

        ctk.CTkLabel(self.db_tab, text="Password", font=font, text_color="#F01382").pack(pady=5)
        self.passwd = ctk.CTkEntry(
            self.db_tab,
            width=400,
            height=40,
            font=font,
            text_color="#3b3b3b",
            placeholder_text="*****",
            placeholder_text_color="darkgrey",
            justify="center"
        )
        self.passwd.pack(pady=5)

        ctk.CTkLabel(self.db_tab, text="Port", font=font, text_color="#F01382").pack(pady=5)
        self.port = ctk.CTkEntry(
            self.db_tab,
            width=400,
            height=40,
            font=font,
            text_color="#3b3b3b",
            placeholder_text="3306",
            placeholder_text_color="darkgrey",
            justify="center"
        )
        self.port.pack(pady=5)

        ctk.CTkLabel(self.db_tab, text="Database Name", font=font, text_color="#F01382").pack(pady=5)
        self.db_name = ctk.CTkEntry(
            self.db_tab,
            width=400,
            height=40,
            font=font,
            text_color="#3b3b3b",
            placeholder_text="LidarDB",
            placeholder_text_color="darkgrey",
            justify="center"
        )
        self.db_name.pack(pady=(5, 0))

        # Set data ke fields (LOGIKA KODE 2, tetap)
        entries = [
            (self.addr, address_val),
            (self.usn, username_val),
            (self.passwd, password_val),
            (self.port, port_val),
            (self.db_name, dbname_val)
        ]

        for entry, value in entries:
            if value:
                entry.insert(0, value)

        btn_area = ctk.CTkFrame(self.db_tab, height=40, width=350, fg_color="white")
        btn_area.pack(padx=20, expand=True)

        ctk.CTkButton(
            btn_area,
            text="Cancel",
            font=font,
            width=200,
            height=50,
            corner_radius=20,
            fg_color="red",
            hover_color="#F01382",
            cursor="hand2",
            command=lambda: self.back_to_home(show_page)
        ).pack(fill="x", pady=10)

        ctk.CTkButton(
            btn_area,
            text="Save",
            font=font,
            width=200,
            height=50,
            corner_radius=20,
            fg_color="#4CAF50",
            hover_color="#45a049",
            cursor="hand2",
            command=self.save_data
        ).pack(fill="x", pady=10)

        # ============================
        # Calibrate Tab (desain kode 1,
        # LOGIKA tetap: toggle_viewer_window + CALIB_MODE flag)
        # ============================
        self.calibrate_tab = ctk.CTkFrame(
            self,
            fg_color="white",
            corner_radius=20,
            border_width=2,
            border_color="darkgrey"
        )

        # Space container seperti di kode 1
        space = ctk.CTkFrame(self.calibrate_tab, fg_color="transparent", corner_radius=20)
        space.pack(fill="x", expand=True, padx=10, pady=10)

        # Tombol untuk toggle viewer (LOGIKA KODE 2, desain kode 1)
        self.openWindow = ctk.CTkButton(
            space,
            text="Open3D LiDAR Viewer",   # tampilan dari kode 1
            fg_color="#F01382",
            font=fontSmall,
            hover=False,
            cursor="hand2",
            command=self.toggle_viewer_window  # LOGIKA TETAP KODE 2
        )
        self.openWindow.pack()

        # Panel kanan calibrate (tetap pakai CalibrateTabContent dari KODE 2)
        right.CalibrateTabContent(space, show_page=show_page).pack(fill="both", expand=True)

        # Default open DB tab
        self.show_tab("db")

    # ============================================================
    # TAB SWITCH (LOGIKA KODE 2, dipertahankan)
    # ============================================================
    def show_tab(self, name):
        """Switch antara DB tab dan Calibrate tab + aktifkan mode viewer."""
        for tab in [self.db_tab, self.calibrate_tab]:
            tab.pack_forget()

        for btn in [self.db_btn, self.calibrate_btn]:
            btn.configure(fg_color="#E0E0E0")

        if name == "db":
            self.db_tab.pack(fill="both", expand=True, pady=10)
            self.db_btn.configure(fg_color="white", text_color="#F01382")

            # HOME MODE → disable panel viewer
            if os.path.exists(CALIB_MODE_FLAG_PATH):
                os.remove(CALIB_MODE_FLAG_PATH)

        elif name == "calibrate":
            self.calibrate_tab.pack(fill="both", expand=True, pady=10)
            self.calibrate_btn.configure(fg_color="white", text_color="#F01382")

            # CALIB MODE → aktifkan panel viewer
            open(CALIB_MODE_FLAG_PATH, "a").close()

    def back_to_home(self, show_page):
        """Saat klik tombol X/back"""
        if os.path.exists(CALIB_MODE_FLAG_PATH):
            os.remove(CALIB_MODE_FLAG_PATH)

        show_page(home.Home)

    # ============================================================
    # VIEWER WINDOW VISIBILITY CONTROL (LOGIKA KODE 2)
    # ============================================================
    def toggle_viewer_window(self):
        """Toggle viewer window via /tmp/lidar_visible"""
        if os.path.exists(VISIBLE_FLAG_PATH):
            os.remove(VISIBLE_FLAG_PATH)
        else:
            open(VISIBLE_FLAG_PATH, "a").close()

    # ============================================================
    # SAVE DB CONFIG (LOGIKA KODE 2)
    # ============================================================
    def save_data(self):
        try:
            address = self.addr.get()
            username = self.usn.get()
            password = self.passwd.get()
            port = self.port.get()
            dbname = self.db_name.get()

            if not all([address, username, password, port, dbname]):
                message.MessageBox(self, "Error", "All fields are required!", type="error")
                return

            save_conf(address, username, password, port, dbname)
            message.MessageBox(self, "Success", "Database configuration saved successfully!", type="info")

        except Exception as e:
            message.MessageBox(self, "Error", f"Failed to save configuration: {str(e)}", type="error")
