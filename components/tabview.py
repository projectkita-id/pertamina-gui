from PIL import Image
from sympy import content
import pages.home as home
import customtkinter as ctk
import components.message_box as message
import components.calibrate_tab_content as right
from components.db_config import init, get_conf, save_conf

class Tabview(ctk.CTkFrame) :
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="white")
        self.show_page = show_page

        init()
        default = get_conf(["address", "username", "password", "port", "db_name"])
        address_val, username_val, password_val, port_val, dbname_val = default if default else ("", "", "", "", "")

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=16, weight="bold")
        fontBig = ctk.CTkFont(family="Verdana", size=24, weight="bold")

        # Tab Frame
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
            command=lambda : self.show_tab("db")
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
            command=lambda : self.show_tab("calibrate")
        )
        self.calibrate_btn.pack(side="left", fill="y", padx=10)

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
            command=lambda : show_page(home.Home)
        )
        back.pack(side="right", padx=10)

        # Database Tab Content
        self.db_tab = ctk.CTkFrame(self, fg_color="white", corner_radius=20, border_width=2, border_color="darkgrey")
        
        ctk.CTkLabel(self.db_tab, text="Address", font=font, text_color="#F01382").pack(pady=(50, 5))
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
        self.db_name.pack(pady=5)

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
            entry.pack(pady=5)

        btn_area = ctk.CTkFrame(self.db_tab, height=40, width=350, fg_color="white")
        btn_area.pack(pady=(0, 20), anchor="center", expand=True)

        cancel_btn = ctk.CTkButton(
            btn_area,
            text="Cancel",
            font=font,
            width=150,
            height=40,
            corner_radius=20,
            fg_color="red",
            hover_color="#ff4d4d",
            cursor="hand2",
            command=lambda : show_page(home.Home)
        ).pack(side="left", padx=(0, 10))

        save_btn = ctk.CTkButton(
            btn_area,
            text="Save",
            font=font,
            width=150,
            height=40,
            corner_radius=20,
            fg_color="#4CAF50",
            hover_color="#45a049",
            cursor="hand2",
            command=self.save_data
        ).pack(side="right", padx=(10, 0))

        # Lidar Tab Content
        self.calibrate_tab = ctk.CTkFrame(self, fg_color="white", corner_radius=20, border_width=2, border_color="darkgrey")
        self.calibrate_tab.grid_columnconfigure(0, weight=1000)
        self.calibrate_tab.grid_columnconfigure(1, weight=0)
        self.calibrate_tab.grid_columnconfigure(2, weight=1)
        self.calibrate_tab.grid_rowconfigure(0, weight=1)

        left_space = ctk.CTkFrame(self.calibrate_tab, fg_color="transparent")
        left_space.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        separator = ctk.CTkFrame(self.calibrate_tab, width=2, fg_color="darkgrey")
        separator.grid(row=0, column=1, sticky="ns")

        right_space = ctk.CTkFrame(self.calibrate_tab, fg_color="transparent", corner_radius=20)
        right_space.grid(row=0, column=2, sticky="nsew", padx=10, pady=10)

        right.CalibrateTabContent(right_space, show_page=show_page).pack(fill="both", expand=True)

        self.show_tab("db")

    def show_tab(self, name) :
        for tab in [self.db_tab, self.calibrate_tab] :
            tab.pack_forget()
        
        for btn in [self.db_btn, self.calibrate_btn] :
            btn.configure(fg_color="#E0E0E0")
        
        if name == "db" :
            self.db_tab.pack(fill="both", expand=True, pady=10)
            self.db_btn.configure(fg_color="white", text_color="#F01382")
        elif name == "calibrate" :
            self.calibrate_tab.pack(fill="both", expand=True, pady=10)
            self.calibrate_btn.configure(fg_color="white", text_color="#F01382")

    def save_data(self) :
        try :
            address = self.addr.get()
            username = self.usn.get()
            password = self.passwd.get()
            port = self.port.get()
            dbname = self.db_name.get()

            if not all([address, username, password, port, dbname]) :
                message.MessageBox(self, "Error", "All fields are required!", type="error")
                return

            save_conf(
                address,
                username,
                password,
                port,
                dbname
            )

            message.MessageBox(self, "Success", "Database configuration saved successfully!", type="info")
        except Exception as e :
            message.MessageBox(self, "Error", f"Failed to save configuration: {str(e)}", type="error")
