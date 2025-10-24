import pages.home as home
import customtkinter as ctk
import components.number_input as input
import components.message_box as message
from components.db_config import init, get_calibration, save_calibration_move, save_calibration_rotate

class CalibrateTabContent(ctk.CTkFrame):
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="transparent", corner_radius=20)
        self.show_page = show_page
        
        init()
        default = get_calibration(["moveX", "moveY", "moveZ", "pitch", "roll", "yaw"])
        moveX_val, moveY_val, moveZ_val, pitch_val, roll_val, yaw_val = default if default else ("0", "0", "0", "0", "0", "0")

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=16, weight="bold")
        fontNormal = ctk.CTkFont(family="Verdana", size=20, weight="bold")

        # Tab Frame
        tabFrame = ctk.CTkFrame(self, fg_color="transparent", corner_radius=20)
        tabFrame.pack(fill="x", side="top", padx=15, pady=(2, 5))

        self.move_btn = ctk.CTkButton(
            tabFrame,
            text="Pindah", 
            font=font,
            width=100,
            height=50,
            corner_radius=15,
            border_width=2,
            cursor="hand2",
            border_color="darkgrey",
            fg_color="#E0E0E0",
            hover_color="white",
            text_color="#F01382",
            command=lambda : self.show_tab("move")
        )
        self.move_btn.pack(side="left", fill="y", padx=10)

        self.rotate_btn = ctk.CTkButton(
            tabFrame,
            text="Rotasi",
            font=font,
            width=100,
            height=50,
            corner_radius=15,
            border_width=2,
            cursor="hand2",
            border_color="darkgrey",
            fg_color="#E0E0E0",
            hover_color="white",
            text_color="#F01382",
            command=lambda : self.show_tab("rotate")
        )
        self.rotate_btn.pack(side="left", fill="y", padx=10)

        separator = ctk.CTkFrame(self, height=2, fg_color="darkgrey")
        separator.pack(fill="x", pady=(5, 10))

        # "Move" Tab Contents
        self.move_tab = ctk.CTkFrame(self, fg_color="white", corner_radius=20)

        ctk.CTkLabel(self.move_tab, text="Maju / Mundur", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveX = input.NumberInput(self.move_tab, key_up="Q", key_down="E")
        self.moveX.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Kiri / Kanan", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveY = input.NumberInput(self.move_tab, key_up="A", key_down="D")
        self.moveY.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Atas / Bawah", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveZ = input.NumberInput(self.move_tab, key_up="W", key_down="S")
        self.moveZ.pack(pady=(0, 20))

        btn_area = ctk.CTkFrame(self.move_tab, height=40, width=350, fg_color="white")
        btn_area.pack(pady=(0, 20), anchor="center", expand=True)

        cancel_btn = ctk.CTkButton(
            btn_area,
            text="Cancel",
            font=font,
            width=100,
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
            width=100,
            height=40,
            corner_radius=20,
            fg_color="#4CAF50",
            hover_color="#45a049",
            cursor="hand2",
            command=self.save_data_move
        ).pack(side="right", padx=(10, 0))

        # "Rotate" Tab Contents
        self.rotate_tab = ctk.CTkFrame(self, fg_color="white", corner_radius=20)

        ctk.CTkLabel(self.rotate_tab, text="Pitch", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.pitch = input.NumberInput(self.rotate_tab, key_up="U", key_down="Y")
        self.pitch.pack(pady=(0, 20))

        ctk.CTkLabel(self.rotate_tab, text="Kiri / Kanan", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.roll = input.NumberInput(self.rotate_tab, key_up="O", key_down="P")
        self.roll.pack(pady=(0, 20))

        ctk.CTkLabel(self.rotate_tab, text="Atas / Bawah", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.yaw = input.NumberInput(self.rotate_tab, key_up="K", key_down="L")
        self.yaw.pack(pady=(0, 20))

        btn_area = ctk.CTkFrame(self.rotate_tab, height=40, width=350, fg_color="white")
        btn_area.pack(pady=(0, 20), anchor="center", expand=True)

        cancel_btn = ctk.CTkButton(
            btn_area,
            text="Cancel",
            font=font,
            width=100,
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
            width=100,
            height=40,
            corner_radius=20,
            fg_color="#4CAF50",
            hover_color="#45a049",
            cursor="hand2",
            command=self.save_data_rotate
        ).pack(side="right", padx=(10, 0))

        entries = [
            (self.moveX, moveX_val),
            (self.moveY, moveY_val),
            (self.moveZ, moveZ_val),
            (self.pitch, pitch_val),
            (self.roll, roll_val),
            (self.yaw, yaw_val)
        ]

        for entry, value in entries :
            if value :
                entry.value.set(value)
            entry.pack(pady=5)

        root = self.winfo_toplevel()
        root.bind_all("<KeyPress>", self._on_any_key_press)

        self.show_tab("move")

    def save_data_move(self):
        try:
            moveX = self.moveX.get()
            moveY = self.moveY.get()
            moveZ = self.moveZ.get()

            save_calibration_move(
                moveX,
                moveY,
                moveZ
            )

            message.MessageBox(self, "Success", "Database configuration saved successfully!", type="info")
        except Exception as e:
            message.MessageBox(self, "Error", f"Failed to save calibration data:\n{str(e)}", type="error")

    def save_data_rotate(self):
        try:
            pitch = self.pitch.get()
            roll = self.roll.get()
            yaw = self.yaw.get()

            save_calibration_rotate(
                pitch, 
                roll, 
                yaw
            )

            message.MessageBox(self, "Success", "Database configuration saved successfully!", type="info")
        except Exception as e:
            message.MessageBox(self, "Error", f"Failed to save calibration data:\n{str(e)}", type="error")

    def _on_any_key_press(self, event):
        from components.number_input import KEY_BIND_MAP

        key = event.keysym.upper()
        print("Key pressed:", key)

        if self.move_tab.winfo_ismapped():
            allowed_keys = ["Q", "E", "A", "D", "W", "S"]
        elif self.rotate_tab.winfo_ismapped():
            allowed_keys = ["U", "Y", "O", "P", "K", "L"]
        else:
            allowed_keys = []

        if key in allowed_keys and key in KEY_BIND_MAP:
            instance, action = KEY_BIND_MAP[key]
            instance._trigger_key(key, action)

    def show_tab(self, name) :
        for tab in [self.move_tab, self.rotate_tab] :
            tab.pack_forget()
        
        for btn in [self.move_btn, self.rotate_btn] :
            btn.configure(fg_color="#E0E0E0")
        
        if name == "move" :
            self.move_tab.pack(fill="both", expand=True)
            self.move_btn.configure(fg_color="white", text_color="#F01382")
        elif name == "rotate" :
            self.rotate_tab.pack(fill="both", expand=True)
            self.rotate_btn.configure(fg_color="white", text_color="#F01382")