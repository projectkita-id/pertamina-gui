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

        # "Move" Tab Contents
        self.move_tab = ctk.CTkFrame(self, fg_color="white", corner_radius=20)
        self.move_tab.pack(fill="x", expand=True)

        ctk.CTkLabel(self.move_tab, text="Maju / Mundur", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveX = input.NumberInput(self.move_tab, key_up="Q", key_down="E")
        self.moveX.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Kiri / Kanan", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveY = input.NumberInput(self.move_tab, key_up="A", key_down="D")
        self.moveY.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Atas / Bawah", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.moveZ = input.NumberInput(self.move_tab, key_up="W", key_down="S")
        self.moveZ.pack(pady=(0, 20))

        

        ctk.CTkLabel(self.move_tab, text="Pitch", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.pitch = input.NumberInput(self.move_tab, key_up="U", key_down="Y")
        self.pitch.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Kiri / Kanan", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.roll = input.NumberInput(self.move_tab, key_up="O", key_down="P")
        self.roll.pack(pady=(0, 20))

        ctk.CTkLabel(self.move_tab, text="Atas / Bawah", font=fontNormal, text_color="#F01382").pack(pady=(20, 10))
        self.yaw = input.NumberInput(self.move_tab, key_up="K", key_down="L")
        self.yaw.pack(pady=(0, 20))

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

    def _on_any_key_press(self, event):
        from components.number_input import KEY_BIND_MAP

        key = event.keysym.upper()
        print("Key pressed:", key)

        if self.move_tab.winfo_ismapped():
            allowed_keys = ["Q", "E", "A", "D", "W", "S"]
        elif self.move_tab.winfo_ismapped():
            allowed_keys = ["U", "Y", "O", "P", "K", "L"]
        else:
            allowed_keys = []

        if key in allowed_keys and key in KEY_BIND_MAP:
            instance, action = KEY_BIND_MAP[key]
            instance._trigger_key(key, action)
