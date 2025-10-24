from PIL import Image
import customtkinter as ctk

KEY_BIND_MAP = {}

class NumberInput(ctk.CTkFrame):
    def __init__(self, parent, min_value=-100, max_value=100, step=1, default=0, key_up=None, key_down=None, **kwargs):
        super().__init__(parent, fg_color="transparent", **kwargs)

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=14, weight="bold")

        self.value = ctk.IntVar(value=default)
        self.min_value = min_value
        self.max_value = max_value
        self.step = step
        self.key_up = key_up
        self.key_down = key_down

        self.entry = ctk.CTkEntry(
            self, textvariable=self.value, width=80, height=45,
            justify="center", font=font, state="readonly", takefocus=0
        )
        self.entry.pack(side="left", padx=(0, 10))
        self.entry.bind("<KeyRelease>", self.validate_number)

        self.button_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.button_frame.pack(side="left", fill="y")

        up_icon = Image.open('assets/icon/arrow-up.png')
        up_icon = ctk.CTkImage(light_image=up_icon, size=(12, 12))
        self.plus_btn = ctk.CTkButton(
            self.button_frame, text="", image=up_icon, width=25, height=18, cursor="hand2",
            command=lambda: self._trigger_key(self.key_up, self.increment),
            fg_color="#E0E0E0", hover_color="#CFCFCF", corner_radius=4
        )
        self.plus_btn.pack(pady=(0, 3))

        down_icon = Image.open('assets/icon/arrow-down.png')
        down_icon = ctk.CTkImage(light_image=down_icon, size=(12, 12))
        self.minus_btn = ctk.CTkButton(
            self.button_frame, text="", image=down_icon, width=25, height=18, cursor="hand2",
            command=lambda: self._trigger_key(self.key_down, self.decrement),
            fg_color="#E0E0E0", hover_color="#CFCFCF", corner_radius=4
        )
        self.minus_btn.pack(pady=(3, 0))
 
        if key_up:
            KEY_BIND_MAP[key_up.upper()] = (self, self.increment)
        if key_down:
            KEY_BIND_MAP[key_down.upper()] = (self, self.decrement)
    
    def _on_any_key_press(self, event):
        key = event.keysym.upper()
        print("Key pressed:", key)

        if self.key_up and key == self.key_up.upper():
            self._trigger_key(self.key_up, self.increment)
        elif self.key_down and key == self.key_down.upper():
            self._trigger_key(self.key_down, self.decrement)

    def _trigger_key(self, key, action):
        print(f"Tombol '{key.upper()}' ditekan")
        action()

    def validate_number(self, event=None):
        try:
            val = int(self.entry.get())
            if val < self.min_value:
                self.value.set(self.min_value)
            elif val > self.max_value:
                self.value.set(self.max_value)
        except ValueError:
            self.value.set(self.min_value)

    def increment(self):
        val = self.value.get() + self.step
        if val <= self.max_value:
            self.value.set(val)

    def decrement(self):
        val = self.value.get() - self.step
        if val >= self.min_value:
            self.value.set(val)

    def get(self):
        return self.value.get()
