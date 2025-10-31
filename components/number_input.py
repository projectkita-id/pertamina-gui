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
            self, textvariable=self.value, height=45,
            justify="center", font=font, state="readonly", takefocus=0
        )
        self.entry.pack(fill="x", padx=(0, 10))
        self.entry.bind("<KeyRelease>", self.validate_number)
 
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
