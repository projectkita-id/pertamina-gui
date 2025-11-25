import customtkinter as ctk

class MessageBox(ctk.CTkToplevel):
    def __init__(self, parent, title, message, type="info"):
        super().__init__(parent)
        self.title(title)
        self.resizable(False, False)

        font = ctk.CTkFont(family="Verdana", size=14, weight="bold")

        width = 300
        height = 180

        parent.update_idletasks()
        parent_x = parent.winfo_rootx()
        parent_y = parent.winfo_rooty()
        parent_width = parent.winfo_width()
        parent_height = parent.winfo_height()

        x = parent_x + (parent_width // 2) - (width // 2)
        y = parent_y + (parent_height // 2) - (height // 2)

        self.geometry(f"{width}x{height}+{x}+{y}")

        if type == "error":
            text_color = "#FF0000"
        else:
            text_color = "#41b831"

        ctk.CTkLabel(self, text=message, wraplength=250, font=font, text_color=text_color).pack(pady=30)
        ctk.CTkButton(self, text="OK", width=100, font=font, fg_color="#2913F0", hover=False, cursor="hand2", command=lambda: self.ok_callback() if hasattr(self, "ok_callback") else self.destroy()).pack(pady=2)

        self.grab_set()
        self.focus_force()