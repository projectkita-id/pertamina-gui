from PIL import Image
import customtkinter as ctk
import pages.settings as settings

class Header(ctk.CTkFrame):
    def __init__(self, parent, show_page):
        super().__init__(parent)
        self.configure(fg_color="white")
        self.pack(fill="x")

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=18, weight="bold")

        # Header Frame
        head = ctk.CTkFrame(self, fg_color="white")
        head.pack(fill="x", padx=20)

        # Header Content
        header = ctk.CTkLabel(head, text="UPPKB PADANG ULAK TANDING", font=font, fg_color="white", text_color="#F01382")
        header.pack(side="left", pady=(20, 0))
        
        setting = Image.open('assets/icon/settings.png')
        setting = ctk.CTkImage(light_image=setting, size=(24, 24))
        setting_button = ctk.CTkButton(head, command=lambda: show_page(settings.Settings), image=setting, text="", width=40, height=40, fg_color="transparent", hover=False, cursor="hand2")
        setting_button.pack(side="right", pady=(20, 0))