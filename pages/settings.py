from PIL import Image
import pages.home as home
import customtkinter as ctk
import components.tabview as tabview

class Settings(ctk.CTkFrame) :
    def __init__(self, parent, show_page) :
        super().__init__(parent, fg_color="white")

        # Font Template
        font = ctk.CTkFont(family="Verdana", size=20, weight="bold")

        tabs = tabview.Tabview(self, show_page=show_page)
        tabs.pack(fill="both", expand=True, padx=20, pady=10)