from PIL import ImageTk
from components.db_config  import init

import customtkinter as ctk
import pages.home as home

if __name__ == "__main__":
    # Style
    style = ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("dark-blue")

    # Main Window
    root = ctk.CTk()
    root.title("Lidar Data Viewer")
    root.geometry("1280x720")
    root.minsize(800, 600)

    # App Icons
    appIcon = ImageTk.PhotoImage(file="assets/icon/logo.ico")
    root.tk.call('wm', 'iconphoto', root._w, appIcon)

    init()
    current_page = None

    def show_page(page) :
        global current_page
        if current_page is not None :
            current_page.pack_forget()
            current_page.destroy()
        new = page(root, show_page)
        new.pack(fill="both", expand=True)
        globals()["current_page"] = new

    show_page(home.Home)

root.mainloop()