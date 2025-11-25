#!/usr/bin/env python3
import customtkinter as ctk
from PIL import ImageTk
from components.db_config import init
import pages.home as home
import sys   # <--- Tambahkan ini

if __name__ == "__main__":
    ctk.set_appearance_mode("light")
    ctk.set_default_color_theme("dark-blue")

    root = ctk.CTk()
    root.title("Lidar Data Viewer")
    root.geometry("500x900")
    root.resizable(False, False)

    try:
        appIcon = ImageTk.PhotoImage(file="assets/icon/logo.ico")
        root.tk.call('wm', 'iconphoto', root._w, appIcon)
    except:
        print("[WARN] Logo icon tidak ditemukan")

    init()

    current_page = [None]

    def show_page(page):
        if current_page[0] is not None:
            current_page[0].pack_forget()
            current_page[0].destroy()

        new_page = page(root, show_page)
        new_page.pack(fill="both", expand=True)
        current_page[0] = new_page

    show_page(home.Home)

    # ---------------- AUTO-CLOSE TIMER ----------------
    AUTO_EXIT_SECONDS = 30 * 60  

    def auto_close():
        print(f"[GUI] Auto-close setelah {AUTO_EXIT_SECONDS} detik, tutup GUI + viewer...")
        cb = getattr(root, "_on_close_callback", None)
        if cb is not None:
            cb()  # kill_viewer() + destroy()
        else:
            root.destroy()

        sys.exit(5)  # <--- Auto-close exit code (untuk restart)
    # --------------------------------------------------

    root.after(AUTO_EXIT_SECONDS * 1000, auto_close)
    root.mainloop()
