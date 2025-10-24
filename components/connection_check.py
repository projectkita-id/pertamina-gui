import socket
import subprocess
import mysql.connector
import components.message_box as message
from components.db_config import get_conf

def lidar_check(ip="192.168.1.190", port=56300, timeout=2):
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(timeout), ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        if result.returncode != 0:
            return False, f"LIDAR is not reachable at {ip}."
    except Exception as e:
        return False, f"Ping error: {str(e)}"
    
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        s.sendto(b'\x00', (ip, port))
        s.close()
        return True, "LIDAR is reachable and responding."
    except Exception as e:
        return False, f"LIDAR UDP check failed: {str(e)}"

def connection_check(self):
    if hasattr(self, "active_messagebox") and self.active_messagebox is not None:
        return

    host, user, password, port, db_name = get_conf(["address", "username", "password", "port", "db_name"])

    if not any([host, user, password, port, db_name]):
        self.after(3000, lambda: connection_check(self))
        return

    try:
        conn = mysql.connector.connect(
            host=host,
            user=user,
            password=password,
            port=port,
            database=db_name,
            connection_timeout=3
        )
        conn.close()
        db_ok = True
        db_msg = "Connection to database has been established successfully."
    except Exception as e:
        db_ok = False
        db_msg = f"Database connection failed: {str(e)}"

    lidar_ok, lidar_msg = lidar_check()

    def on_close():
        if hasattr(self, "active_messagebox") and self.active_messagebox is not None:
            self.active_messagebox.destroy()
            self.active_messagebox = None
        self.after(3000, lambda: connection_check(self))

    if not db_ok:
        self.active_messagebox = message.MessageBox(self, "Database Error", db_msg, type="error")
        self.active_messagebox.protocol("WM_DELETE_WINDOW", on_close)
        self.active_messagebox.ok_callback = on_close
    elif not lidar_ok:
        self.active_messagebox = message.MessageBox(self, "LIDAR Error", lidar_msg, type="error")
        self.active_messagebox.protocol("WM_DELETE_WINDOW", on_close)
        self.active_messagebox.ok_callback = on_close
    else:
        self.after(3000, lambda: connection_check(self))