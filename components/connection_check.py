import socket
import subprocess
import mysql.connector
import threading
import time
import components.message_box as message
from components.db_config import get_conf


def lidar_check(ip="10.12.32.10", port=56300, timeout=2):
    try:
        result = subprocess.run(
            ["ping","-c","1","-W",str(timeout),ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if result.returncode != 0:
            return False, f"LIDAR not reachable at {ip}"
    except:
        return False, "Ping error"

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        s.sendto(b'\x00', (ip, port))
        s.close()
        return True, "LIDAR OK"
    except Exception as e:
        return False, f"LIDAR UDP failed: {e}"


def connection_check(self):
    """Run DB + LIDAR check in background thread (non-blocking)."""

    if hasattr(self, "active_messagebox") and self.active_messagebox is not None:
        return

    def worker():
        host, user, password, port, db_name = get_conf(["address","username","password","port","db_name"])

        if not any([host,user,password,port,db_name]):
            try:
                self.after(3000, lambda: connection_check(self))
            except:
                pass
            return

        # DB check
        try:
            conn = mysql.connector.connect(
                host=host, user=user, password=password,
                port=port, database=db_name, connection_timeout=3
            )
            conn.close()
            db_ok = True
            db_msg = "Database OK"
        except Exception as e:
            db_ok = False
            db_msg = str(e)

        # Lidar check
        lidar_ok, lidar_msg = lidar_check()

        def update_ui():
            if not self.winfo_exists():
                return

            def next():
                self.after(3000, lambda: connection_check(self))

            if not db_ok:
                self.active_messagebox = message.MessageBox(self, "DB Error", db_msg, type="error")
                self.active_messagebox.ok_callback = lambda: (_clear_msg(self), next())
            elif not lidar_ok:
                self.active_messagebox = message.MessageBox(self, "LIDAR Error", lidar_msg, type="error")
                self.active_messagebox.ok_callback = lambda: (_clear_msg(self), next())
            else:
                next()

        try:
            self.after(0, update_ui)
        except:
            pass

    threading.Thread(target=worker, daemon=True).start()


def _clear_msg(self):
    if hasattr(self, "active_messagebox") and self.active_messagebox is not None:
        try:
            self.active_messagebox.destroy()
        except:
            pass
        self.active_messagebox = None
