import os
import sqlite3

DB_PATH = "config.db"

def init() :
    if not os.path.exists(DB_PATH) :
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS settings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                address TEXT,
                username TEXT,
                password TEXT,
                port TEXT,
                db_name TEXT
            )
        """)

        cursor.execute("""
            CREATE TABLE IF NOT EXISTS calibration (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                moveX TEXT,
                moveY TEXT,
                moveZ TEXT,
                pitch TEXT,
                roll TEXT,
                yaw TEXT
            )
        """)
        conn.commit()
        conn.close()

def get_conf(keys=None) :
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT address, username, password, port, db_name FROM settings ORDER BY id DESC LIMIT 1")
    row = cursor.fetchone()
    conn.close()

    if not row:
        if keys is None:
            return {"address": "", "username": "", "password": "", "port": "", "db_name": ""}
        else:
            return ["" for _ in keys] if isinstance(keys, list) else ""

    columns = ["address", "username", "password", "port", "db_name"]
    data = dict(zip(columns, row))
    
    if keys is None:
        return data

    if isinstance(keys, list):
        values = [data.get(k, "") for k in keys]
        return values
    else:
        return data.get(keys, "")
    
def get_calibration(keys=None) :
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT moveX, moveY, moveZ, pitch, roll, yaw FROM calibration ORDER BY id DESC LIMIT 1")
    row = cursor.fetchone()
    conn.close()

    if not row:
        defaults = {"moveX": "0", "moveY": "0", "moveZ": "0", "pitch": "0", "roll": "0", "yaw": "0"}
        if keys is None:
            return defaults
        elif isinstance(keys, list):
            return [defaults.get(k, "0") for k in keys]
        else:
            return defaults.get(keys, "0")
    
    columns = ["moveX", "moveY", "moveZ", "pitch", "roll", "yaw"]
    data = dict(zip(columns, row))

    if keys is None:
        return data
    elif isinstance(keys, list):
        return [data.get(k, "0") for k in keys]
    else:
        return data.get(keys, "0")

def save_conf(address, username, password, port, db_name) :
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM settings LIMIT 1")
    if cursor.fetchone() :
        cursor.execute("""
            UPDATE settings SET address=?, username=?, password=?, port=?, db_name=? WHERE id = (SELECT id FROM settings LIMIT 1)
        """, (address, username, password, port, db_name))
    else :
        cursor.execute("""
            INSERT INTO settings (address, username, password, port, db_name) VALUES (?, ?, ?, ?, ?)
        """, (address, username, password, port, db_name))
    conn.commit()
    conn.close()

def save_calibration_move(moveX, moveY, moveZ) :
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM calibration LIMIT 1")
    if cursor.fetchone() :
        cursor.execute("""
            UPDATE calibration SET moveX=?, moveY=?, moveZ=? WHERE id = (SELECT id FROM calibration LIMIT 1)
        """, (moveX, moveY, moveZ))
    else :
        cursor.execute("""
            INSERT INTO calibration (moveX, moveY, moveZ) VALUES (?, ?, ?)
        """, (moveX, moveY, moveZ))
    conn.commit()
    conn.close()

def save_calibration_rotate(pitch, roll, yaw) :
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM calibration LIMIT 1")
    if cursor.fetchone() :
        cursor.execute("""
            UPDATE calibration SET pitch=?, roll=?, yaw=? WHERE id = (SELECT id FROM calibration LIMIT 1)
        """, (pitch, roll, yaw))
    else :
        cursor.execute("""
            INSERT INTO calibration (pitch, roll, yaw) VALUES (?, ?, ?)
        """, ( pitch, roll, yaw))
    conn.commit()
    conn.close()