import serial, time, sys, json
import socket
import threading
import time
import serial
import sys


STATE_FILE = "frontend/openrb_state.json"

def null_legs():
    positions = {}
    for i in (1, 2, 3, 4):
        positions[i] = dict(x=float(0), y=float(0), z=float(0))
    return positions


class OpenRB:
    def __init__(self, host="127.0.0.1", port=5002, serial_port=None, baud=57600):
        # Serial setup
        if serial_port is None:
            if sys.platform.startswith("win"):
                serial_port = "COM4"
            elif sys.platform.startswith("linux"):
                serial_port = "/dev/ttyACM0"
            elif sys.platform.startswith("darwin"):
                serial_port = "/dev/tty.usbmodem2101"

        self.ser = serial.Serial(serial_port, baud, timeout=0.1)
        time.sleep(2.0)
        print(f"[OpenRB] Connected on {serial_port} at {baud} baud")

        # TCP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, port))
        self.sock.listen(1)
        print(f"[OpenRB] Waiting for client on {host}:{port}")

        self.conn, addr = self.sock.accept()
        print(f"[OpenRB] Client connected from {addr}")

    def send_sync_positions(self, id_pos_pairs):
        """
        Send all servo positions at once.
        id_pos_pairs: list of (id, position) tuples
        Example: [(1, 512), (2, 256), (3, 900)]
        """
        parts = []
        for dxl_id, pos in id_pos_pairs:
            parts.append(f"{dxl_id} {pos}")
        line = "SYNC " + " ".join(parts) + "\n"
        self.ser.write(line.encode())
        print("[SYNC SEND]", line.strip())

    def read_data(self, coord):
        line = "READ " + "\n"
        self.ser.write(line.encode())

        if not self.ser.in_waiting:
            return None

        line = self.ser.readline().decode(errors="ignore").strip()

        if not line.startswith("READ"):
            return None

        try:
            payload = line[5:]
            motors = payload.split(";")
            data = {}
            for motor in motors:
                if not motor:
                    continue
                parts = motor.split(",")
                if len(parts) != 4:
                    continue
                motor_id = int(parts[0])
                data[motor_id] = {
                    "load": float(parts[1]),
                    "pos": float(parts[2]),
                    "vel": float(parts[3])
                }

            # Build combined data AFTER parsing all motors
            if not coord:
                combined_data = {
                    "motors": data,
                    "legs": null_legs()
                }
            else :
                combined_data = {
                    "motors": data,
                    "legs": coord
                }

           import serial, time, sys, json
import socket
import threading
import time
import serial
import sys

STATE_FILE = "frontend/openrb_state.json"

def null_legs():
    positions = {}
    for i in (1, 2, 3, 4):
        positions[i] = dict(x=float(0), y=float(0), z=float(0))
    return positions


class OpenRB:
    def __init__(self, host="127.0.0.1", port=5002, serial_port=None, baud=57600,
                 currents_ref=eeCurrents):
        """
        currents_ref: optional reference to a dict that should hold present currents (mA)
                      keyed by motor ID; e.g., eeCurrents in Main.py
        """
        # Serial setup
        if serial_port is None:
            if sys.platform.startswith("win"):
                serial_port = "COM4"
            elif sys.platform.startswith("linux"):
                serial_port = "/dev/ttyACM0"
            elif sys.platform.startswith("darwin"):
                serial_port = "/dev/tty.usbmodem2101"

        self.ser = serial.Serial(serial_port, baud, timeout=0.1)
        time.sleep(2.0)
        print(f"[OpenRB] Connected on {serial_port} at {baud} baud")

        # TCP socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((host, port))
        self.sock.listen(1)
        print(f"[OpenRB] Waiting for client on {host}:{port}")

        self.conn, addr = self.sock.accept()
        print(f"[OpenRB] Client connected from {addr}")

        # --- NEW: shared currents dict (optional) + lock
        self.eeCurrents = currents_ref  # reference to the dict in Main.py
        self._lock = threading.Lock()

    def send_sync_positions(self, id_pos_pairs):
        parts = []
        for dxl_id, pos in id_pos_pairs:
            parts.append(f"{dxl_id} {pos}")
        line = "SYNC " + " ".join(parts) + "\n"
        self.ser.write(line.encode())
        print("[SYNC SEND]", line.strip())

    def read_data(self, coord):
        # request a fresh line
        line = "READ " + "\n"
        self.ser.write(line.encode())

        if not self.ser.in_waiting:
            return None

        line = self.ser.readline().decode(errors="ignore").strip()

        if not line.startswith("READ"):
            return None

        try:
            payload = line[5:]
            motors = payload.split(";")
            data = {}
            for motor in motors:
                if not motor:
                    continue
                parts = motor.split(",")
                if len(parts) != 4:
                    continue
                motor_id = int(parts[0])
                data[motor_id] = {
                    "load": float(parts[1]),  # <-- your "current" value
                    "pos":  float(parts[2]),
                    "vel":  float(parts[3])
                }

            # update the shared eeCurrents dict (if provided)
            if self.eeCurrents is not None:
                for motor_id, leg in self.motor_to_leg.items():
                    if motor_id in data:
                        self.eeCurrents[leg] = data[motor_id]["load"]

            # Send to connected client
            self.conn.sendall((json.dumps(combined_data) + "\n").encode())

            return data

        except Exception as e:
            print(f"[OpenRB] Parse error: {e}, line={line}")
            return None

    
    def transmit_mode(self,int):
        line = ("mode"+ str(int) + "\n").encode()
        self.conn.sendall(line)
    
    def transmit_leg(self,int):
        line = ("leg"+ str(int) + "\n").encode()
        self.conn.sendall(line)


    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OpenRB] Serial closed")