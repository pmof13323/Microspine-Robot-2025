import serial, time, sys, json
import socket
import threading
import time
import serial
import sys


STATE_FILE = "frontend/openrb_state.json"

class OpenRB:
    def __init__(self, host="127.0.0.1", port=5000, serial_port=None, baud=57600):
        # Serial setup
        if serial_port is None:
            if sys.platform.startswith("win"):
                serial_port = "COM4"
            elif sys.platform.startswith("linux"):
                serial_port = "/dev/ttyUSB0"
            elif sys.platform.startswith("darwin"):
                serial_port = "/dev/tty.usbmodem21101"

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
            combined_data = {
                "motors": data,
                "legs": coord
            }

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
