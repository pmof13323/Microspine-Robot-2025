import serial, time, sys

class OpenRB:
    def __init__(self, port=None, baud=115200):
        # Default port per OS
        if port is None:
            if sys.platform.startswith("win"):
                port = "COM4"
            elif sys.platform.startswith("linux"):
                port = "/dev/ttyUSB0"
            elif sys.platform.startswith("darwin"):  # macOS
                port = "/dev/tty.usbmodem21101"

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # let board enumerate
        print(f"[OpenRB] Connected on {port} at {baud} baud")

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

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OpenRB] Serial closed")
