import serial, time, sys, json

STATE_FILE = "frontend/openrb_state.json"

class OpenRB:
    def __init__(self, port=None, baud=115200):
        # Default port per OS
        if port is None:
            if sys.platform.startswith("win"):
                port = "COM3"
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

    def read_data(self):
        if not self.ser.in_waiting:
            return None

        line = self.ser.readline().decode(errors="ignore").strip()

        if not line.startswith("READ"):
            return None

        try:
            payload = line[5:]  # strip "READ "
            motors = payload.split(";")  # split multiple motors

            data = {}
            for motor in motors:
                if not motor:
                    continue
                parts = motor.split(",")
                if len(parts) != 4:
                    continue  # skip malformed entries
                motor_id = int(parts[0])
                data[motor_id] = {
                    "load": float(parts[1]),
                    "vel": float(parts[2]),
                    "pos": float(parts[3])
                }

            # print for debugging
            #print("[OpenRB] Read data:", data)

            # write to JSON
            with open(STATE_FILE, "w") as f:
                json.dump(data, f)

            return data

        except Exception as e:
            print(f"[OpenRB] Parse error: {e}, line={line}")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[OpenRB] Serial closed")
