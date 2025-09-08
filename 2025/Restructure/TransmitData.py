import serial, time, sys

class OpenRB:
    def __init__(self, port=None, baud=115200):
        # Default port per OS
        if port is None:
            if sys.platform.startswith("win"):
                port = "COM3"
            elif sys.platform.startswith("linux"):
                port = "/dev/ttyUSB0"
            elif sys.platform.startswith("darwin"):  # macOS
                port = "/dev/tty.usbmodem2101"

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # let board enumerate
        print(f"[OpenRB] Connected on {port} at {baud} baud")

    def send_leg_command(self, leg, q1, q2, q3, grip):
        """Send one line to OpenRB over serial."""
        line = f"L {leg} {q1:.2f} {q2:.2f} {q3:.2f} {grip:.2f}\n"
        self.ser.write(line.encode())
        print(line.strip())

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()