# sensor_loop.py

import threading
import time
from sensor_read import SensorArray

class SensorBackgroundReader:
    def __init__(self):
        self.sensor_array = SensorArray()
        self.lock = threading.Lock()
        self.latest_data = {}

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run_loop(self):
        while self.running:
            try:
                self.sensor_array.step()
                with self.lock:
                    self.latest_data = {
                        "sensors": [row[:] for row in self.sensor_array.voltages],
                        "torques": [row[:] for row in self.sensor_array.forces],
                        "timestamp": self.sensor_array.timestamp,
                    }
            except Exception as e:
                print(f"[Sensor Error] {e}")
            time.sleep(0.05)  # Adjust sampling rate here (10 Hz)

    def get_data(self):
        with self.lock:
            return self.latest_data.copy()
