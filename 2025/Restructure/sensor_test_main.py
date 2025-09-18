import time
from sensor_array import SensorArray  # assuming you put the class in sensor_array.py

def main():
    sensors = SensorArray()
    print("Reading sensors... Press Ctrl+C to stop.")
    
    try:
        while True:
            sensors.step()
            sensors.print_status()
            time.sleep(0.1)  # 10 Hz refresh rate
    except KeyboardInterrupt:
        print("\nStopped by user.")

if __name__ == "__main__":
    main()