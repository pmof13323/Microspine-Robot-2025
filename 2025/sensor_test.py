from ADCPi import ADCPi
import csv
import time
from datetime import datetime

adc = ADCPi(0x68, 0x69, 18)  # bit rate is 18MHz

# inputting test parameters for filename
test_title = input("Enter test title: ")
test_number = input("Enter test number: ")
csv_file = f"{test_title}_{test_number}.csv"

# initialising CSV file with header
header = [
    "Timestamp",
    "Sensor 1 Raw Value",
    "Sensor 1 Voltage (V)",
    "Sensor 2 Raw Value",
    "Sensor 2 Voltage (V)",
    "Sensor 3 Raw Value",
    "Sensor 3 Voltage (V)",
]

# reading data and writing to CSV
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(header)
    print("logging")

    while True:
        raw1 = adc.read_raw(1)
        voltage1 = adc.read_voltage(1)
        raw2 = adc.read_raw(2)
        voltage2 = adc.read_voltage(2)
        raw3 = adc.read_raw(3)
        voltage3 = adc.read_voltage(3)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        writer.writerow([
            timestamp,
            f"{raw1:.4f}",
            f"{voltage1:.4f}",
            f"{raw2:.4f}",
            f"{voltage2:.4f}",
            f"{raw3:.4f}",
            f"{voltage3:.4f}"
        ])
        file.flush()
        print(f"{timestamp}|{voltage1:.4f}|{voltage2:.4f}|{voltage3:.4f}")
        time.sleep(0.1) #10 samples per second
