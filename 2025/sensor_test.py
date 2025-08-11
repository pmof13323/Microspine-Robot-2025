from ADCPi import ADCPi
import csv
import time
from datetime import datetime

adc = ADCPi(0x68, 0x69, 18)  # bit rate is 18MHz

csv_file = "voltage_log.csv"

# inputting test parameters
test_title = input("Enter test title: ")
test_number = input("Enter test number: ")

# initialising CSV file with header
header = [
    "Test Title",
    "Test Number",
    "Timestamp",
    "Sensor 1 Voltage (V)",
    "Sensor 2 Voltage (V)",
    "Sensor 3 Voltage (V)",
]

# reading data and writing to CSV
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(header)
    print("logging")

    while True:
        voltage1 = adc.read_voltage(1)
        voltage2 = adc.read_voltage(2)
        voltage3 = adc.read_voltage(3)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        writer.writerow(
            [
                test_title,
                test_number,
                timestamp,
                f"{voltage1:.4f}",
                f"{voltage2:.4f}",
                f"{voltage3:.4f}",
            ]
        )
        file.flush()
        print(f"{timestamp}|{voltage1:.4f}|{voltage2:.4f}|{voltage3:.4f}")
        time.sleep(0.1)
