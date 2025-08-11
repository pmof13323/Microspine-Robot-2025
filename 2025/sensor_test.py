from ADCPi import ADCPi
import csv
import time
from datetime import datetime

adc = ADCPi(0x68,0x68, 18)

csv_file = "voltage_log.csv"

header = ["Timestamp", "Voltage (V)"]


with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    print("logging")
    
    while True:
        voltage = adc.read_voltage(1)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        writer.writerow([timestamp, f"{voltage:.4f}"])
        file.flush()
        print(f"{timestamp}|{voltage:.4f}")
        time.sleep(0.1)

