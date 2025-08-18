from ADCPi import ADCPi
import csv
import time
from datetime import datetime

VCC = 5.0  # supply voltage 
R_FIXED = 100000.0  # 100 kΩ known resistor
EPS = 1e-6  # avoid division by zero 

adc = ADCPi(0x68, 0x69, 18)  # bit rate is 18MHz


# V = Vcc * R_fixed / (R_fixed + R_fsr)
def resistance_from_voltage(v):
    v = min(max(v, EPS), VCC - EPS)
    return R_FIXED * ((VCC - v) / v)  # ohms


# R[kΩ] = 336.04 * F[g]^(-0.712)
def force_g_from_resistance_ohm(r_ohm):
    r_kohm = r_ohm / 1000.0
    return (r_kohm / 336.04) ** (-1.0 / 0.712)


def force_N_from_g(Fg):
    return (Fg / 1000.0) * 9.81


# inputting test parameters for filename
test_title = input("Enter test title: ")
test_number = input("Enter test number: ")
csv_file = f"{test_title}_{test_number}.csv"

# initialising CSV file with header
header = [
    "Timestamp",

    "Sensor 1 Raw Value",
    "Sensor 1 Voltage (V)",
    "Sensor 1 Resistance (Ohm)",
    "Sensor 1 Force (g)",
    "Sensor 1 Force (N)",

    "Sensor 2 Raw Value",
    "Sensor 2 Voltage (V)",
    "Sensor 2 Resistance (Ohm)",
    "Sensor 2 Force (g)",
    "Sensor 2 Force (N)",

    "Sensor 3 Raw Value",
    "Sensor 3 Voltage (V)",
    "Sensor 3 Resistance (Ohm)",
    "Sensor 3 Force (g)",
    "Sensor 3 Force (N)",
]

# reading data and writing to CSV
with open(csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(header)
    print("logging")

    while True:

        raw1 = adc.read_raw(1)
        voltage1 = adc.read_voltage(1)
        r1 = resistance_from_voltage(voltage1)
        f1_g = force_g_from_resistance_ohm(r1)
        f1_N = force_N_from_g(f1_g)

        raw2 = adc.read_raw(2)
        voltage2 = adc.read_voltage(2)
        r2 = resistance_from_voltage(voltage2)
        f2_g = force_g_from_resistance_ohm(r2)
        f2_N = force_N_from_g(f2_g)

        raw3 = adc.read_raw(3)
        voltage3 = adc.read_voltage(3)
        r3 = resistance_from_voltage(voltage3)
        f3_g = force_g_from_resistance_ohm(r3)
        f3_N = force_N_from_g(f3_g)

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        writer.writerow(
            [
                timestamp,

                f"{raw1:.4f}",
                f"{voltage1:.4f}",
                f"{r1:.1f}",
                f"{f1_g:.1f}",
                f"{f1_N:.3f}",
            
                f"{raw2:.4f}",
                f"{voltage2:.4f}",
                f"{r2:.1f}",
                f"{f2_g:.1f}",
                f"{f2_N:.3f}",

                f"{raw3:.4f}",
                f"{voltage3:.4f}",
                f"{r3:.1f}",
                f"{f3_g:.1f}",
                f"{f3_N:.3f}",
            ]
        )
        file.flush()
        print(
            f"{timestamp} | "
            f"S1 Raw={raw1:.4f} V={voltage1:.4f} R={r1:.1f}Ω F={f1_g:.1f}g/{f1_N:.3f}N | "
            f"S2 Raw={raw2:.4f} V={voltage2:.4f} R={r2:.1f}Ω F={f2_g:.1f}g/{f2_N:.3f}N | "
            f"S3 Raw={raw3:.4f} V={voltage3:.4f} R={r3:.1f}Ω F={f3_g:.1f}g/{f3_N:.3f}N"
        )
        time.sleep(0.1)  # 10 samples per second
