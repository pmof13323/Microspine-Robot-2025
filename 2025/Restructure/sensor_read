from ADCPi import ADCPi
import csv
import time
import math
from datetime import datetime

class SensorArray:
    VCC = 5.0          # supply voltage
    R_FIXED = 100000.0 # 100 kΩ known resistor
    EPS = 1e-6         # avoid division by zero 
    SCALEFACTOR = 1    # universal scale factor

    def __init__(self):
        # Two ADCs: each has 6 pins → 2 groups per ADC
        self.adc1 = ADCPi(0x68, 0x69, 18)
        self.adc2 = ADCPi(0x6C, 0x6D, 18)

        # 4 groups × 3 sensors
        self.voltages = [[0.0 for _ in range(3)] for _ in range(4)]
        self.raw_data = [[0.0 for _ in range(3)] for _ in range(4)]
        self.resistances = [[0.0 for _ in range(3)] for _ in range(4)]
        self.forces = [[0.0 for _ in range(3)] for _ in range(4)]

        self.timestamp = None
        self.total_force = 0.0

    @staticmethod
    def _resistance_from_voltage(v):
        v = min(max(v, SensorArray.EPS), SensorArray.VCC - SensorArray.EPS)
        return SensorArray.R_FIXED * (SensorArray.VCC / v) - SensorArray.R_FIXED

    @staticmethod
    def _grip_force_from_voltage(v):
        if v < 3.77:
            return 421.81 * v
        else:
            return (8e-05) * (math.exp(4.4006 * v))

    @staticmethod
    def _force_N_from_g(Fg):
        return (Fg / 1000.0) * 9.81

    def step(self):
        """Read all 12 sensors, update stored voltages and other data."""
        self.total_force = 0.0

        # --- First ADC: groups 0 & 1
        for i in range(6):
            group = 0 if i < 3 else 1
            sensor = i % 3

            raw = self.adc1.read_raw(i + 1)
            voltage = self.adc1.read_voltage(i + 1)

            r = self._resistance_from_voltage(voltage)
            f_g = self._grip_force_from_voltage(voltage)
            f_N = self._force_N_from_g(f_g)

            self.raw_data[group][sensor] = raw
            self.voltages[group][sensor] = voltage
            self.resistances[group][sensor] = r
            self.forces[group][sensor] = f_N
            self.total_force += f_N

        # --- Second ADC: groups 2 & 3
        for i in range(6):
            group = 2 if i < 3 else 3
            sensor = i % 3

            raw = self.adc2.read_raw(i + 1)
            voltage = self.adc2.read_voltage(i + 1)

            r = self._resistance_from_voltage(voltage)
            f_g = self._grip_force_from_voltage(voltage)
            f_N = self._force_N_from_g(f_g)

            self.raw_data[group][sensor] = raw
            self.voltages[group][sensor] = voltage
            self.resistances[group][sensor] = r
            self.forces[group][sensor] = f_N
            self.total_force += f_N

        self.timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def getVal(self, group, sensor):
        """Return just the voltage for a given group/sensor index."""
        if 0 <= group < 4 and 0 <= sensor < 3:
            return self.voltages[group][sensor]
        else:
            raise IndexError("Group or sensor index out of range.")

    def print_status(self):
        """Pretty print all sensor voltages for quick debugging."""
        print(f"Timestamp: {self.timestamp}")
        for g in range(4):
            voltages_str = " | ".join(f"{self.voltages[g][s]:.4f}V" for s in range(3))
            print(f"Group {g}: {voltages_str}")
        print(f"Total Force: {self.total_force:.2f}N\n")