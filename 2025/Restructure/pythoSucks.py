import serial
from serial import Serial
s = serial.Serial("COM3", 115200, timeout=0.1)
print("Connected:", s.is_open)
s.close()