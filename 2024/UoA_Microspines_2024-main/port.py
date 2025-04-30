## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

# Port class file:
# Used to connect to microcontroller
# Includes all port-related functions:
# init, open, close, and get port

from dynamixel_sdk import *     # import necessary port initialiser config from SDK

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Default setting
BAUDRATE = 57600                 # Dynamixel default baudrate : 57600

class Port:
    # port class requires the device name and store the handler
    def __init__(self, device):
        self.name = device
        self.handler = None

    # initialise port handler
    def init(self):
        self.handler = PortHandler(self.name)

    def getHandler(self):
        return self.handler
    
    # open port and update baudrate to verify comms to port
    def open(self):
        # Open port
        if self.handler.openPort():
            print("Succeeded opening the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.handler.setBaudRate(BAUDRATE):
            print("Succeeded changing the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    # close port
    def close(self):
        self.handler.closePort()