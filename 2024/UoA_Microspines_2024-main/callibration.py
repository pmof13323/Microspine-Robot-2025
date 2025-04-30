## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

#
# commands for configuring ID of motors as they are set to ID=1 by default
# 

# import port, servo and config files for set up
from port import Port
from servo import Servo
from config import mac as config

# initialise port
port = Port(config.DEVICENAME)
port.init()
port.open()

# create instance of motor using current ID
motor = Servo(7, port.handler)

# ID to be set
motor.setID(3)

# close port to finish command
port.close()