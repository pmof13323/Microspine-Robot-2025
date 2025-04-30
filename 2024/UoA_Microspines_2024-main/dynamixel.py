#!/usr/bin/env python
## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

# DYNAMIXEL Driver file:
# Links writing and reading memory addresses to functions
# Creates variables to store each memory address value
# Modular functions which can be modified to suit project 

# Get all necessary functions and configurations from Dynamixel SDK
from dynamixel_sdk import *         # Uses Dynamixel SDK library

## Control Table Address ##
# EEPROM Area 
ADDR_ID                 = 7
ADDR_BAUD_RATE          = 8
ADDR_OPERATING_MODE     = 11 

ADDR_TEMP_LIMIT         = 31
ADDR_MIN_VOLTAGE_LIMIT  = 32
ADDR_MAX_VOLTAGE_LIMIT  = 34
ADDR_CURRENT_LIMIT      = 38
ADDR_VELOCITY_LIMIT     = 44
ADDR_MIN_POSITION_LIMIT = 48
ADDR_MAX_POSITION_LIMIT = 52

ADDR_SHUTDOWN           = 63        # 1 Byte Address      


# RAM Area
ADDR_TORQUE_ENABLE      = 64        # 1 Byte Address

ADDR_POSITION_D_GAIN    = 80
ADDR_POSITION_I_GAIN    = 82
ADDR_POSITION_P_GAIN    = 84

ADDR_GOAL_CURRENT       = 102       
ADDR_GOAL_VELOCITY      = 104
ADDR_GOAL_POSITION      = 116       # 4 Byte Address

ADDR_PRESENT_CURRENT    = 126       # 2 Byte Address   
ADDR_PRESENT_VELOCITY   = 128       # 4 Byte Address   
ADDR_PRESENT_POSITION   = 132       # 4 Byte Address   


# Operating Modes
DXL_CURRENT_CONTROL         = 0
DXL_VELOCITY_CONTROL        = 1
DXL_POSITION_CONTROL        = 3         # Default 
DXL_EXTENDED_POS_CONTROL    = 4         # Extended Position Control Mode(Multi-turn)
DXL_CURRENT_POS_CONTROL     = 5         # Current-based Position Control Mode
DXL_PWM_CONTROL             = 16        # PWM Control Mode (Voltage Control Mode)

# Protocol version
PROTOCOL_VERSION = 2.0 

# Torque 
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DXL_MINIMUM_POSITION_VALUE = 2150           
DXL_MAXIMUM_POSITION_VALUE = 2680

DXL_POSITION_P_GAIN = 500
DXL_POSITION_I_GAIN = 50
DXL_POSITION_D_GAIN = 900

DXL_MOVING_STATUS_THRESHOLD = 50

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

##################
## ID FUNCTIONS ##
##################

# set ID
def setID(portHandler, ID, newID):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, ID, ADDR_ID, newID
    )

    if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return newID

#####################
## OPERATING MODES ##
#####################
    
# Get Operating Mode
def getOpMode(portHandler, ID):
    dxl_operation_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, ID, ADDR_OPERATING_MODE
    )

    if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_operation_mode

# Set Operating Mode
def setOpMode(portHandler, ID, mode):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, ID, ADDR_OPERATING_MODE, mode
    )

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1
    
########################
## POSITION FUNCTIONS ##
########################

# Get current position
def getPresentPosition(portHandler, ID):
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, ID, ADDR_PRESENT_POSITION
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_present_position

# Set Goal Position    
def setPosition(portHandler, ID, goal_position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1

############################
## TORQUE ENABLE CONTROLS ##
############################

# Set Torque Enable value
def setTorque(portHandler, ID, status):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, ID, ADDR_TORQUE_ENABLE, status
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1
    
#######################
## CURRENT FUNCTIONS ##
#######################
    
# Set Goal Current
def setGoalCurrent(portHandler, ID, val):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, ID, ADDR_GOAL_CURRENT, val
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1
    
# Get Present Current
def getPresent_Current(portHandler, ID):
    dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, ID, ADDR_PRESENT_CURRENT
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_present_current
    
#########################
## POSITION PID VALUES ##
#########################
    
# Set P Gain
def setPosition_P_Gain(portHandler, ID, val):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, ID, ADDR_POSITION_P_GAIN, val
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1

# Get P Gain 
def getPosition_P_Gain(portHandler, ID):
    dxl_position_p_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, ID, ADDR_POSITION_P_GAIN
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_position_p_gain
    
# Set I Gain
def setPosition_I_Gain(portHandler, ID, val):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, ID, ADDR_POSITION_I_GAIN, val
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1
    
# Get I Gain 
def getPosition_I_Gain(portHandler, ID):
    dxl_position_i_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, ID, ADDR_POSITION_I_GAIN
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_position_i_gain
    
# Set D Gain
def setPosition_D_Gain(portHandler, ID, val):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
        portHandler, ID, ADDR_POSITION_D_GAIN, val
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1
    
# Get D Gain 
def getPosition_D_Gain(portHandler, ID):
    dxl_position_d_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
        portHandler, ID, ADDR_POSITION_D_GAIN
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_position_d_gain    
    

###############################
## MOTOR SHUTDOWN AND REBOOT ##
###############################

# Get any Shutdown messages
def getShutdown(portHandler, ID):
    dxl_shutdown, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, ID, ADDR_SHUTDOWN
    )
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return dxl_shutdown

# Reboot Motor for restarting processes
def Reboot(portHandler, ID):
    dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, ID)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        return 1