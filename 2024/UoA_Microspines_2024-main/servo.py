## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

# Servo Implementation class file:
# Building custom functions from driver file to suit project
# Designed to run commands for each motor
# Storing Port and ID details so that they don't have to be passed every time
# All functions print to console to keep track of status

import dynamixel                # import dynamixel driver file
from types import NoneType      # import NoneType to check when addresses are empty

class Servo:

    ######################
    ## INITIALISE MOTOR ##
    ######################

    def __init__(self, ID, handler):
        # initialise with motorID, gain values, and the port handler 
        self.ID = ID
        self.gain = 5
        self.gain_d = 0.1
        self.port = handler

    ###############
    ## MODIFY ID ##
    ###############
        
    def setID(self, newID):
        res = dynamixel.setID(self.port, self.ID, newID)
        if res == newID:
            print("Succesfully Updated ID")
            self.ID = newID

    #######################
    ## SET LOAD LIMIT ##
    #######################

    def setCurrLim(self, val):
        success = dynamixel.setGoalCurrent(self.port, self.ID, val)
        print("[ID:%03d] Successfully Set Load Limit" % (self.ID)) if success else None

    ###############################
    ## ENABLE AND DISABLE TORQUE ##
    ###############################

    def enable(self):
        success = dynamixel.setTorque(self.port, self.ID, dynamixel.TORQUE_ENABLE)
        print("[ID:%03d] Successfully Enabled Torque" % (self.ID)) if success else None

    def disable(self):
        success = dynamixel.setTorque(self.port, self.ID, dynamixel.TORQUE_DISABLE)
        print("[ID:%03d] Succesfully Disabled Torque" % (self.ID)) if success else None

    ####################
    ## SET PID VALUES ##
    ####################

    # Tested values
    # P = 500
    # I = 50
    # D = 900
        
    def setPos_PID(self, P_val, I_val, D_val):
        P_success = dynamixel.setPosition_P_Gain(self.port, self.ID, P_val)
        I_success = dynamixel.setPosition_I_Gain(self.port, self.ID, I_val)
        D_success = dynamixel.setPosition_D_Gain(self.port, self.ID, D_val)

        print("[ID:%03d] Succesfully Set P Gain" % (self.ID)) if P_success else None
        print("[ID:%03d] Succesfully Set I Gain" % (self.ID)) if I_success else None
        print("[ID:%03d] Succesfully Set D Gain" % (self.ID)) if D_success else None

    def getPos_PID(self):
        P_val = dynamixel.getPosition_P_Gain(self.port, self.ID)
        I_val = dynamixel.getPosition_I_Gain(self.port, self.ID)
        D_val = dynamixel.getPosition_D_Gain(self.port, self.ID)

        return [P_val, I_val, D_val]


    #####################
    ## OPERATING MODES ##
    #####################
    
    def getOpMode(self):
        return dynamixel.getOpMode(self.port, self.ID)
    
    # Set to Current Control Operating Mode
    def setCurrMode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_CURRENT_CONTROL)
        print("[ID:%03d] Succesfully set to Current Control Mode" % (self.ID)) if success else None

    # Set to Velocity Control Operating Mode
    def setVelocityMode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_VELOCITY_CONTROL)
        print("[ID:%03d] Succesfully set to Velocity Control Mode" % (self.ID)) if success else None

    # Set to Position Control Operating Mode
    def setPosMode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_POSITION_CONTROL)
        print("[ID:%03d] Succesfully set to Position Control Mode" % (self.ID)) if success else None

    # Set to Extended Position Control Operating Mode (Multi-turn)
    def setExtendedPosMode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_EXTENDED_POS_CONTROL)
        print("[ID:%03d] Succesfully set to Extended Position Control Mode" % (self.ID)) if success else None

    # Set to Current-Based Position Control Mode
    def setCurrPosMode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_CURRENT_POS_CONTROL)
        print("[ID:%03d] Succesfully set to Current-Based Position Control Mode" % (self.ID)) if success else None

    # Set to PWM Control Mode (Voltage Control Mode)
    def setPWMmode(self):
        success = dynamixel.setOpMode(self.port, self.ID, dynamixel.DXL_PWM_CONTROL)
        print("[ID:%03d] Succesfully set to PWM Voltage Control Mode" % (self.ID)) if success else None

    #######################
    ## POSITION CONTROLS ##
    #######################

    def setPos(self, goal):
        success = dynamixel.setPosition(self.port, self.ID, goal)
        print("[ID:%03d] Succesfully Set Position" % (self.ID)) if success else None

    def getPos(self):
        dxl_present_position = dynamixel.getPresentPosition(self.port, self.ID)
        return dxl_present_position

    ################
    ## ENGAGEMENT ##
    ################

    def getLoad(self):
        current = dynamixel.getPresent_Current(self.port, self.ID)
        
        # catch any motors that are overdriven and need rebooting
        if isinstance(current, NoneType):
            current = 1750
            self.reboot()

        # Engagement processes produce negative current,
        # which when received as an 2 Byte unsigned int, 
        # needs to be converted when over 32767
        current = (65535 - current) if current >= 32767 else current

        # calculation from calibration data
        load = 51.165 + (4.568 * current) if current != 0 else 0
        load = load/1000
        
        message = "[ID:" + str(self.ID) + "] Present Load is: " + str(load) + " kg"
        print(message)

        return load
    
    ###########################
    ## REBOOTS AND SHUTDOWNS ##
    ###########################
    
    def getShutdown(self):
        shutdown = dynamixel.getShutdown(self.port, self.ID)
        return shutdown
    
    def reboot(self):
        success = dynamixel.Reboot(self.port, self.ID)
        print("[ID:%03d] Reboot Succeeded\n" % (self.ID)) if success else None
        

