import numpy as np
import time
from TransmitData import OpenRB
from Position import *

class WalkGait:
    def __init__(self, controller, OpenRB, Angles):
        self.name = "Walking cycle"
        self.controller = controller
        self.rb = OpenRB  # serial comms
        self.angles = Angles
        incrementer = 30 #mm
        sticky = True

    # Automatically appends EE
        # Takes in a leg number, and a 'sticky' flag
        # Returns a Bool
            # True if sensors give good reading after EE current reached
            # False if sensors give bad reading after EE current reached
    def appedEE(legNum, stickyFlag)
        # Checking for sticky flag
        if stickyFlag
            return True
        
        # Start adhering EE
        while current < maxCurrent
            # send sync write to activate leg ee
            # run a timer to determine how long it moves for

        # Once current is reached, run if statement
            # if all sensors are green
        if sensorVoltage > 13.2 #Assuming sensor voltage of 4.4V per sensor
            return True
            
            # if less than 3 sensors are green 
        if sensorVoltage <= 13.2
            # send sync write to deactivate leg ee, runnning for a duration as determined by the above timer
            
            # one ee full disengaged
            return False




    # Completes the follow steps to move a limb
      # 1. Translate up in Z from current EE location
      # 2. Move EE across to new location
      # 3. Translate EE down in z to new location
        # Takes in leg number, new position
        # Returns a Bool
            # True if successful
            # False if failure occurs
    def moveLimb(legNum, goalPos)
        # Read xyz of requested EE
        legXYZ = Positions.get_all_leg_positions().[legNum] # This is fro Position.py

        # Translate the EE +z 
        legXYZ[2] += incrementer
        sol, feasible = self._solve_leg_for_world_target(self.leg, legXYZ) # This is fro Position.py
            # May need to include a condition to deny if its feasable
            # send sol angles to sync write
            
        # Move EE to requested x and y location
        legXYZ[0] = goalPos[0] # x
        legXYZ[1] = goalPos[1] # y
        sol, feasible = self._solve_leg_for_world_target(self.leg, legXYZ) # This is fro Position.py
            # May need to include a condition to deny if its feasable
            # send sol angles to sync write

        # Translate the EE -z
        sol, feasible = self._solve_leg_for_world_target(self.leg, goalPos) # This is fro Position.py
            # May need to include a condition to deny if its feasable
            # send sol angles to sync write

        # NEED a way to keep track of XYZ positions

       return True





    # runs a single iteration in the direction of the stick (N, E, S, W only)
        # Takes in a direction
        # Returns a Bool
            # True when cycle is done
            # False if failure occurs
    def moveThisDirection(direction)

        # Given the requested direction, calculate the EE positions
        if direction == 'N'
            leg1XYZ = [175.0, -145.26, -207.5]
            leg2XYZ = [0.0, -145.26, -207.5]
            leg3XYZ = [0.0, 145.26, -207.5]
            leg4XYZ = [175.0, 145.26, -207.5]
        elif direction == 'S'
            leg1XYZ = [0.0, -145.26, -207.5]
            leg2XYZ = [-175.0, -145.26, -207.5]
            leg3XYZ = [-175.0, 145.26, -207.5]
            leg4XYZ = [0.0, 145.26, -207.5]
        elif direction == 'W'
            leg1XYZ = [160.0, 0.0, -207.5]
            leg2XYZ = [-160.0, 0.0, -207.5]
            leg3XYZ = [-160.0, 175.0, -207.5]
            leg4XYZ = [160.0, 175.0, -207.5]
        elif direction == 'E'
            leg1XYZ = [160.0, 175.0, -207.5]
            leg2XYZ = [-160.0, 175.0, -207.5]
            leg3XYZ = [-160.0, 0.0, -207.5]
            leg4XYZ = [160.0, 0.0, -207.5]
    
        # Start gait cycle
        # Move leg 1
        moveLimb(1, leg1XYZ)
        # call appendEE
        eeStatus = appedEE(1, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(1, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 2
        moveLimb(2, leg2XYZ)
        # call appendEE
        eeStatus = appedEE(2, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(2, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 3
        moveLimb(3, leg3XYZ)
        # call appendEE
        eeStatus = appedEE(3, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(3, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 4
        moveLimb(4, leg4XYZ)
        # call appendEE
        eeStatus = appedEE(4, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(4, stickyFlag)
            failureCount++
            if failureCount > 3
                return False


            # Move body in direction 
                # maybe call a move body func from Position.py
    
        return True



    # moves the body to its initil position
        # returns true when complete
        # returns false if failure occurs 
    def moveToInitial()


        leg1Home = [159.0, -145.0, -205.0]
        leg2Home = [-159.0, -145.0, -205.0]
        leg3Home = [-159.0, 145.0, -205.0]
        leg4Home = [159.0, 145.0, -205.0]
            
        # Move the robot back into its initial position    
        # Move leg 1
        moveLimb(1, leg1Home)
        # call appendEE
        eeStatus = appedEE(1, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(1, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 2
        moveLimb(2, leg2Home)
        # call appendEE
        eeStatus = appedEE(2, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(2, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 3
        moveLimb(3, leg3Home)
        # call appendEE
        eeStatus = appedEE(3, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(3, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        # Move leg 4
        moveLimb(4, leg4Home)
        # call appendEE
        eeStatus = appedEE(4, stickyFlag) 
            # if false, call appendEE again
            # if true, move on
        failureCount = 0
        if !eeStatus
            eeStatus = appedEE(4, stickyFlag)
            failureCount++
            if failureCount > 3
                return False

        return True
            



    def step(self):
        print("Running WalkGait...")
        sync_targets = []

        # Move robot into initial psoition and adhering
        moveToInitial()

        # Reading in joystick
        js = getattr(self.controller, "joystick", None)

        if js is not None:
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)

                if abs(val) > self.dead:
                    if i == 0: # North/South
                        if val > 0: # North
                            self.direction = 'N'
                        if val < 0: # South
                            self.direction = 'S'

                    elif i == 1: #East/West
                        if val > 0: # East
                            self.direction = 'E'
                        if val < 0: # West
                            self.direction = 'W'

        moveThisDirection(direction)



        # Moving robot in the specified direction


                    
















        print(f"\n")
        print(f" Walking Control Mode ")
        print("+-------------------------------------------------------+")
        print(" ")
        print(" ")
        print("                    Under Construction                   ")
        print(" ")
        print(" ")
        print("+-------------------------------------------------------+")
        self.rb.send_sync_positions(sync_targets)
        print("+-------------------------------------------------------+")