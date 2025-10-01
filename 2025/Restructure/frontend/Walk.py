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

    # Automatically appends EE
        # Takes in a leg number, and a 'sticky' flag
        # Returns a Bool
            # True if sensors give good reading after EE current reached
            # False if sensors give bad reading after EE current reached
    def appedEE()
        # Start adhering EE

        # Once current is reached, run if statement
            # if all sensors are green
                # return True
            
            # if less than 3 sensors are green 
                # retract EE
                # return false when fully retracted





    # Completes the follow steps to move a limb
      # 1. Translate up in Z from current EE location
      # 2. Move EE across to new location
      # 3. Translate EE down in z to new location
        # Takes in leg number, new position
        # Returns a Bool
            # True if successful
            # False if failure occurs
    def moveLimb(legNum)
        # Read xyz of requested EE
        legXYZ = Positions.get_all_leg_positions().[legNum]

        # Translate the EE +z 
        legXYZ[2] += incrementer
        sol, feasible = self._solve_leg_for_world_target(self.leg, self.Pw)
            if feasible:
                self.last_valid[self.leg]["sol"] = sol
                self.last_valid[self.leg]["Pw"]  = self.Pw.copy()
            else:
                sol = self.last_valid[self.leg]["sol"] if self.last_valid[self.leg]["sol"] else sol
                if self.last_valid[self.leg]["Pw"] is not None:
                    self.Pw = self.last_valid[self.leg]["Pw"].copy()


        # Move EE to requested x and y location

        # Translate the EE -z

        # return True





    # runs a single iteration in the direction of the stick (N, E, S, W only)
        # Takes in a direction
        # Returns a Bool
            # True when cycle is done
            # False if failure occurs
    def moveThisDirection(direction)

        # Given the requested direction, calculate the EE positions
        if direction == 'N'
            

    
        # Start gait cycle
            # Move leg 1
            # call appendEE
                # if false, call appendEE again
                # if true, move on
            
            # Move leg 2
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move leg 3
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move leg 4
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move body in direction 

    




    # moves the body to its initil position
        # returns true when complete
        # returns false if failure occurs 
    def moveToInitial()
            
        # Move the robot back into its initial position    
            # Move leg 1
            # call appendEE
                # if false, call appendEE again
                # if true, move on
            
            # Move leg 2
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move leg 3
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move leg 4
            # call appendEE
                # if false, call appendEE again
                # if true, move on

            # Move body forward
            





    def step(self):
        print("Running WalkGait...")
        sync_targets = []

        # Move robot into initial psoition and adhering


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