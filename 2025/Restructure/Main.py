import pygame
import sys
import time
from Gait import *
import ModeSelector
import Controller

if __name__ == "__main__":
    controller = Controller.Controller()
    

    motorAngles = {
            1: [0.0, 0.0, 90.0],
            2: [0.0, 0.0, 90.0],
            3: [0.0, 0.0, 90.0],
            4: [0.0, 0.0, 90.0],
        }

    eePositions = {
            1: [0.0, 0.0, 0.0],
            2: [0.0, 0.0, 0.0],
            3: [0.0, 0.0, 0.0],
            4: [0.0, 0.0, 0.0],
        }

    eeCurrents = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
            4: 0.0,
        }
    openrb= OpenRB("127.0.0.1",5002,None,57600,eeCurrents)

    gaits = [inital(openrb),PosGait(controller,openrb,motorAngles,eePositions), AngleGait(controller,openrb,motorAngles), WalkGait(controller,openrb,motorAngles,eePositions,eeCurrents)]
    selector = ModeSelector.ModeSelector(gaits,openrb)

    # Map D-pad directions to gait indices
    try:
        while True:
            controller.update()
            dir = controller.dpad_direction()
            selector.select_gait_by_button(dir)
        
            # Run current gait
            coord=selector.current_gait.step()

            openrb.read_data(coord)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()