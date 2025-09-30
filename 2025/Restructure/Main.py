import pygame
import sys
import time
from Gait import *
import ModeSelector
import Controller

if __name__ == "__main__":
    controller = Controller.Controller()
    openrb= OpenRB()

    motorAngles = {
            1: [0.0, 0.0, 90.0],
            2: [0.0, 0.0, 90.0],
            3: [0.0, 0.0, 90.0],
            4: [0.0, 0.0, 90.0],
        }
    
    gaits = [inital(openrb),PosGait(controller,openrb,motorAngles), AngleGait(controller,openrb,motorAngles), WalkGait(controller,openrb,motorAngles)]
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