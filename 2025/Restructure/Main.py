import pygame
import sys
import time
from Gait import *
import ModeSelector
import Controller

if __name__ == "__main__":
    controller = Controller.Controller()
    openrb= OpenRB()
    gaits = [inital(openrb),PosGait(controller,openrb), AngleGait(), WalkGait()]
    selector = ModeSelector.ModeSelector(gaits)

    # Map D-pad directions to gait indices

    try:
        while True:
            controller.update()
            dir = controller.dpad_direction()
            dir = controller.dpad_direction_mac()
            selector.select_gait_by_button(dir)

            # Run current gait
            selector.current_gait.step()
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()