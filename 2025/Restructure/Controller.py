import pygame
import sys

class Controller:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected")
            sys.exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print("Joystick detected")

        self.button_mapping = {
            0:"A",
            1:"B",
            2:"X",
            3:"Y",
            4:"Left_bumper",
            5:"Right_bumper",
            6:"Back",
            7:"Start",
            8:"Xbox",    
            9:"Left_Stick",
            10:"Right_Stick", 
        }

        # store current button states
        self.buttons = {name: False for name in self.button_mapping.values()}

        # store current D-pad state
        self.dpad = (0, 0)  # (x, y)

    def update(self):
        """Update button and D-pad states from pygame events"""
        pygame.event.pump()

        # Update buttons
        for i in range(self.joystick.get_numbuttons()):
            name = self.button_mapping.get(i, f"Button_{i}")
            self.buttons[name] = bool(self.joystick.get_button(i))

        # Update D-pad
        num_hats = self.joystick.get_numhats()
        if num_hats > 0:
            self.dpad = self.joystick.get_hat(0)
        else:
            self.dpad = (0, 0)

    def is_pressed(self, name):
        """Check button presses"""
        return self.buttons.get(name, False)

    def dpad_direction(self):
        """Return string of D-pad direction"""
        x, y = self.dpad
        if (x, y) == (0, 0):
            return None
        if x == -1:
            return "left"
        if x == 1:
            return "right"
        if y == 1:
            return "up"
        if y == -1:
            return "down"
