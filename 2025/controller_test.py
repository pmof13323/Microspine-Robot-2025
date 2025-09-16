import pygame
import sys
import serial
pygame.init()
pygame.joystick.init()

# open serial connection to OpenRB-150
## ser = serial.Serial('/dev/ttyACM1', 57600, timeout=1)

'''
if not ser.is_open:
    print("failed to connect to OpenRB-150")
    sys.exit() 
else:
    print("connection to OpenRB-150 established")
'''

if pygame.joystick.get_count() == 0:
    print("no joystick")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print (f"joystick detected")

'''
button_mapping = {
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
'''

button_mapping = {
    0:"A",
    1:"B",
    2:"X",
    3:"Y",
    4:"Back",
    5:"Xbox",
    6:"Start",
    7:"Left_Stick",
    8:"Right_Stick",    
    9:"Left_bumper",
    10:"Right_bumper", 
}

print("printing input")

try:
    while True:
        pygame.event.pump()

        # check for button press
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                name = button_mapping.get(i, f"Button_{i}")
                print(f"Button Pressed {name}")
                ## ser.write((f"BUTTON {name}\n").encode())

        # check for joystick movement
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if (i == 4 or i == 5) and val >= 0:
                print(f"trigger {i} moved: {val:.2f}")
                ## ser.write((f"TRIGGER {i} {val:.2f}\n").encode())
            elif (i != 4 and i != 5) and abs(val) > 0.15:
                print(f"stick {i} moved: {val:.2f}")
                ## ser.write((f"STICK {i} {val:.2f}\n").encode())

        # --- D-pad check ---
        # macOS/Linux: hat(0)
        if joystick.get_numhats() > 0:
            dx, dy = joystick.get_hat(0)
            if dx == 1:  print("D-pad Right")
            if dx == -1: print("D-pad Left")
            if dy == 1:  print("D-pad Up")
            if dy == -1: print("D-pad Down")

        # Windows fallback: buttons
        else:
            if joystick.get_button(11): print("D-pad Up +")
            if joystick.get_button(12): print("D-pad Down +")
            if joystick.get_button(13): print("D-pad Left +")
            if joystick.get_button(14): print("D-pad Right +")


        pygame.time.wait(50)


except KeyboardInterrupt:
    print("\nstopping\n")

## finally:
    ## if ser.is_open:
        ## ser.close()    
    ## pygame.quit()