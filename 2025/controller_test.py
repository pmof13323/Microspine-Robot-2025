import pygame
import sys
import serial
pygame.init()
pygame.joystick.init()

# open serial connection to OpenRB-150
ser = serial.Serial('/dev/ttyACM2', 57600, timeout=1)

if not ser.is_open:
    print("failed to connect to OpenRB-150")
    sys.exit()
else:
    print("connection to OpenRB-150 established")

if pygame.joystick.get_count() == 0:
    print("no joystick")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print (f"joystick detected")

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

print("printing input")

try:
    while True:
        pygame.event.pump()

        # check for button press
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                name = button_mapping.get(i, f"Button_{i}")
                print(f"Button Pressed {name}")
                ser.write((f"BUTTON {name}\n").encode())

        # check for joystick movement
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            if (i == 2 or i == 5) and val >= -0.9:
                print(f"trigger {i} moved: {val:.2f}")
                ser.write((f"TRIGGER {i} {val:.2f}\n").encode())
            elif (i != 2 and i != 5) and abs(val) > 0.1:
                print(f"stick {i} moved: {val:.2f}")
                ser.write((f"STICK {i} {val:.2f}\n").encode())

        # check for D-pad input
        D_pad = joystick.get_hat(0)
        if D_pad != (0, 0):
            if D_pad[0] == -1:
                print("D-pad Left")
                ser.write(b"D_PAD LEFT\n")
            if D_pad[0] == 1:
                print("D-pad Right")
                ser.write(b"D_PAD RIGHT\n")
            if D_pad[1] == -1:
                print("D-pad Down")
                ser.write(b"D_PAD DOWN\n")
            if D_pad[1] == 1:
                print("D-pad Up")
                ser.write(b"D_PAD UP\n")

        pygame.time.wait(50)


except KeyboardInterrupt:
    print("\nstopping\n")

finally:
    if ser.is_open:
        ser.close()    
    pygame.quit()