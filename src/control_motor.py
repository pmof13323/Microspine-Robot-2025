import serial
import keyboard  

ser = serial.Serial('/dev/tty.usbmodemXXXX', 115200)  # replace with actual port

print("Press 'a' (left), 'd' (right), 's' (stop). Press ESC to quit.")

try:
    while True:
        if keyboard.is_pressed('a'):
            ser.write(b'a')
        elif keyboard.is_pressed('d'):
            ser.write(b'd')
        elif keyboard.is_pressed('s'):
            ser.write(b's')
        elif keyboard.is_pressed('esc'):
            print("Exiting.")
            break
except KeyboardInterrupt:
    pass

ser.close()
