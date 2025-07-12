import serial
import time
from pynput import keyboard

ser = serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=1)
time.sleep(2)

print("Ready")

def on_press(key):
    try:
        if key.char == 'a':
            ser.write(b'a')
            print("Left")
        elif key.char == 'd':
            ser.write(b'd')
            print("Right")
        elif key.char == 's':
            ser.write(b's')
            print("Stop")
    except AttributeError:
        if key == keyboard.Key.esc:
            print("Exiting.")
            ser.close()
            return False  

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
