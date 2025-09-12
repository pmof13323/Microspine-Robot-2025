#ifndef CONFIG_H
#define CONFIG_H

// Number of Dynamixels
#define NUM_DXLs 16

// Serial port and direction pin
#define DXL_SERIAL Serial1
#define DXL_DIR_PIN -1   // adjust if using a real direction pin

// Debugging
#define DEBUG 1

// Max velocity and acceleration constants
#define V_MAX 1023      // adjust based on your Dynamixel model
#define A_MAX 32767     // adjust based on your Dynamixel model

#endif // CONFIG_H
