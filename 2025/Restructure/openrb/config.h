#ifndef CONFIG_H
#define CONFIG_H

// Number of Dynamixels
#define NUM_DXLs 16

// Serial port and direction pin
#define DXL_SERIAL Serial1

// Debugging
#define DEBUG 1

// Max velocity and acceleration constants
#define V_MAX 80      // adjust based on your Dynamixel model
#define A_MAX 10     // adjust based on your Dynamixel model

#endif // CONFIG_H
