#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1; // OpenRB-150 does not use DIR pin
#else
  #error "This sketch is designed for OpenRB-150."
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Common Dynamixel protocol 2.0 baudrates and their corresponding values
const long baudrates[] = {57600, 115200, 1000000, 2000000, 3000000, 4000000};  // Add more if needed
const int num_baudrates = sizeof(baudrates) / sizeof(baudrates[0]);

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  DEBUG_SERIAL.println("🔍 Starting Dynamixel Baudrate and ID Scanner...");

  for (int i = 0; i < num_baudrates; i++) {
    long baud = baudrates[i];
    DEBUG_SERIAL.print("\n Trying baudrate: ");
    DEBUG_SERIAL.println(baud);

    dxl.begin(baud);
    dxl.setPortProtocolVersion(2.0);  // Assuming Protocol 2.0

    // Scan range: Dynamixel default range is 0–252
    for (uint8_t id = 0; id <= 252; id++) {
      if (dxl.ping(id)) {
        DEBUG_SERIAL.print("✅ Found motor at ID ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(" @ ");
        DEBUG_SERIAL.print(baud);
        DEBUG_SERIAL.println(" bps");
      }
    }
  }

  DEBUG_SERIAL.println("\n✅ Scan complete.");
}

void loop() {
  // Nothing to do in loop
}
