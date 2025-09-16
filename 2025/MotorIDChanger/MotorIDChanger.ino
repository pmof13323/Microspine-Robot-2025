#include <Dynamixel2Arduino.h>

#define OLD_ID 12   // Default ID for new motors
#define NEW_ID 16     // Change this to 3, 4, etc. for other motors

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #error "This sketch is configured for OpenRB only."
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
const float DXL_PROTOCOL_VERSION = 2.0;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("Pinging ID ");
  DEBUG_SERIAL.println(OLD_ID);
  
  if (dxl.ping(OLD_ID)) {
    dxl.torqueOff(OLD_ID);
    dxl.writeControlTableItem(ControlTableItem::ID, OLD_ID, NEW_ID);
    DEBUG_SERIAL.print("ID changed to ");
    DEBUG_SERIAL.println(NEW_ID);
  } else {
    DEBUG_SERIAL.println("Motor not found.");
  }
}

void loop() {
  // Nothing here
}
