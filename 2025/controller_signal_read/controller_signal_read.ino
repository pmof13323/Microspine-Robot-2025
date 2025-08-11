

#include <Dynamixel2Arduino.h>

// Hardware configuration for OpenRB-150
#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #error "This sketch is designed for OpenRB-150."
#endif

const uint8_t DXL_ID = 5;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

void setup() {
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(DXL_ID);
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);

  while (!Serial); // Wait for serial port to be ready
  Serial.println("OpenRB-150 Ready");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read one line
    input.trim(); // Remove whitespace
    Serial.print("Received: ");
    Serial.println(input);

    if (input == "D_PAD LEFT") {
      Serial.println("Turning left");
      dxl.setGoalVelocity(DXL_ID, -300);
    } else if (input == "D_PAD RIGHT") {
      Serial.println("Turning right");
      dxl.setGoalVelocity(DXL_ID, 300);
    } else if (input == "BUTTON X") {
      Serial.println("Turning left");
      dxl.setGoalVelocity(DXL_ID, 0);
    }
  }
}
