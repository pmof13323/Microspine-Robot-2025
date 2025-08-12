#include <Dynamixel2Arduino.h>

// OpenRB-150 setup
#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #error "This sketch is designed for OpenRB-150."
#endif

// Motor IDs
const uint8_t TOES      = 13;
const uint8_t HIP_YAW   = 1;
const uint8_t HIP_PITCH = 2;
const uint8_t KNEE      = 3;

const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// === Tunables ===
const int16_t TOES_SPEED      = 300;  // A → +, B → -
const int16_t HIP_YAW_SPEED   = 20;  // D-pad RIGHT → +, LEFT → -
const int16_t HIP_PITCH_SPEED = 20;  // D-pad UP → +, DOWN → -
const int16_t KNEE_SPEED      = 20;  // Right bumper → +, Left bumper → -
const uint32_t WATCHDOG_MS    = 200;  // stop channel if no command within this time

// State (cached velocities + last-refresh times)
int16_t toes_vel_cmd = 0;      uint32_t last_toes_ms = 0;
int16_t hipyaw_vel_cmd = 0;    uint32_t last_hipyaw_ms = 0;
int16_t hippitch_vel_cmd = 0;  uint32_t last_hippitch_ms = 0;
int16_t knee_vel_cmd = 0;      uint32_t last_knee_ms = 0;

void setVel(uint8_t id, int16_t &cached, int16_t v) {
  if (v != cached) {
    dxl.setGoalVelocity(id, v);
    cached = v;
  }
}

void stopAll() {
  setVel(TOES, toes_vel_cmd, 0);
  setVel(HIP_YAW, hipyaw_vel_cmd, 0);
  setVel(HIP_PITCH, hippitch_vel_cmd, 0);
  setVel(KNEE, knee_vel_cmd, 0);
}

void prepMotor(uint8_t id) {
  dxl.ping(id);
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_VELOCITY);
  dxl.torqueOn(id);
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  Serial.setTimeout(5); // snappy line reads

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  prepMotor(TOES);
  prepMotor(HIP_YAW);
  prepMotor(HIP_PITCH);
  prepMotor(KNEE);

  while (!Serial); // Wait for USB-CDC
  Serial.println("Initialisation Complete");

  uint32_t now = millis();
  last_toes_ms = last_hipyaw_ms = last_hippitch_ms = last_knee_ms = now;
}

void loop() {
  uint32_t now = millis();

  // Read one full line if available
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      // Examples from your Python:
      // "D_PAD LEFT/RIGHT/UP/DOWN"
      // "BUTTON A/B/X/Left_bumper/Right_bumper"
      // (sticks/triggers may also arrive; we just ignore them)
      // Serial.print("Received: "); Serial.println(line);

      // --- STOP ALL ---
      if (line == "BUTTON X") {
        stopAll();
        last_toes_ms = last_hipyaw_ms = last_hippitch_ms = last_knee_ms = now;
      }
      // --- TOES via A/B ---
      else if (line == "BUTTON A") {
        setVel(TOES, toes_vel_cmd, +TOES_SPEED);
        last_toes_ms = now;
      } else if (line == "BUTTON B") {
        setVel(TOES, toes_vel_cmd, -TOES_SPEED);
        last_toes_ms = now;
      }
      // --- KNEE via bumpers ---
      else if (line == "BUTTON Right_bumper") {
        setVel(KNEE, knee_vel_cmd, +KNEE_SPEED);
        last_knee_ms = now;
      } else if (line == "BUTTON Left_bumper") {
        setVel(KNEE, knee_vel_cmd, -KNEE_SPEED);
        last_knee_ms = now;
      }
      // --- HIP_YAW via D-pad LEFT/RIGHT ---
      else if (line == "D_PAD RIGHT") {
        setVel(HIP_YAW, hipyaw_vel_cmd, +HIP_YAW_SPEED);
        last_hipyaw_ms = now;
      } else if (line == "D_PAD LEFT") {
        setVel(HIP_YAW, hipyaw_vel_cmd, -HIP_YAW_SPEED);
        last_hipyaw_ms = now;
      }
      // --- HIP_PITCH via D-pad UP/DOWN ---
      else if (line == "D_PAD UP") {
        setVel(HIP_PITCH, hippitch_vel_cmd, +HIP_PITCH_SPEED);
        last_hippitch_ms = now;
      } else if (line == "D_PAD DOWN") {
        setVel(HIP_PITCH, hippitch_vel_cmd, -HIP_PITCH_SPEED);
        last_hippitch_ms = now;
      }
      // else: ignore other lines (STICK/TRIGGER/etc.)
    }
  }

  // --- Watchdogs: zero any channel that hasn't been refreshed recently ---
  if (now - last_toes_ms > WATCHDOG_MS)     setVel(TOES, toes_vel_cmd, 0);
  if (now - last_hipyaw_ms > WATCHDOG_MS)   setVel(HIP_YAW, hipyaw_vel_cmd, 0);
  if (now - last_hippitch_ms > WATCHDOG_MS) setVel(HIP_PITCH, hippitch_vel_cmd, 0);
  if (now - last_knee_ms > WATCHDOG_MS)     setVel(KNEE, knee_vel_cmd, 0);
}
