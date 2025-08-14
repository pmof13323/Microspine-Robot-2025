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

// === Tunables (max speeds) ===
// Adjust to taste; these are the peak |velocity| when stick/trigger is fully deflected.
const int16_t TOES_MAX_SPEED      = 300;  // triggers → toes (variable)
const int16_t HIP_YAW_MAX_SPEED   = 40;   // left stick X → hip yaw (variable)
const int16_t HIP_PITCH_MAX_SPEED = 40;   // left stick Y → hip pitch (variable)
const int16_t KNEE_MAX_SPEED      = 40;   // right stick Y → knee (variable)

const uint32_t WATCHDOG_MS = 200;  // stop channel if no command within this time

// ========== State (cached velocities + last-refresh times) ==========
int16_t toes_vel_cmd = 0;      uint32_t last_toes_ms = 0;
int16_t hipyaw_vel_cmd = 0;    uint32_t last_hipyaw_ms = 0;
int16_t hippitch_vel_cmd = 0;  uint32_t last_hippitch_ms = 0;
int16_t knee_vel_cmd = 0;      uint32_t last_knee_ms = 0;

// For combining triggers into one toes command
float last_LT_norm = 0.0f;   // left trigger normalized 0..1
float last_RT_norm = 0.0f;   // right trigger normalized 0..1

// ================== Helpers ==================
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

// Normalize trigger value from [-1,1] (pygame) to [0,1]
static inline float normTrigger(float v) {
  float n = (v + 1.0f) * 0.5f;   // -1 -> 0, +1 -> 1
  if (n < 0) n = 0;
  if (n > 1) n = 1;
  return n;
}

// Clamp helper
static inline int16_t clampI16(int32_t v, int16_t lim) {
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return (int16_t)v;
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
      // "TRIGGER 2 <val>" (left trigger), "TRIGGER 5 <val>" (right trigger)
      // "STICK 0 <val>" (LX), "STICK 1 <val>" (LY), "STICK 4 <val>" (RY)
      // "BUTTON X" (stop all)
      // Serial.print("Received: "); Serial.println(line);

      // --- STOP ALL ---
      if (line == "BUTTON X") {
        stopAll();
        last_toes_ms = last_hipyaw_ms = last_hippitch_ms = last_knee_ms = now;
      }

      // --- TRIGGERS → TOES (variable speed, RT forward, LT reverse) ---
      else if (line.startsWith("TRIGGER ")) {
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);
        if (sp1 > 0 && sp2 > sp1) {
          int axis = line.substring(sp1 + 1, sp2).toInt();  // 2 or 5
          float val = line.substring(sp2 + 1).toFloat();    // -1..+1
          float n = normTrigger(val);

          if (axis == 2) {        // Left trigger
            last_LT_norm = n;
          } else if (axis == 5) { // Right trigger
            last_RT_norm = n;
          }

          // Net toes command: RT forward (+), LT reverse (-)
          float net = last_RT_norm - last_LT_norm;  // -1..+1
          int16_t v = clampI16((int32_t)(net * TOES_MAX_SPEED), TOES_MAX_SPEED);
          setVel(TOES, toes_vel_cmd, v);
          last_toes_ms = now;
        }
      }

      // --- STICKS → variable speeds ---
      else if (line.startsWith("STICK ")) {
        int sp1 = line.indexOf(' ');
        int sp2 = line.indexOf(' ', sp1 + 1);
        if (sp1 > 0 && sp2 > sp1) {
          int axis = line.substring(sp1 + 1, sp2).toInt();
          float val = line.substring(sp2 + 1).toFloat();  // -1..+1 from pygame

          // Left stick X (axis 0) → HIP_YAW
          if (axis == 0) {
            int16_t v = clampI16((int32_t)(val * HIP_YAW_MAX_SPEED), HIP_YAW_MAX_SPEED);
            setVel(HIP_YAW, hipyaw_vel_cmd, v);
            last_hipyaw_ms = now;
          }
          // Left stick Y (axis 1) → HIP_PITCH (invert so up is +; flip sign if you prefer)
          else if (axis == 1) {
            int16_t v = clampI16((int32_t)(-val * HIP_PITCH_MAX_SPEED), HIP_PITCH_MAX_SPEED);
            setVel(HIP_PITCH, hippitch_vel_cmd, v);
            last_hippitch_ms = now;
          }
          // Right stick Y (axis 4) → KNEE (invert so up is +)
          else if (axis == 4) {
            int16_t v = clampI16((int32_t)(-val * KNEE_MAX_SPEED), KNEE_MAX_SPEED);
            setVel(KNEE, knee_vel_cmd, v);
            last_knee_ms = now;
          }
          // Ignore other axes for now
        }
      }

      // Ignore other lines (buttons/D-pad/etc.) for this mapping.
    }
  }

  // --- Watchdogs: zero any channel that hasn't been refreshed recently ---
  if (now - last_toes_ms > WATCHDOG_MS)     setVel(TOES, toes_vel_cmd, 0);
  if (now - last_hipyaw_ms > WATCHDOG_MS)   setVel(HIP_YAW, hipyaw_vel_cmd, 0);
  if (now - last_hippitch_ms > WATCHDOG_MS) setVel(HIP_PITCH, hippitch_vel_cmd, 0);
  if (now - last_knee_ms > WATCHDOG_MS)     setVel(KNEE, knee_vel_cmd, 0);

  // No delay() — keep loop responsive
}
