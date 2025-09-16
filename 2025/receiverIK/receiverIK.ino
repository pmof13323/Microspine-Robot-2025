#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #error "This sketch is designed for OpenRB-150."
#endif

using namespace ControlTableItem;

// USB to host (Python/Serial Monitor)
const uint32_t USB_BAUD = 115200;
// Dynamixel bus baud (XL/XC default)
const float    DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUD             = 57600;

// -------- mapping helpers --------
inline uint8_t idFor(uint8_t leg, uint8_t joint){ return (uint8_t)((leg-1U)*3U + joint); }   // 1..12
inline uint8_t gripIdFor(uint8_t leg){ return (uint8_t)(12 + leg); }                         // 13..16

// Per-joint limits (index by ID; 0 unused)
struct Limit { float lo; float hi; };
Limit DEG_LIMIT[13] = {
  {0,0},
  {-90,90},{-120,120},{-120,120},
  {-90,90},{-120,120},{-120,120},
  {-90,90},{-120,120},{-120,120},
  {-90,90},{-120,120},{-120,120}
};

// Direction flips for joints (index by ID; +1 normal, -1 flipped)
constexpr int8_t DIR[13] = {
  0,
  +1,+1,-1,
  +1,+1,-1,
  +1,+1,-1,
  +1,+1,-1
};
// Optional flips for grips (IDs 13..16)
constexpr int8_t GRIP_DIR[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0, +1,+1,+1,+1};

// Position conversion
const int32_t CENTER_TICK   = 2048;
const float   TICKS_PER_DEG = 4096.0f/360.0f;
inline float  clampf(float v,float lo,float hi){ if(v<lo)return lo; if(v>hi)return hi; return v; }
inline int32_t round_i32(float t){ return (int32_t)(t + (t>=0?0.5f:-0.5f)); }
inline int32_t degToTickForId(uint8_t id,float user_deg){
  float mech_deg = DIR[id]*user_deg;
  return round_i32(CENTER_TICK + mech_deg*TICKS_PER_DEG);
}

// Motion profiles (for joints)
uint32_t PROFILE_VEL = 80, PROFILE_ACC = 20;
// Grip velocity scaling (no limits)
const int32_t GRIP_VEL_BASE = 320;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void applyProfiles(uint8_t id){
  dxl.writeControlTableItem(PROFILE_VELOCITY,     id, PROFILE_VEL);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROFILE_ACC);
}

// --- tiny parsing ---
static String readLine(){
  String line = DEBUG_SERIAL.readStringUntil('\n');
  if(line.length()==0) line = DEBUG_SERIAL.readStringUntil('\r');
  line.trim(); return line;
}
int tokenize(String s, String outTok[], int maxTok){
  s.replace('\t',' ');
  while(s.indexOf("  ")>=0) s.replace("  "," ");
  int n=0, start=0;
  while(n<maxTok){
    int sp = s.indexOf(' ', start);
    String part = (sp==-1)? s.substring(start) : s.substring(start, sp);
    part.trim();
    if(part.length()>0) outTok[n++]=part;
    if(sp==-1) break;
    start = sp+1;
  }
  return n;
}

// --- motion ---
void moveLeg(uint8_t leg, float q1, float q2, float q3){
  if(leg<1 || leg>4) return;
  uint8_t id_y = idFor(leg,1), id_h = idFor(leg,2), id_k = idFor(leg,3);
  float a_y = clampf(q1, DEG_LIMIT[id_y].lo,  DEG_LIMIT[id_y].hi);
  float a_h = clampf(q2, DEG_LIMIT[id_h].lo,  DEG_LIMIT[id_h].hi);
  float a_k = clampf(q3, DEG_LIMIT[id_k].lo,  DEG_LIMIT[id_k].hi);
  dxl.setGoalPosition(id_y, degToTickForId(id_y, a_y));
  dxl.setGoalPosition(id_h, degToTickForId(id_h, a_h));
  dxl.setGoalPosition(id_k, degToTickForId(id_k, a_k));
}
void driveGrip(uint8_t leg, float grip){
  if(leg<1 || leg>4) return;
  uint8_t id = gripIdFor(leg);
  int32_t vel = (int32_t)(GRIP_DIR[id] * grip * GRIP_VEL_BASE); // no clamp
  dxl.setGoalVelocity(id, vel);
}

void setup(){
  DEBUG_SERIAL.begin(USB_BAUD);
  DEBUG_SERIAL.setTimeout(150);
  // Do NOT block here; Python will open the port when ready.
  // while(!DEBUG_SERIAL) {}

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Bring joints up in position mode
  for(uint8_t id=1; id<=12; ++id){
    if(!dxl.ping(id)) continue;
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    applyProfiles(id);
    dxl.torqueOn(id);
  }
  // Bring grips up in velocity mode
  for(uint8_t id=13; id<=16; ++id){
    if(!dxl.ping(id)) continue;
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_VELOCITY);
    dxl.torqueOn(id);
  }

  DEBUG_SERIAL.println(F("Ready"));
}

void loop(){
  if(!DEBUG_SERIAL.available()) return;

  String line = readLine();
  if(line.length()==0) return;

  int bar = line.indexOf('|'); if(bar>=0) line = line.substring(0,bar);
  line.trim();

  String tok[8]; int n = tokenize(line, tok, 8);
  if(n < 5) return;                 // need at least: L leg q1 q2 q3
  tok[0].toLowerCase(); if(tok[0]!="l") return;

  int   leg  = tok[1].toInt();
  float q1   = tok[2].toFloat();
  float q2   = tok[3].toFloat();
  float q3   = tok[4].toFloat();
  float grip = (n>=6)? tok[5].toFloat() : 0.0f;

  moveLeg((uint8_t)leg, q1,q2,q3);
  driveGrip((uint8_t)leg, grip);
}
