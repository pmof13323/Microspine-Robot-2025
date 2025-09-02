#include <Dynamixel2Arduino.h>

// ===== OpenRB-150 setup =====
#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #error "This sketch is designed for OpenRB-150."
#endif

using namespace ControlTableItem;

const float    DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUD             = 57600;

// ================== Motor map & limits ==================
// IDs: 1..12 (1=Leg1 Yaw, 2=Leg1 Hip, 3=Leg1 Knee, 4..6=Leg2, 7..9=Leg3, 10..12=Leg4)
const uint8_t IDS[] = {1,2,3,4,5,6,7,8,9,10,11,12};
const size_t  N_ALL = sizeof(IDS)/sizeof(IDS[0]);

// Per-ID degree limits (yaw ±90°, pitch ±120°)
struct Limit { float lo; float hi; };
// index 0 unused so we can index by ID directly
Limit DEG_LIMIT[13] = {
  {0,0},          // 0 (unused)
  {-90,  90},     //  1: L1 yaw
  {-120, 120},    //  2: L1 hip
  {-120, 120},    //  3: L1 knee
  {-90,  90},     //  4: L2 yaw
  {-120, 120},    //  5: L2 hip
  {-120, 120},    //  6: L2 knee
  {-90,  90},     //  7: L3 yaw
  {-120, 120},    //  8: L3 hip
  {-120, 120},    //  9: L3 knee
  {-90,  90},     // 10: L4 yaw
  {-120, 120},    // 11: L4 hip
  {-120, 120}     // 12: L4 knee
};

// Compile-time direction flips: +1 normal, -1 flipped (index 0 unused)
constexpr int8_t DIR[13] = {
  0,
  +1,+1,-1,   //  1..3  (Leg 1)
  +1,+1,-1,   //  4..6  (Leg 2)
  +1,+1,-1,   //  7..9  (Leg 3)
  +1,+1,-1    // 10..12 (Leg 4)
};

// Map (leg 1..4, joint 1..3) -> motor ID 1..12
inline uint8_t idFor(uint8_t leg, uint8_t joint /*1=yaw,2=hip,3=knee*/) {
  return (uint8_t)((leg - 1U) * 3U + joint);
}

// X-series scale: 0..4095 ticks ≈ 360°; we treat 0° as tick 2048
const int32_t CENTER_TICK   = 2048;
const float   TICKS_PER_DEG = 4096.0f / 360.0f;

// Motion profile (raw units; fixed)
uint32_t PROFILE_VEL  = 80;
uint32_t PROFILE_ACC  = 20;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ================== Utility ==================
inline bool   idKnown(uint8_t id){ return (id >= 1 && id <= 12); }
inline float  clampf(float v, float lo, float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
inline int32_t round_to_i32(float t){ return (int32_t)(t + (t >= 0 ? 0.5f : -0.5f)); }

inline int32_t degToTickForId(uint8_t id, float user_deg) {
  float mech_deg = DIR[id] * user_deg; // flip sign at compile-time config
  return round_to_i32(CENTER_TICK + mech_deg * TICKS_PER_DEG);
}

void applyProfiles(uint8_t id){
  dxl.writeControlTableItem(PROFILE_VELOCITY,     id, PROFILE_VEL);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROFILE_ACC);
}

// ================== Leg motion ==================
void moveLeg(uint8_t leg, float theta_1, float theta_2, float theta_3){
  if(leg < 1 || leg > 4){
    DEBUG_SERIAL.println(F("⚠️  moveLeg: 'leg' must be 1..4"));
    return;
  }
  uint8_t id_yaw  = idFor(leg, 1);
  uint8_t id_hip  = idFor(leg, 2);
  uint8_t id_knee = idFor(leg, 3);

  float a_yaw  = clampf(theta_1, DEG_LIMIT[id_yaw].lo,  DEG_LIMIT[id_yaw].hi);
  float a_hip  = clampf(theta_2, DEG_LIMIT[id_hip].lo,  DEG_LIMIT[id_hip].hi);
  float a_knee = clampf(theta_3, DEG_LIMIT[id_knee].lo, DEG_LIMIT[id_knee].hi);

  dxl.setGoalPosition(id_yaw,  degToTickForId(id_yaw,  a_yaw));
  dxl.setGoalPosition(id_hip,  degToTickForId(id_hip,  a_hip));
  dxl.setGoalPosition(id_knee, degToTickForId(id_knee, a_knee));

  DEBUG_SERIAL.print(F("Leg ")); DEBUG_SERIAL.print(leg);
  DEBUG_SERIAL.print(F(" -> yaw="));  DEBUG_SERIAL.print(a_yaw, 2);
  DEBUG_SERIAL.print(F("°, hip="));   DEBUG_SERIAL.print(a_hip, 2);
  DEBUG_SERIAL.print(F("°, knee="));  DEBUG_SERIAL.print(a_knee, 2);
  DEBUG_SERIAL.println(F("°"));
}

void moveAllLegs(float theta_1, float theta_2, float theta_3){
  for(uint8_t leg = 1; leg <= 4; ++leg){
    uint8_t id_yaw  = idFor(leg, 1);
    uint8_t id_hip  = idFor(leg, 2);
    uint8_t id_knee = idFor(leg, 3);

    float a_yaw  = clampf(theta_1, DEG_LIMIT[id_yaw].lo,  DEG_LIMIT[id_yaw].hi);
    float a_hip  = clampf(theta_2, DEG_LIMIT[id_hip].lo,  DEG_LIMIT[id_hip].hi);
    float a_knee = clampf(theta_3, DEG_LIMIT[id_knee].lo, DEG_LIMIT[id_knee].hi);

    dxl.setGoalPosition(id_yaw,  degToTickForId(id_yaw,  a_yaw));
    dxl.setGoalPosition(id_hip,  degToTickForId(id_hip,  a_hip));
    dxl.setGoalPosition(id_knee, degToTickForId(id_knee, a_knee));
  }
  DEBUG_SERIAL.print(F("All legs -> yaw=")); DEBUG_SERIAL.print(theta_1, 2);
  DEBUG_SERIAL.print(F("°, hip="));          DEBUG_SERIAL.print(theta_2, 2);
  DEBUG_SERIAL.print(F("°, knee="));         DEBUG_SERIAL.print(theta_3, 2);
  DEBUG_SERIAL.println(F("°"));
}

// ================== Input helpers ==================
static String readLineFlexible(){
  String line = DEBUG_SERIAL.readStringUntil('\n');
  if(line.length()==0){
    String alt = DEBUG_SERIAL.readStringUntil('\r');
    if(alt.length()>0) line = alt;
  }
  line.trim();
  return line;
}
int tokenizeLine(String line, String outTok[], int maxTok){
  line.trim();
  line.replace(',', ' ');
  while(line.indexOf("  ") >= 0) line.replace("  ", " ");
  int n = 0, start = 0;
  while(n < maxTok){
    int sp = line.indexOf(' ', start);
    String part = (sp == -1) ? line.substring(start) : line.substring(start, sp);
    part.trim();
    if(part.length() > 0) outTok[n++] = part;
    if(sp == -1) break;
    start = sp + 1;
  }
  return n;
}
bool strIsInt(const String& s){
  if(s.length()==0) return false;
  int i = (s[0]=='-' || s[0]=='+') ? 1 : 0;
  for(; i < s.length(); ++i) if(!isDigit(s[i])) return false;
  return true;
}
bool strIsNumber(const String& s){ // int or float
  if(s.length()==0) return false;
  bool seenDot = false; int i = (s[0]=='-' || s[0]=='+') ? 1 : 0;
  for(; i < s.length(); ++i){
    char c = s[i];
    if(c == '.'){ if(seenDot) return false; seenDot = true; }
    else if(!isDigit(c)) return false;
  }
  return true;
}

// ================== Arduino lifecycle ==================
void setup(){
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.setTimeout(120);
  while(!DEBUG_SERIAL) {}

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Bring all 12 motors up in a known state
  for(size_t i=0;i<N_ALL;++i){
    uint8_t id = IDS[i];
    if(!dxl.ping(id)){
      DEBUG_SERIAL.print(F("ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(" not found."));
      continue;
    }
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    applyProfiles(id);
    dxl.torqueOn(id);
  }

  DEBUG_SERIAL.println(F("\nReady. Usage:"));
  DEBUG_SERIAL.println(F("  leg <n> <yaw_deg> <hip_deg> <knee_deg>    (n = 1..4)"));
  DEBUG_SERIAL.println(F("  leg a <yaw_deg> <hip_deg> <knee_deg>      (move ALL legs)"));
  DEBUG_SERIAL.println(F("  leg all <yaw_deg> <hip_deg> <knee_deg>    (alias)"));
}

void loop(){
  if(!DEBUG_SERIAL.available()) return;

  String line = readLineFlexible();
  if(line.length()==0) return;

  DEBUG_SERIAL.print(F("> "));
  DEBUG_SERIAL.println(line);

  String tok[8];
  int n = tokenizeLine(line, tok, 8);
  if(n == 0) return;

  String c0 = tok[0];
  c0.toLowerCase();

  // Command: leg <n|a|all> <theta1> <theta2> <theta3>
  if(c0 == "leg" && n >= 5 && strIsNumber(tok[2]) && strIsNumber(tok[3]) && strIsNumber(tok[4])){
    String tgt = tok[1]; tgt.toLowerCase();
    float q1 = tok[2].toFloat();
    float q2 = tok[3].toFloat();
    float q3 = tok[4].toFloat();

    if(tgt == "a" || tgt == "all"){
      moveAllLegs(q1, q2, q3);
      return;
    }
    if(strIsInt(tok[1])){
      int leg = tok[1].toInt();
      moveLeg((uint8_t)leg, q1, q2, q3);
      return;
    }
  }

  DEBUG_SERIAL.println(F("❓ Unknown. Try:"));
  DEBUG_SERIAL.println(F("  leg 2 30 -15 45"));
  DEBUG_SERIAL.println(F("  leg a 10 20 -90"));
}
