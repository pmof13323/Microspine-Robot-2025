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

// ===== Compile-time direction flips =====
// +1 = normal, -1 = flipped (index 0 unused). Change here, then re-upload.
constexpr int8_t DIR[13] = {
  0,   // 0 (unused)
  +1,+1,-1,   //  1..3  (Leg 1)
  +1,+1,-1,   //  4..6  (Leg 2)
  +1,+1,-1,   //  7..9  (Leg 3)
  +1,+1,-1    // 10..12 (Leg 4)
};
// Example (flip Leg 2 hip & knee -> IDs 5 & 6):
// constexpr int8_t DIR[13] = {0, +1,+1,+1, +1,-1,-1, +1,+1,+1, +1,+1,+1};

// Map (leg 1..4, joint 1..3) -> motor ID 1..12
inline uint8_t idFor(uint8_t leg, uint8_t joint /*1=yaw,2=hip,3=knee*/) {
  return (uint8_t)((leg - 1U) * 3U + joint);
}

// X-series scale: 0..4095 ticks ≈ 360°; we treat 0° as tick 2048
const int32_t CENTER_TICK   = 2048;
const float   TICKS_PER_DEG = 4096.0f / 360.0f;

// Motion profile (raw units; tune to taste)
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
inline float ticksToDegForId(uint8_t id, int32_t tick) {
  float mech_deg = (tick - CENTER_TICK) / TICKS_PER_DEG;
  return DIR[id] * mech_deg; // report using user sign
}

void applyProfiles(uint8_t id){
  dxl.writeControlTableItem(PROFILE_VELOCITY,     id, PROFILE_VEL);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, id, PROFILE_ACC);
}

void printHelp() {
  DEBUG_SERIAL.println(F("\n=== Quadruped Zero + Leg Control (XC330, OpenRB-150) ==="));
  DEBUG_SERIAL.println(F("Calibration:"));
  DEBUG_SERIAL.println(F("  z <id>         -> set CURRENT pose of motor <id> as 0° (EEPROM)"));
  DEBUG_SERIAL.println(F("  z all          -> set CURRENT pose of all 1..12 as 0°"));
  DEBUG_SERIAL.println(F("  clear <id>     -> clear homing offset (factory zero)"));
  DEBUG_SERIAL.println(F("  clear all      -> clear all homing offsets"));
  DEBUG_SERIAL.println(F("Control:"));
  DEBUG_SERIAL.println(F("  leg <n> <y> <h> <k> -> move leg n (1..4) to yaw/hip/knee degrees"));
  DEBUG_SERIAL.println(F("  m <id> <deg>        -> move single motor by ID"));
  DEBUG_SERIAL.println(F("  <id> <deg>          -> shorthand for single motor"));
  DEBUG_SERIAL.println(F("  go <y> <h> <k>      -> alias: leg 1 <y> <h> <k>"));
  DEBUG_SERIAL.println(F("  gait [reps] [dwell_ms] -> run Leg 1 4-pose sequence (default reps=5, dwell=350ms)"));
  DEBUG_SERIAL.println(F("                           e.g. 'gait' or 'gait 8 250'"));
  DEBUG_SERIAL.println(F("Status & tuning:"));
  DEBUG_SERIAL.println(F("  show               -> print present deg & HOMING_OFFSET"));
  DEBUG_SERIAL.println(F("  vel <raw>          -> set PROFILE_VELOCITY"));
  DEBUG_SERIAL.println(F("  acc <raw>          -> set PROFILE_ACCELERATION"));
  DEBUG_SERIAL.println(F("  h                  -> help"));
}

// ================== Robust reboot & reacquire ==================
bool waitAfterRebootAndReacquire(uint8_t id, uint32_t total_ms = 1500){
  uint32_t start = millis();
  delay(600);  // bootloader window
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  const uint8_t MAX_TRIES = 8;
  for(uint8_t k=0;k<MAX_TRIES;++k){
    if(dxl.ping(id)) return true;
    delay(100 + 50*k);
    if(millis() - start > total_ms) break;
  }
  return false;
}
bool reenablePosModeAndTorque(uint8_t id){
  dxl.torqueOff(id);
  if(!dxl.setOperatingMode(id, OP_POSITION)) return false;
  applyProfiles(id);
  dxl.torqueOn(id);
  return true;
}

// ================== Zeroing (SIGN-CORRECTED) ==================
bool setZeroHere(uint8_t id){
  if(!dxl.ping(id)){
    DEBUG_SERIAL.print(F("⚠️  ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(" not responding before zeroing."));
    return false;
  }
  dxl.torqueOff(id);
  int32_t present_tick = dxl.getPresentPosition(id);

  // reported = raw + offset ; want reported = 2048 -> offset = 2048 - raw
  int32_t homing_offset = CENTER_TICK - present_tick;

  if(!dxl.writeControlTableItem(HOMING_OFFSET, id, homing_offset)){
    DEBUG_SERIAL.print(F("❌ Write HOMING_OFFSET failed on ID ")); DEBUG_SERIAL.println(id);
    return false;
  }
  dxl.reboot(id);

  if(!waitAfterRebootAndReacquire(id)){
    DEBUG_SERIAL.print(F("❌ ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(" did not respond after reboot."));
    return false;
  }
  if(!reenablePosModeAndTorque(id)){
    DEBUG_SERIAL.print(F("❌ Failed to re-enable pos mode/torque on ID ")); DEBUG_SERIAL.println(id);
    return false;
  }

  // Verify & one-shot correction if joint shifted
  int32_t tick_after = dxl.getPresentPosition(id);
  int32_t err_ticks  = tick_after - CENTER_TICK;
  if(abs(err_ticks) > 5){
    int32_t new_offset = homing_offset - err_ticks;
    dxl.torqueOff(id);
    dxl.writeControlTableItem(HOMING_OFFSET, id, new_offset);
    dxl.reboot(id);
    if(!waitAfterRebootAndReacquire(id) || !reenablePosModeAndTorque(id)){
      DEBUG_SERIAL.println(F("⚠️  Reacquire failed after correction write."));
      return false;
    }
  }

  int32_t final_off = dxl.readControlTableItem(HOMING_OFFSET, id);
  DEBUG_SERIAL.print(F("✅ ID ")); DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(F(": HOMING_OFFSET now "));
  DEBUG_SERIAL.print(final_off);
  DEBUG_SERIAL.print(F(" ticks ("));
  DEBUG_SERIAL.print(final_off / TICKS_PER_DEG, 2);
  DEBUG_SERIAL.println(F(" deg)"));
  return true;
}
bool clearZero(uint8_t id){
  if(!dxl.ping(id)){
    DEBUG_SERIAL.print(F("⚠️  ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(" not responding before clear."));
    return false;
  }
  dxl.torqueOff(id);
  if(!dxl.writeControlTableItem(HOMING_OFFSET, id, (int32_t)0)){
    DEBUG_SERIAL.print(F("⚠️  Failed to clear HOMING_OFFSET on ID ")); DEBUG_SERIAL.println(id);
    return false;
  }
  dxl.reboot(id);
  if(!waitAfterRebootAndReacquire(id)){
    DEBUG_SERIAL.print(F("❌ ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(" did not respond after reboot."));
    return false;
  }
  if(!reenablePosModeAndTorque(id)){
    DEBUG_SERIAL.print(F("⚠️  Failed to re-enable pos mode/torque on ID ")); DEBUG_SERIAL.println(id);
    return false;
  }
  DEBUG_SERIAL.print(F("✅ ID ")); DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.println(F(": homing offset cleared."));
  return true;
}

// ================== Motion ==================
void moveMotorToDeg(uint8_t id, float deg_in){
  if(!idKnown(id)){ DEBUG_SERIAL.println(F("⚠️  Invalid ID")); return; }
  float deg = clampf(deg_in, DEG_LIMIT[id].lo, DEG_LIMIT[id].hi);
  dxl.setGoalPosition(id, degToTickForId(id, deg));
  DEBUG_SERIAL.print(F("➡️  ID ")); DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(F(" -> ")); DEBUG_SERIAL.print(deg, 2); DEBUG_SERIAL.println(F(" deg"));
}

/**
 * moveLeg
 * @param leg     1..4
 * @param theta_1 hip yaw   (deg)
 * @param theta_2 hip pitch (deg)
 * @param theta_3 knee pitch(deg)
 */
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

// ================== Gait ==================
/**
 * runGait
 * Runs the requested pose sequence on one leg.
 * @param leg       Leg index (default 1)
 * @param reps      How many times to repeat the 4-pose block (default 5)
 * @param dwell_ms  Delay between poses in ms (default 350)
 */
void runGait(uint8_t leg = 1, int reps = 5, uint16_t dwell_ms = 350){
  if(leg < 1 || leg > 4){
    DEBUG_SERIAL.println(F("⚠️  runGait: 'leg' must be 1..4"));
    return;
  }
  if(reps < 1) reps = 1;

  DEBUG_SERIAL.print(F("▶️  Gait on leg ")); DEBUG_SERIAL.print(leg);
  DEBUG_SERIAL.print(F(", reps=")); DEBUG_SERIAL.print(reps);
  DEBUG_SERIAL.print(F(", dwell=")); DEBUG_SERIAL.print(dwell_ms);
  DEBUG_SERIAL.println(F(" ms"));

  for(int r = 0; r < reps; ++r){
    moveLeg(leg,  60,  30, -120); delay(dwell_ms);
    moveLeg(leg,  60, -30,  -60); delay(dwell_ms);
    moveLeg(leg, -60, -30,  -60); delay(dwell_ms);
    moveLeg(leg, -60,  30, -120); delay(dwell_ms);
  }
  DEBUG_SERIAL.println(F("✅ Gait complete."));
}

// ================== WALK sequence (your choreography goes here) ==================
/**
 * runWalk
 * Runs your custom multi-leg walking routine.
 * @param cycles    Number of times to loop your sequence (default 1)
 * @param dwell_ms  Delay between poses/steps (ms, default 300)
 *
 * HOW TO USE:
 *  - Replace the placeholder steps with your own calls to moveLeg(leg, yaw, hip, knee).
 *  - You can also call moveMotorToDeg(id, deg) for single joints if you prefer.
 *  - Use delay(dwell_ms) between steps or tune per-step delays yourself.
 */
void runWalk(int cycles = 1, uint16_t dwell_ms = 300){
  if(cycles < 1) cycles = 1;

  DEBUG_SERIAL.print(F("▶️  WALK starting, cycles="));

    // Ready Stance
    moveLeg(1, 0,  10, -100);
    moveLeg(2, 0,  10, -100);
    moveLeg(3, 0,  10, -100);
    moveLeg(4, 0,  10, -100);
    delay(2000);

  for(int c = 0; c < cycles; ++c){

    // Leg 1 move
    moveLeg(1, 10, 30, -120);
    delay(200);
    moveLeg(1, 20, 10, -100);
    delay(200);

    // Shift 1
    moveLeg(1, 15, 10, -100);
    moveLeg(2, -5, 10, -100);
    moveLeg(3, 5, 10, -100);
    moveLeg(4, 5, 10, -100);
    delay(200);
    
    // Leg 3 move
    moveLeg(3, -5, 30, -120);
    delay(200);
    moveLeg(3, -15, 10, -100);
    delay(200);

    // Shift 2
    moveLeg(1, 10, 10, -100);
    moveLeg(2, -10, 10, -100);
    moveLeg(3, -10, 10, -100);
    moveLeg(4, 10, 10, -100);

    // Leg 4 move
    moveLeg(4, 0, 30, -120);
    delay(200);
    moveLeg(4, -10, 10, -100);
    delay(200);

    // Shift 3
    moveLeg(1, 5, 10, -100);
    moveLeg(2, -15, 10, -100);
    moveLeg(3, -5, 10, -100);
    moveLeg(4, -5, 10, -100);
    delay(200);
    
    // Leg 2 move
    moveLeg(2, -5, 30, -120);
    delay(200);
    moveLeg(2, 5, 10, -100);
    delay(200);

    // Shift 4
    moveLeg(1, 0, 10, -100);
    moveLeg(2, 0, 10, -100);
    moveLeg(3, 0, 10, -100);
    moveLeg(4, 0, 10, -100);

    delay(dwell_ms);

  DEBUG_SERIAL.println(F("✅ WALK done."));

  }
  
}


void showAll(){
  for(size_t i=0;i<N_ALL;++i){
    uint8_t id = IDS[i];
    if(!dxl.ping(id)){ DEBUG_SERIAL.print(F("ID ")); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(F(": (no response)")); continue; }
    int32_t tick = dxl.getPresentPosition(id);
    float user_deg = ticksToDegForId(id, tick);           // respects DIR[]
    int32_t off  = dxl.readControlTableItem(HOMING_OFFSET, id);
    DEBUG_SERIAL.print(F("ID ")); DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(F(": present=")); DEBUG_SERIAL.print(user_deg, 2); DEBUG_SERIAL.print(F(" deg"));
    DEBUG_SERIAL.print(F(", HOMING_OFFSET=")); DEBUG_SERIAL.print(off); DEBUG_SERIAL.print(F(" ticks ("));
    DEBUG_SERIAL.print(off / TICKS_PER_DEG, 2); DEBUG_SERIAL.println(F(" deg mech)"));
    DEBUG_SERIAL.print(F("  walk [cycles] [dwell_ms] -> run your custom walking routine (default 1 cycle, 300 ms)"));

  }
}

// ================== Input helpers (no sscanf) ==================
static String readLineFlexible(){
  // Works with NL, CR, both, or no line ending (timeout)
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
  DEBUG_SERIAL.setTimeout(120); // snappy line reads even with "No line ending"
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

  printHelp();
  DEBUG_SERIAL.println(F("\nReady. Examples:"));
  DEBUG_SERIAL.println(F("  leg 2 30 -15 45"));
  DEBUG_SERIAL.println(F("  m 8 -20     (or)   8 -20"));
  DEBUG_SERIAL.println(F("  go 10 30 -60   (alias for leg 1)"));
  DEBUG_SERIAL.println(F("  gait            (Leg 1, 5 reps, 350 ms dwell)"));
  DEBUG_SERIAL.println(F("  show"));
}

void loop(){
  if(!DEBUG_SERIAL.available()) return;

  String line = readLineFlexible();
  if(line.length()==0) return;

  DEBUG_SERIAL.print(F("> "));
  DEBUG_SERIAL.println(line);

  String tok[10];
  int n = tokenizeLine(line, tok, 10);
  if(n == 0) return;

  String c0 = tok[0];
  c0.toLowerCase();

  // help / show
  if(c0 == "h")    { printHelp(); return; }
  if(c0 == "show") { showAll();  return; }

  // velocity / acceleration
  if(c0 == "vel" && n >= 2 && strIsInt(tok[1])){
    PROFILE_VEL = (uint32_t)tok[1].toInt();
    for(size_t i=0;i<N_ALL;i++) applyProfiles(IDS[i]);
    DEBUG_SERIAL.print(F("✓ PROFILE_VELOCITY = ")); DEBUG_SERIAL.println(PROFILE_VEL);
    return;
  }
  if(c0 == "acc" && n >= 2 && strIsInt(tok[1])){
    PROFILE_ACC = (uint32_t)tok[1].toInt();
    for(size_t i=0;i<N_ALL;i++) applyProfiles(IDS[i]);
    DEBUG_SERIAL.print(F("✓ PROFILE_ACCELERATION = ")); DEBUG_SERIAL.println(PROFILE_ACC);
    return;
  }

  // gait [reps] [dwell_ms]   (Leg 1 only per request)
  if(c0 == "gait"){
    int reps = 5;
    uint16_t dwell = 350;

    if(n >= 2 && strIsInt(tok[1])) reps = tok[1].toInt();
    if(n >= 3 && strIsInt(tok[2])) {
      long d = tok[2].toInt();
      if(d < 0) d = 0;
      if(d > 5000) d = 5000; // clamp
      dwell = (uint16_t)d;
    }

    runGait(1, reps, dwell);
    return;
  }

  // walk [cycles] [dwell_ms]
  if(c0 == "walk"){
    int cycles = 1;
    uint16_t dwell = 300;

    if(n >= 2 && strIsInt(tok[1])) cycles = tok[1].toInt();
    if(n >= 3 && strIsInt(tok[2])) {
      long d = tok[2].toInt();
      if(d < 0) d = 0;
      if(d > 5000) d = 5000; // safety clamp
      dwell = (uint16_t)d;
    }

    runWalk(cycles, dwell);
    return;
  }


  // leg <n> <theta1> <theta2> <theta3>
  if(c0 == "leg" && n >= 5 && strIsInt(tok[1]) && strIsNumber(tok[2]) && strIsNumber(tok[3]) && strIsNumber(tok[4])){
    int leg = tok[1].toInt();
    moveLeg((uint8_t)leg, tok[2].toFloat(), tok[3].toFloat(), tok[4].toFloat());
    return;
  }

  // go <y> <h> <k>  (alias for leg 1)
  if(c0 == "go" && n >= 4 && strIsNumber(tok[1]) && strIsNumber(tok[2]) && strIsNumber(tok[3])){
    moveLeg(1, tok[1].toFloat(), tok[2].toFloat(), tok[3].toFloat());
    return;
  }

  // m <id> <deg>
  if(c0 == "m" && n >= 3 && strIsInt(tok[1]) && strIsNumber(tok[2])){
    int id = tok[1].toInt();
    if(idKnown((uint8_t)id)) moveMotorToDeg((uint8_t)id, tok[2].toFloat());
    else DEBUG_SERIAL.println(F("⚠️  Unknown ID."));
    return;
  }

  // "<id> <deg>" shorthand
  if(n >= 2 && strIsInt(tok[0]) && strIsNumber(tok[1])){
    int id = tok[0].toInt();
    if(idKnown((uint8_t)id)) moveMotorToDeg((uint8_t)id, tok[1].toFloat());
    else DEBUG_SERIAL.println(F("⚠️  Unknown ID."));
    return;
  }

  // z <id> / z all
  if(c0 == "z" && n >= 2){
    String a1 = tok[1]; a1.toLowerCase();
    if(a1 == "all"){ for(size_t i=0;i<N_ALL;i++) setZeroHere(IDS[i]); return; }
    if(strIsInt(tok[1])){
      int id = tok[1].toInt();
      if(idKnown((uint8_t)id)) setZeroHere((uint8_t)id);
      else DEBUG_SERIAL.println(F("⚠️  Unknown ID."));
      return;
    }
  }

  // clear <id> / clear all
  if(c0 == "clear" && n >= 2){
    String a1 = tok[1]; a1.toLowerCase();
    if(a1 == "all"){ for(size_t i=0;i<N_ALL;i++) clearZero(IDS[i]); return; }
    if(strIsInt(tok[1])){
      int id = tok[1].toInt();
      if(idKnown((uint8_t)id)) clearZero((uint8_t)id);
      else DEBUG_SERIAL.println(F("⚠️  Unknown ID."));
      return;
    }
  }

  DEBUG_SERIAL.println(F("❓ Unrecognized input. Try: leg 2 30 -15 45  |  m 8 -20  |  1 -20  |  go 10 30 -60  |  gait  |  show"));
}
