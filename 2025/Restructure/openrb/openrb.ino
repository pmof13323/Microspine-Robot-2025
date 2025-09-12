#include <Dynamixel2Arduino.h>
#include "DXLInterface.hpp"
#include "config.h"

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
#else
  #error "This sketch is designed for OpenRB-150."
#endif

using namespace ControlTableItem;

// USB to host
const uint32_t USB_BAUD = 115200;

// Dynamixel bus
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUD = 57600;

// Motion profiles
uint32_t PROFILE_VEL = 80, PROFILE_ACC = 20;
const int32_t GRIP_VEL_BASE = 320;

// Mapping helpers
inline uint8_t idFor(uint8_t leg, uint8_t joint){ return (leg-1)*3 + joint; }   // 1..12
inline uint8_t gripIdFor(uint8_t leg){ return 12 + leg; }                        // 13..16

inline float clampf(float v, float lo, float hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }
inline int32_t round_i32(float t){ return (int32_t)(t + (t>=0?0.5f:-0.5f)); }
inline int32_t degToTickForId(uint8_t id, float deg){ return round_i32(2048 + deg*4096.0f/360.0f); }

// --- Serial parsing ---
String readLine(){
    String line = DEBUG_SERIAL.readStringUntil('\n');
    if(line.length() == 0) line = DEBUG_SERIAL.readStringUntil('\r');
    line.trim();
    return line;
}

int tokenize(String s, String outTok[], int maxTok){
    s.replace('\t',' ');
    while(s.indexOf("  ")>=0) s.replace("  "," ");
    int n=0, start=0;
    while(n<maxTok){
        int sp = s.indexOf(' ', start);
        String part = (sp==-1)? s.substring(start) : s.substring(start, sp);
        part.trim();
        if(part.length()>0) outTok[n++] = part;
        if(sp==-1) break;
        start = sp + 1;
    }
    return n;
}

// --- DXLInterface instance ---
DXLInterface dxlInterface;

// --- Motion functions ---
void moveLeg(uint8_t leg, float q1, float q2, float q3){
    if(leg < 1 || leg > 4) return;
    dxlInterface.setPosition(idFor(leg,1), degToTickForId(idFor(leg,1), clampf(q1,-90,90)));
    dxlInterface.setPosition(idFor(leg,2), degToTickForId(idFor(leg,2), clampf(q2,-120,120)));
    dxlInterface.setPosition(idFor(leg,3), degToTickForId(idFor(leg,3), clampf(q3,-120,120)));
}

void driveGrip(uint8_t leg, float grip){
    if(leg < 1 || leg > 4) return;
    int32_t vel = (int32_t)(grip * GRIP_VEL_BASE); // adjust if needed
    dxlInterface.setVelocity_im(gripIdFor(leg), vel);
}

void setup(){
    DEBUG_SERIAL.begin(USB_BAUD);
    dxlInterface.init();
    dxlInterface.configureDXLBuffers();

    // Init all DXLs
    for(uint8_t id=1; id<=16; ++id){
        dxlInterface.registerDXL(id);
    }
    dxlInterface.initDXLs();

    // Enable torque on all leg motors (IDs 1..12)
    for (uint8_t id = 1; id <= 12; ++id) {
        dxlInterface.enableDXLTorque(id);
    }

    for (uint8_t id = 1; id <= 12; ++id) {
        dxlInterface.enableDXLTorque(id);
        dxlInterface.setDXLControlMode(id, OP_POSITION);
    }

    DEBUG_SERIAL.println(F("Ready"));
}

void loop(){
    if(!DEBUG_SERIAL.available()) return;

    String line = readLine();
    if(line.length() == 0) return;

    int bar = line.indexOf('|');
    if(bar >= 0) line = line.substring(0, bar);
    line.trim();

    String tok[32];  // large enough for SYNC 16 motors
    int n = tokenize(line, tok, 32);
    if(n < 2) return;

    tok[0].toLowerCase();

    if(tok[0] == "l"){  // single leg command
        if(n < 5) return;
        int leg = tok[1].toInt();
        float q1 = tok[2].toFloat();
        float q2 = tok[3].toFloat();
        float q3 = tok[4].toFloat();
        float grip = (n>=6)? tok[5].toFloat() : 0.0f;

        moveLeg((uint8_t)leg, q1, q2, q3);
        driveGrip((uint8_t)leg, grip);
        dxlInterface.writeDXLData();

    } else if(tok[0] == "sync"){  // multi-motor sync command
        if((n-1) % 2 != 0) return;  // must be pairs of id + pos

        for(int i=1; i<n; i+=2){
            int id = tok[i].toInt();
            int pos = tok[i+1].toInt();
            dxlInterface.setPosition(id, pos);
        }
        dxlInterface.writeDXLData();  // send all at once
    }
}
