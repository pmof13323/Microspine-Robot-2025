#include "DXLInterface.hpp"

#include <Dynamixel2Arduino.h>
#include <actuator.h>

DXLInterface::DXLInterface()
    : dxl(DXL_SERIAL, -1) {

    for (int i = 0; i < NUM_DXLs; i++) {
        IDs[i] = -1;
    }

    num_IDs = 0;
}

int DXLInterface::init() {

    // Start Interface
    const uint8_t DXL_ID = 1;
    const float DXL_PROTOCOL_VERSION = 2.0;

    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
    dxl.begin(1000000);

    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    return 1;
}

int DXLInterface::configureDXLBuffers() {

    // Fill the members of structure to fastSyncRead using external user packet buffer
    SyncRead_info.packet.buf_capacity = BUF_SIZE;
    SyncRead_info.packet.p_buf = user_pkt_buf;
    SyncRead_info.packet.is_completed = false;
    SyncRead_info.addr = SR_START_ADDR;
    SyncRead_info.addr_length = SR_ADDR_LEN;
    SyncRead_info.p_xels = SyncRead_data_info;
    SyncRead_info.xel_count = 0;

    for (int i = 0; i < num_IDs; i++) {
        SyncRead_data_info[i].id = IDs[i];
        SyncRead_data_info[i].p_recv_buf = (uint8_t *)&SyncRead_data[i];
        SyncRead_info.xel_count++;
    }

    // --- Sync Write: Goal Position ---
    SyncWritePos_info.packet.p_buf = nullptr;  // internal buffer
    SyncWritePos_info.packet.is_completed = false;
    SyncWritePos_info.addr = 116; // Goal Position start address
    SyncWritePos_info.addr_length = sizeof(int32_t);
    SyncWritePos_info.p_xels = SyncWritePos_data_info;
    SyncWritePos_info.xel_count = 0;

    for (int i = 0; i < num_IDs; i++) {
        SyncWritePos_data[i].goal_position = des_positions[i]; // initialise
        SyncWritePos_data_info[i].id = IDs[i];
        SyncWritePos_data_info[i].p_data = (uint8_t *)&SyncWritePos_data[i];
        SyncWritePos_info.xel_count++;
    }
    SyncWritePos_info.is_info_changed = true;

    // --- Sync Write: Goal Velocity ---
    SyncWriteVel_info.packet.p_buf = nullptr;  // internal buffer
    SyncWriteVel_info.packet.is_completed = false;
    SyncWriteVel_info.addr = 104; // Goal Velocity start address
    SyncWriteVel_info.addr_length = sizeof(int32_t);
    SyncWriteVel_info.p_xels = SyncWriteVel_data_info;
    SyncWriteVel_info.xel_count = 0;

    for (int i = 0; i < num_IDs; i++) {
        SyncWriteVel_data[i].goal_velocity = 0; // start with zero velocity
        SyncWriteVel_data_info[i].id = IDs[i];
        SyncWriteVel_data_info[i].p_data = (uint8_t *)&SyncWriteVel_data[i];
        SyncWriteVel_info.xel_count++;
    }
    SyncWriteVel_info.is_info_changed = true;

    return 1; // success
}

int DXLInterface::registerDXL(int ID) {

    int ID_in_arr = 0;

    if (num_IDs > NUM_DXLs - 1) {
        return 0;
    }

    // Check if ID exists in array
    for (int i = 0; i < NUM_DXLs; i++) {
        if (IDs[i] == ID) {
            ID_in_arr = 1;
        }
    }

    if (!ID_in_arr) {
        IDs[num_IDs] = ID;
    }

    num_IDs++;

    return 1;
}

int DXLInterface::deregisterDXL(int ID) {
    int ID_found = 0;
    int found_index = -1;

    // Check if ID exists in array and find its index
    for (int i = 0; i < num_IDs; i++) {
        if (IDs[i] == ID) {
            ID_found = 1;
            found_index = i;
            break;
        }
    }

    // If ID not found, return 0 (failure)
    if (!ID_found) {
        return 0;
    }

    // Shift all elements after the found index to the left
    for (int i = found_index; i < num_IDs - 1; i++) {
        IDs[i] = IDs[i + 1];
    }

    // Decrease the count of IDs
    num_IDs--;

    return 1; // Success
}

int DXLInterface::initDXLs() {
}

// Group Methods
int DXLInterface::enableDXLTorque(int ID) {
    dxl.torqueOn(ID);

    int result = dxl.writeControlTableItem(0x51, ID, V_MAX);
    result = dxl.writeControlTableItem(0x50, ID, A_MAX);
    
    if (DEBUG) {
        Serial.println(dxl.readControlTableItem(0x51, ID));
        Serial.println(dxl.readControlTableItem(0x50, ID));
    }
}

int DXLInterface::enableDXLTorqueNoAcceleration(int ID) {
    dxl.torqueOn(ID);

    return 1;
}

int DXLInterface::disableDXLTorque(int ID) {
    dxl.torqueOff(ID);
}

int DXLInterface::setPosition(int ID, int position) {
    int found = 0;
    for (int i = 0; i < num_IDs; i++) {
        if (ID == IDs[i]) {
            des_positions[i] = position;
            SyncWritePos_data[i].goal_position = position;
            found = 1;
        }
    }

    // Mark position sync-write group as changed
    if (found) {
        SyncWritePos_info.is_info_changed = true;
    }

    return found;
}

int DXLInterface::setVelocity(int ID, int velocity) {
    int found = 0;
    for (int i = 0; i < num_IDs; i++) {
        if (ID == IDs[i]) {
            //SyncWriteVel_data[i].goal_velocity = velocity;

            //des_velocities[i] = velocity;

            // Convert velocity request to raw Dynamixel value
            if (velocity == 1) {
                SyncWriteVel_data[i].goal_velocity = 2047;
            } else if (velocity == -1) {
                SyncWriteVel_data[i].goal_velocity = -2047;
            } else {
                SyncWriteVel_data[i].goal_velocity = 0;
            }

            found = 1;
        }
    }

    // Mark velocity sync-write group as changed
    if (found) {
        SyncWriteVel_info.is_info_changed = true;
    }

    return found;
}

int DXLInterface::setVelocity_im(int ID, int velocity) {

    dxl.setGoalVelocity(ID, velocity);
}

// mode must be Dynamixel enum or int conversion
int DXLInterface::setDXLControlMode(int ID, int mode) {

    disableDXLTorque(ID);

    if (DEBUG) {
        Serial.print("Setting Mode: ");
        Serial.println(mode);
    }

    if (mode == OP_VELOCITY) {

        int result = dxl.writeControlTableItem(0x51, ID, V_MAX);

    } else if (mode == OP_EXTENDED_POSITION) {

        // TODO: Check why the macros arent working.
        int result = dxl.writeControlTableItem(0x51, ID, V_MAX);
        // int result = dxl.writeControlTableItem(PROFILE_VELOCITY, ID, V_MAX);

        // result = dxl.writeControlTableItem(PROFILE_ACCELERATION, ID, A_MAX);
        result = dxl.writeControlTableItem(0x50, ID, A_MAX);
    }

    int result = dxl.setOperatingMode(ID, mode);

    if (DEBUG) {
        Serial.print("Set Op Mode: ");
        Serial.println(result);
    }

    if (mode == OP_VELOCITY){
        enableDXLTorqueNoAcceleration(ID);
    } else {
        enableDXLTorque(ID);
    }
}

int DXLInterface::getPosition(int ID) {

    for (int j = 0; j < num_IDs; j++) {
        if (ID == IDs[j]) {
            // Populate arrays
            return positions[j];
        }
    }
}

int DXLInterface::getVelocity(int ID) {

    for (int j = 0; j < num_IDs; j++) {
        if (ID == IDs[j]) {
            // Populate arrays
            return velocities[j];
        }
    }
}

int DXLInterface::getLoad(int ID) {

    for (int j = 0; j < num_IDs; j++) {
        if (ID == IDs[j]) {
            // Populate arrays
            return loads[j];
        }
    }
}

// Sync RW Methods
int DXLInterface::readDXLData() {

    if (DEBUG) {
        Serial.print("DXL Read IDs: [");
        for (int i = 0; i < num_IDs; i++) {

            int ID = SyncRead_info.p_xels[i].id;
            Serial.print(ID);
            Serial.print(", ");
        }
        Serial.println("]");
    }

    // Transmit predefined fastSyncRead instruction packet
    // and receive a status packet from each DYNAMIXEL
    int recv_cnt = dxl.fastSyncRead(&SyncRead_info);
    if (recv_cnt > 0) {
        if (DEBUG) {
            Serial.print("[fastSyncRead] Success, Received ID Count: ");
            Serial.println(recv_cnt);
        }
        for (int i = 0; i < recv_cnt; i++) {

            int ID = SyncRead_info.p_xels[i].id;

            if (DEBUG) {
                Serial.print("  ID: ");
                Serial.print(ID);
                Serial.print(":");
                Serial.print(SyncRead_data[i].present_load);
                Serial.print(":");
                Serial.print(SyncRead_data[i].present_velocity);
                Serial.print(":");
                Serial.println(SyncRead_data[i].present_position);
            }

            for (int j = 0; j < num_IDs; j++) {
                if (ID == IDs[j]) {
                    // Populate arrays
                    // TODO: Do we need to do this here? or save it for lookups?
                    loads[j] = SyncRead_data[i].present_load;
                    velocities[j] = SyncRead_data[i].present_velocity;
                    positions[j] = SyncRead_data[i].present_position;
                }
            }
        }
    } else {
        Serial.print("[fastSyncRead] Fail, Lib error code: ");
        Serial.println(dxl.getLastLibErrCode());
    }
}

String DXLInterface::getReadData(){
    String line = "READ ";

    for (int i = 0; i < num_IDs; i++) {
        line += String(IDs[i]);
        line += ",";
        line += String(positions[i]);
        line += ",";
        line += String(velocities[i]);
        line += ",";
        line += String(loads[i]);
        if (i < num_IDs - 1) line += ";";
    }

    return line;
}


int DXLInterface::writeDXLData() {
    int try_count = 0;

    // --- Write Positions ---
    if (SyncWritePos_info.is_info_changed) {
        if (dxl.syncWrite(&SyncWritePos_info)) {
            if (DEBUG) {
                Serial.println("[SyncWrite] Positions Success");
                Serial.print("Count: ");
                Serial.println(SyncWritePos_info.xel_count);
                for (int i = 0; i < SyncWritePos_info.xel_count; i++) {
                    Serial.print("  ID: ");
                    Serial.print(SyncWritePos_info.p_xels[i].id);
                    Serial.print("\t Goal Position: ");
                    Serial.println(SyncWritePos_data[i].goal_position);
                }
            }
        } else {
            Serial.print("[SyncWrite] Positions Fail, Lib error code: ");
            Serial.println(dxl.getLastLibErrCode());
        }
        SyncWritePos_info.is_info_changed = false; // reset flag
    }

    // --- Write Velocities ---
    if (SyncWriteVel_info.is_info_changed) {
        if (dxl.syncWrite(&SyncWriteVel_info)) {
            if (DEBUG) {
                Serial.println("[SyncWrite] Velocities Success");
                Serial.print("Count: ");
                Serial.println(SyncWriteVel_info.xel_count);
                for (int i = 0; i < SyncWriteVel_info.xel_count; i++) {
                    Serial.print("  ID: ");
                    Serial.print(SyncWriteVel_info.p_xels[i].id);
                    Serial.print("\t Goal Velocity: ");
                    Serial.println(SyncWriteVel_data[i].goal_velocity);
                    Serial.print("\t Goal Operation Mode: ");
                    Serial.print(dxl.readControlTableItem(OPERATION_MODE, SyncWriteVel_info.p_xels[i].id));
                    Serial.print("\t ENABLE: ");
                    Serial.print(dxl.readControlTableItem(34, SyncWriteVel_info.p_xels[i].id));
                    Serial.print("\t Read Velocity: ");
                    Serial.print(dxl.readControlTableItem(59, SyncWriteVel_info.p_xels[i].id));
                    Serial.print('\n');
                }
            }
        } else {
            Serial.print("[SyncWrite] Velocities Fail, Lib error code: ");
            Serial.println(dxl.getLastLibErrCode());
        }
        SyncWriteVel_info.is_info_changed = false; // reset flag
    }

    return 1; // success
}


int DXLInterface::updateDXLData() {}


// Function to print operating mode of a single motor
void DXLInterface::printMotorOpMode(uint8_t motorID) {
    int8_t mode = dxl.readControlTableItem(OPERATION_MODE, motorID);
    Serial.print("Motor ID ");
    Serial.print(motorID);
    Serial.print(" Operating Mode: ");
    Serial.println((mode));
}