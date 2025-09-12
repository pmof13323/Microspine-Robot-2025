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
    SyncRead_info.is_info_changed = true;

    // Fill the members of structure to syncWrite using internal packet buffer
    SyncWrite_info.packet.p_buf = nullptr;
    SyncWrite_info.packet.is_completed = false;
    SyncWrite_info.addr = SW_START_ADDR;
    SyncWrite_info.addr_length = SW_ADDR_LEN;
    SyncWrite_info.p_xels = SyncWrite_data_info;
    SyncWrite_info.xel_count = 0;

    // Insert a initial Position to the syncWrite Packet
    for (int i = 0; i < num_IDs; i++) {
        SyncWrite_data[i].goal_position = des_positions[i];
    }

    for (int i = 0; i < num_IDs; i++) {
        SyncWrite_data_info[i].id = IDs[i];
        SyncWrite_data_info[i].p_data = (uint8_t *)&SyncWrite_data[i].goal_position;
        SyncWrite_info.xel_count++;
    }
    SyncWrite_info.is_info_changed = true;
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
int DXLInterface::disableDXLTorque(int ID) {
    dxl.torqueOff(ID);
}

int DXLInterface::setPosition(int ID, int position) {

    int found = 0;
    for (int i = 0; i < num_IDs; i++) {
        if (ID == IDs[i]) {
            des_positions[i] = position;
            SyncWrite_data[i].goal_position = position;
            found = 1;
        }
    }

    // Data has changed - update DXLs.
    SyncWrite_info.is_info_changed = true;

    // Return true if found and set correctly
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

    enableDXLTorque(ID);
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

int DXLInterface::writeDXLData() {

    // Update the SyncWrite packet status
    // TODO: Update this based on access?

    int try_count = 0;

    if (SyncWrite_info.is_info_changed == true) {
        // Build a syncWrite Packet and transmit to DYNAMIXEL
        if (dxl.syncWrite(&SyncWrite_info) == true) {
            if (DEBUG) {
                Serial.println("[SyncWrite] Success");
            }
            for (int i = 0; i < SyncWrite_info.xel_count; i++) {
                if (DEBUG) {
                    Serial.print("  ID: ");
                    Serial.print(SyncWrite_info.p_xels[i].id);
                    Serial.print("\t Goal Position: ");
                    Serial.println(SyncWrite_data[i].goal_position);
                }
            }
        } else {
            Serial.print("[SyncWrite] Fail, Lib error code: ");
            Serial.print(dxl.getLastLibErrCode());
        }
    }

    // delay(300);
}

int DXLInterface::updateDXLData() {}