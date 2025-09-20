#ifndef DXLINTERFACE_H
#define DXLINTERFACE_H

#include <Dynamixel2Arduino.h>

#include "config.h"

#define OPERATION_MODE 9

const uint16_t BUF_SIZE = 256;


// struct BulkReadDataStructure {
//     int16_t present_load;
//     int32_t present_velocity;
//     int32_t present_position;
// } __attribute__((packed));

// struct BulkWriteDataStructure {
//     int32_t goal_position;
// } __attribute__((packed));

// BulkWriteDataStructure bw_data_xel_1;
// BwDataXel2 bw_data_xel_2;
// DYNAMIXEL::InfoBulkWriteInst_t bulk_write_info;
// DYNAMIXEL::XELInfoBulkWrite_t bulk_write_data[NUM_DXLS];

const uint16_t SR_START_ADDR = 126;     // Present Load
const uint16_t SR_ADDR_LEN = 2 + 4 + 4; // Load + Vel + Pos
const uint16_t SW_VEL_START_ADDR = 104; // goal velocity
const uint16_t SW_POS_START_ADDR = 116; //goal postion
const uint16_t SW_ADDR_LEN = 4;

typedef struct SyncReadDataStruct {
    int16_t present_load;
    int32_t present_velocity;
    int32_t present_position;
} __attribute__((packed)) SyncReadData_t;

typedef struct {
    int32_t goal_velocity;  // For sync write at 104
} __attribute__((packed)) SyncWriteVelocity_t;

typedef struct {
    int32_t goal_position;  // For sync write at 116
} __attribute__((packed)) SyncWritePosition_t;


class DXLInterface {

    Dynamixel2Arduino dxl;

    uint8_t IDs[NUM_DXLs];

    uint8_t num_IDs;

    double positions[NUM_DXLs];
    double velocities[NUM_DXLs];
    double loads[NUM_DXLs];

    int32_t des_positions[NUM_DXLs];
    double des_velocities[NUM_DXLs];
    double des_torques[NUM_DXLs];

    // DXL Sync Read/Write
    SyncReadData_t SyncRead_data[NUM_DXLs];
    DYNAMIXEL::InfoSyncReadInst_t SyncRead_info;
    DYNAMIXEL::XELInfoSyncRead_t SyncRead_data_info[NUM_DXLs];

    SyncWriteVelocity_t SyncWriteVel_data[NUM_DXLs];
    DYNAMIXEL::InfoSyncWriteInst_t SyncWriteVel_info;
    DYNAMIXEL::XELInfoSyncWrite_t SyncWriteVel_data_info[NUM_DXLs];

    SyncWritePosition_t SyncWritePos_data[NUM_DXLs];
    DYNAMIXEL::InfoSyncWriteInst_t SyncWritePos_info;
    DYNAMIXEL::XELInfoSyncWrite_t SyncWritePos_data_info[NUM_DXLs];

    uint8_t user_pkt_buf[BUF_SIZE];

public:
    DXLInterface();

    // Init must be called to initialise connection to Dynamixels
    int init();
    // Configure buffers must be called (only once) before bulk/sync operations
    int configureDXLBuffers();

    // Add DXL to Group
    int registerDXL(int ID);
    int deregisterDXL(int ID);

    int initDXLs();

    // Group Methods
    // - Immediate Mode
    int enableDXLTorque(int ID);
    int enableDXLTorqueNoAcceleration(int ID);
    int disableDXLTorque(int ID);

    int setDXLControlMode(int ID, int mode);

    int setVelocity_im(int ID, int velocity);

    // - Sync Mode
    int setPosition(int ID, int position);
    int setVelocity(int ID, int position);

    // Individual Methods
    int getPosition(int ID);
    int getVelocity(int ID);
    int getLoad(int ID);

    // Bulk RW Methods
    int readDXLData();
    int writeDXLData();
    String getReadData();

    const char* opModeName(int8_t mode);
    void printMotorOpMode(uint8_t motorID);

    int updateDXLData();
};



#endif // !DXLINTERFACE_H