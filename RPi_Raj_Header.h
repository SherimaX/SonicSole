// GLOBAL VARIABLES
#define N_STR 512
#define LENGTH_SINGLE_PACKET 57
#define LENGTH_BUFFER_BLOCK 40926 // 28500//4047
#define NUMBER_BUFFER_PACKET 25   // 718 //71

// UDP Parameters
#define PORT 25000
#define MAXLINE 1024


////////////////////////////////////////////////////
// DATA STRUCTURES

struct structStreamingTimingInformation
{
    unsigned int interval;
    unsigned int duration;
    unsigned int delay;
} sStreamingTime;

struct structComponentLinearAcceleration
{
    // Big Endian
    float az;
    float ay;
    float ax;
};

struct structComponentRawAcceleration
{
    // Big Endian
    float r_az;
    float r_ay;
    float r_ax;
};

struct structComponentQuaternion
{
    // Big Endian
    float qw;
    float qz;
    float qy;
    float qx;
};

struct structComponentRawGyro
{
    // Big Endian
    float gz;
    float gy;
    float gx;
};

struct structComponentRawMag
{
    // Big Endian
    float mz;
    float my;
    float mx;
};

struct structEulerAngles
{
    // Big Endian
    float roll;
    float yaw;
    float pitch;
};

struct structComponentSensorData
{
    // Big Endian
    float mx;
    float my;
    float mz;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;
};

struct structStreamingData
{
    // Big Endian
    structComponentRawAcceleration acc;
    structComponentRawGyro gyro;
    structComponentLinearAcceleration lAcc;
    structComponentQuaternion q;
};

union unionStreamingData
{
    structStreamingData sData;
    uint8_t vData[sizeof(structStreamingData)];
} uStreamingDataIMU;

inline void fillIMUData(uint8_t* pointer, uint8_t *dataIMUPacket, int x) {
    pointer[3] = dataIMUPacket[x];
    pointer[2] = dataIMUPacket[x + 1];
    pointer[1] = dataIMUPacket[x + 2];
    pointer[0] = dataIMUPacket[x + 3];
}


inline void reconstructIMUPacket(uint8_t *dataIMUPacket, structComponentQuaternion &structQuat, structComponentLinearAcceleration &structAcce, structComponentRawGyro &structGyro, structComponentRawAcceleration &structRAcc)
{
    uint8_t *pointer;
    
    fillIMUData((uint8_t *)&structQuat.qx, dataIMUPacket, 0);
    fillIMUData((uint8_t *)&structQuat.qy, dataIMUPacket, 4);
    fillIMUData((uint8_t *)&structQuat.qz, dataIMUPacket, 8);
    fillIMUData((uint8_t *)&structQuat.qw, dataIMUPacket, 12);

    fillIMUData((uint8_t *)&structAcce.ax, dataIMUPacket, 16);
    fillIMUData((uint8_t *)&structAcce.ay, dataIMUPacket, 20);
    fillIMUData((uint8_t *)&structAcce.az, dataIMUPacket, 24);

    fillIMUData((uint8_t *)&structGyro.gx, dataIMUPacket, 28);
    fillIMUData((uint8_t *)&structGyro.gy, dataIMUPacket, 32);
    fillIMUData((uint8_t *)&structGyro.gz, dataIMUPacket, 36);

    fillIMUData((uint8_t *)&structRAcc.r_ax, dataIMUPacket, 40);
    fillIMUData((uint8_t *)&structRAcc.r_ay, dataIMUPacket, 44);
    fillIMUData((uint8_t *)&structRAcc.r_az, dataIMUPacket, 48);
}

inline void constructLogPacket(uint8_t *dataPacket, uint32_t currentTime,
                               structComponentQuaternion structQuat,
                               structComponentLinearAcceleration structAcce,
                               structComponentRawGyro structGyro,
                               structComponentRawAcceleration structRAcc,
                               uint16_t p0, uint16_t p1, uint16_t p2, uint16_t p3,
                               uint16_t p4, uint16_t p5, uint16_t p6, uint16_t p7,
                               uint32_t syncTime, uint8_t syncTrigger)
{
    uint8_t *pointer;
    int16_t val;

    // HEADER
    ////////////////////////////////////////

    dataPacket[0] = 0x01;
    dataPacket[1] = 0x02;
    dataPacket[2] = 0x03;

    // TIMESTAMP
    ////////////////////////////////////////

    pointer = (uint8_t *)&currentTime;
    dataPacket[3] = pointer[3];
    dataPacket[4] = pointer[2];
    dataPacket[5] = pointer[1];
    dataPacket[6] = pointer[0];

    // QUATERNION
    ////////////////////////////////////////

    val = int16_t(structQuat.qw * 5000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[7] = pointer[1];
    dataPacket[8] = pointer[0];

    val = int16_t(structQuat.qx * 5000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[9] = pointer[1];
    dataPacket[10] = pointer[0];

    val = int16_t(structQuat.qy * 5000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[11] = pointer[1];
    dataPacket[12] = pointer[0];

    val = int16_t(structQuat.qz * 5000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[13] = pointer[1];
    dataPacket[14] = pointer[0];

    // LINEAR ACCELERATION
    ////////////////////////////////////////

    val = int16_t(structAcce.ax * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[15] = pointer[1];
    dataPacket[16] = pointer[0];

    val = int16_t(structAcce.ay * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[17] = pointer[1];
    dataPacket[18] = pointer[0];

    val = int16_t(structAcce.az * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[19] = pointer[1];
    dataPacket[20] = pointer[0];

    // ANGULAR VELOCITY
    ////////////////////////////////////////

    val = int16_t(structGyro.gx * 900.0f);
    pointer = (uint8_t *)&val;
    dataPacket[21] = pointer[1];
    dataPacket[22] = pointer[0];

    val = int16_t(structGyro.gy * 900.0f);
    pointer = (uint8_t *)&val;
    dataPacket[23] = pointer[1];
    dataPacket[24] = pointer[0];

    val = int16_t(structGyro.gz * 900.0f);
    pointer = (uint8_t *)&val;
    dataPacket[25] = pointer[1];
    dataPacket[26] = pointer[0];

    // ACCELERATION
    ////////////////////////////////////////

    val = int16_t(structRAcc.r_ax * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[27] = pointer[1];
    dataPacket[28] = pointer[0];

    val = int16_t(structRAcc.r_ay * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[29] = pointer[1];
    dataPacket[30] = pointer[0];

    val = int16_t(structRAcc.r_az * 1000.0f);
    pointer = (uint8_t *)&val;
    dataPacket[31] = pointer[1];
    dataPacket[32] = pointer[0];

    // PRESSURE SENSORS
    ////////////////////////////////////////

    pointer = (uint8_t *)&p0;
    dataPacket[33] = pointer[1];
    dataPacket[34] = pointer[0];

    pointer = (uint8_t *)&p1;
    dataPacket[35] = pointer[1];
    dataPacket[36] = pointer[0];

    pointer = (uint8_t *)&p2;
    dataPacket[37] = pointer[1];
    dataPacket[38] = pointer[0];

    pointer = (uint8_t *)&p3;
    dataPacket[39] = pointer[1];
    dataPacket[40] = pointer[0];

    pointer = (uint8_t *)&p4;
    dataPacket[41] = pointer[1];
    dataPacket[42] = pointer[0];

    pointer = (uint8_t *)&p5;
    dataPacket[43] = pointer[1];
    dataPacket[44] = pointer[0];

    pointer = (uint8_t *)&p6;
    dataPacket[45] = pointer[1];
    dataPacket[46] = pointer[0];

    pointer = (uint8_t *)&p7;
    dataPacket[47] = pointer[1];
    dataPacket[48] = pointer[0];

    // SYNC SIGNAL
    ////////////////////////////////////////

    dataPacket[49] = syncTrigger;

    pointer = (uint8_t *)&syncTime;
    dataPacket[50] = pointer[3];
    dataPacket[51] = pointer[2];
    dataPacket[52] = pointer[1];
    dataPacket[53] = pointer[0];

    // TAIL
    ////////////////////////////////////////

    dataPacket[54] = 0x04;
    dataPacket[55] = 0x05;
    dataPacket[56] = 0x06;
}

inline void reconstructBinaryPacketBinary_test(uint8_t *recvbuffer, float &ax, float &ay, float &az)
{
  uint8_t *pointer;

  pointer = (uint8_t *)&ax;
  pointer[3] = recvbuffer[0];
  pointer[2] = recvbuffer[1];
  pointer[1] = recvbuffer[2];
  pointer[0] = recvbuffer[3];

  pointer = (uint8_t *)&ay;
  pointer[3] = recvbuffer[4];
  pointer[2] = recvbuffer[5];
  pointer[1] = recvbuffer[6];
  pointer[0] = recvbuffer[7];

  pointer = (uint8_t *)&az;
  pointer[3] = recvbuffer[8];
  pointer[2] = recvbuffer[9];
  pointer[1] = recvbuffer[10];
  pointer[0] = recvbuffer[11];
}

uint8_t calcCRC256(uint8_t *dataPacket, uint8_t nByte)
{
  uint16_t checksum = 0;
  for (uint8_t i = 1; i < nByte; i++)
  {
    checksum += dataPacket[i];
  }
  return (checksum % 256);
}

void createLogPacket_v01(uint8_t *dataPacket, uint64_t currentTime)
{
    uint8_t *pointer;

    dataPacket[0] = 0x01;
    dataPacket[1] = 0x02;
    dataPacket[2] = 0x03;

    pointer = (uint8_t *)&currentTime;
    dataPacket[3] = pointer[7];
    dataPacket[4] = pointer[6];
    dataPacket[5] = pointer[5];
    dataPacket[6] = pointer[4];
    dataPacket[7] = pointer[3];
    dataPacket[8] = pointer[2];
    dataPacket[9] = pointer[1];
    dataPacket[10] = pointer[0];

    dataPacket[11] = 0x04;
    dataPacket[12] = 0x05;
    dataPacket[13] = 0x06;
}

////////////////////////////////////////////////////
// YEI FUNCTIONS

inline void YEIwriteCommandValue(int serial, uint8_t cmd, uint8_t value)
{
    YEIdataPacket[0] = START_NO_RESP_HEADER;
    YEIdataPacket[1] = cmd;
    YEIdataPacket[2] = value;
    YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
    write(serial, YEIdataPacket, 4);
    delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIwriteCommandNoDelay(int serial, uint8_t cmd)
{
    YEIdataPacket[0] = START_NO_RESP_HEADER;
    YEIdataPacket[1] = cmd;
    YEIdataPacket[2] = calcCRC256(YEIdataPacket, 2);
    write(serial, YEIdataPacket, 3);
}

inline void YEIwriteCommand(int serial, uint8_t cmd)
{
    YEIwriteCommandNoDelay(serial, cmd);
    delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIsettingsHeader(int serial)
{
    // Settings Header
    YEIdataPacket[0] = START_NO_RESP_HEADER;
    YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
    YEIdataPacket[2] = 0x00;
    YEIdataPacket[3] = 0x00;
    YEIdataPacket[4] = 0x00;
    YEIdataPacket[5] = 0x00;
    YEIdataPacket[6] = calcCRC256(YEIdataPacket, 6);
    write(serial, YEIdataPacket, 7);
    delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIsetStreamingMode(int serial, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4, uint8_t slot5, uint8_t slot6, uint8_t slot7, uint8_t slot8)
{
    // Setting Streaming Mode
    YEIdataPacket[0] = START_NO_RESP_HEADER;
    YEIdataPacket[1] = CMD_SET_STREAMING_SLOT;

    YEIdataPacket[2] = slot1; // 1st slot
    YEIdataPacket[3] = slot2; // 2nd slot
    YEIdataPacket[4] = slot3; // 3rd slot
    YEIdataPacket[5] = slot4; // 4th slot
    YEIdataPacket[6] = slot5; // 5th slot
    YEIdataPacket[7] = slot6; // 6th slot
    YEIdataPacket[8] = slot7; // 7th slot
    YEIdataPacket[9] = slot8; // 8th slot

    YEIdataPacket[10] = calcCRC256(YEIdataPacket, 10);
    write(serial, YEIdataPacket, 11);

    delay(YEI_DELAY_AFTER_COMMAND);
}

inline void YEIsetStreamingTime(int serial)
{
    // Set Streaming Time
    YEIdataPacket[0] = START_NO_RESP_HEADER;
    YEIdataPacket[1] = CMD_SET_STREAMING_TIMING;

    uint8_t *pointer = (uint8_t *)&sStreamingTime.interval;
    YEIdataPacket[2] = pointer[3];
    YEIdataPacket[3] = pointer[2];
    YEIdataPacket[4] = pointer[1];
    YEIdataPacket[5] = pointer[0];

    pointer = (uint8_t *)&sStreamingTime.duration;
    YEIdataPacket[6] = pointer[3];
    YEIdataPacket[7] = pointer[2];
    YEIdataPacket[8] = pointer[1];
    YEIdataPacket[9] = pointer[0];

    pointer = (uint8_t *)&sStreamingTime.delay;
    YEIdataPacket[10] = pointer[3];
    YEIdataPacket[11] = pointer[2];
    YEIdataPacket[12] = pointer[1];
    YEIdataPacket[13] = pointer[0];

    YEIdataPacket[14] = calcCRC256(YEIdataPacket, 14);
    write(serial, YEIdataPacket, 15);
}

// Polling mode
inline void YEIgetStreamingBatch(unionStreamingData &uStreamingDataIMU)
{
    int nPacketStreamingData = sizeof(uStreamingDataIMU);
    int bIMU = 0;

    YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);

    if (serialDataAvail(IMU))
    {
        read(IMU, &uStreamingDataIMU.vData, 26);
    }
}

