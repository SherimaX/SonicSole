#ifndef SONICSOLE_H
#define SONICSOLE_H

// NPi Sensor Read and Data Logging

#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <iomanip>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <time.h>
#include <ctime>
#include <sstream>

// WiringPi libraries
#include <wiringPi.h>
#include <wiringSerial.h>
#include <wiringPiI2C.h>
#include <termios.h>
// #include "wiringSerial.h"

// User-defined libraries
#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.h"

// libraries from ADC code
#include <wiringPiSPI.h>

// libraries for UDP
#include <bits/stdc++.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// IMU API
// #include "IMUAPI/threespace_api_export.h"

#define MAX_RUN_TIME 600000000

using namespace std;

static int SPI_CHANNEL = 0;
static int CS = 17; 
static const int IMU_PACKET_LENGTH = 52;
// static const int IMU_PACKET_LENGTH = 28;


// mutex dataMutex[3];
// mutex writeMutex;
// condition_variable condWrite;

uint64_t getMicrosTimeStamp();
uint64_t getSecondsTimeStamp(); 

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr);

class SonicSole {
public:
    bool mode = true;
    int thresholdCounter = 0;
    int thresholdCross = 0;

    int heelSensorAddr = 224;
    int foreSensorAddr = 160;

    double minHeelPressure = 0;
    double minForePressure = 0;
    double minCombinedPressure = 0;

    double maxHeelPressure = 0;
    double maxForePressure = 0;
    double maxCombinedPressure = 0;

    double currHeelPressure = 0;
    double currForePressure = 0;
    double currCombinedPressure = 0;

    double prevHeelPressure = 0;
    double prevForePressure = 0;
    double prevCombinedPressure = 0;

    double ax = 0;
    double ay = 0;
    double az = 0;

    uint64_t startTime = 0;
    uint64_t currentTime = 0;
    // uint64_t startInterval = 0;
    // uint64_t endInterval = 0;
    
    uint8_t dataIMUPacket[IMU_PACKET_LENGTH];
    bool recordState = true;

    // tss_device_id sensor_id;

    //vector<int> timeArr;
    //int timeArr[200];

    SonicSole();
    ~SonicSole();
    void motorVibrate();
    // void detectModeChange();
    void runSoundMode();
    void runVibrateMode();

    // void toCSV();
    void toCSV(double time, double heelpresh, double forepresh, float az);
    void openCSVFile();
    void closeCSVFile();

    void readIMU();
    double getRunningTime();
    void updateCurrentTime();
    void updatePressure();
    int getSensorReadings(unsigned char signal);
    bool getMode();
    void switchMode();
    uint64_t getCurrentTime(); 
    int getCurrForePressure();
    int getCurrHeelPressure();
    void sendFlexSensorData(int flexSensorData);
    void sendFlexSensorData(int flexSensorData, int port);
    bool detectHeelThreshold();
    void updateThresholdCounter();

    // void getAccelVectorData(float ax, float ay, float az, vector<float>& axVector, vector<float>& ayVector, vector<float>& azVector);
    void getAccelVectorData(float az, vector<float>& azVector);
    float vectorIntegral(vector<float> v);

private: 
    double heelThresholdInterval = 0;
    double previousHeelThresholdTime = 0;
    double currentHeelThresholdTime = 0;
    unsigned char SPIbuff[3];

    // void updateHeelThresholdInterval();
    ofstream outFile;
    bool detectThreshold(int prevReading, int currReading, int minReading, int maxReading);
    bool detectCombinedThreshold();
    void playSound();
    // uint64_t getCurrentTime();
};

#endif //SONICSOLE_H
