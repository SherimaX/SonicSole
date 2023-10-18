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

#define MAX_RUN_TIME 600000000

using namespace std;

static int SPI_CHANNEL = 0;
static int CS = 17;

// mutex dataMutex[3];
// mutex writeMutex;
// condition_variable condWrite;

uint64_t getMicrosTimeStamp();

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr);

class SonicSole {
public:
    bool mode = true;
    int thresholdCounter = 0;

    double maxHeelPressure = 0;
    double minHeelPressure = 1000;

    double maxCombinedPressure = 0;
    double minCombinedPressure = 1000;

    double currHeelPressure = 0;
    double prevHeelPressure = 0;

    double currForePressure = 0;
    double prevForePressure = 0;

    double currCombinedPressure = 0;
    double prevCombinedPressure = 0;

    uint64_t startTime = 0;
    uint64_t currentTime = 0;

    SonicSole();
    void motorVibrate();
    void detectModeChange();
    void runSoundMode();
    void runVibrateMode();
    void toCSV();
    uint64_t getRunningTime();
    void updateCurrentTime();
    void updatePressure();
    int getSensorReadings(unsigned char signal);
    bool getMode();

private: 
    double heelThresholdInterval = 0;
    double previousHeelThresholdTime = 0;
    double currentHeelThresholdTime = 0;
    int thresholdCross = 0;
    unsigned char SPIbuff[3];

    void updateHeelThresholdInterval();
    void updateThresholdCounter();
    bool detectThreshold(int prevReading, int currReading, int minReading, int maxReading);
    void updateThresholdCounter();
    bool detectHeelThreshold();
    bool detectCombinedThreshold();
    void playSound();
    uint64_t getCurrentTime();
};

#endif //SONICSOLE_H
