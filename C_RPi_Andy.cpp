// NPi Sensor Read and Data Logging

#include <iostream>
// #include <fstream>
// #include <mutex>
// #include <thread>
// #include <condition_variable>
// #include <chrono>
// #include <iomanip>

// #include <stdio.h>
// #include <stdlib.h>
// #include <stdint.h>
// #include <string.h>
// #include <fcntl.h>
// #include <unistd.h>
// #include <errno.h>
// #include <inttypes.h>
// #include <sys/time.h>
// #include <sys/ioctl.h>
// #include <linux/i2c-dev.h>
// #include <time.h>

// // WiringPi libraries
// #include <wiringPi.h>
// #include <wiringSerial.h>
// #include <wiringPiI2C.h>
// #include <termios.h>
// #include "wiringSerial.h"

// User-defined libraries
#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.hpp"

// // libraries from ADC code
// #include <wiringPiSPI.h>

// // libraries for UDP
// #include <bits/stdc++.h>
// #include <string.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>

#define MAX_RUN_TIME 600000000

#include "SonicSole.h"


using namespace std;

// // channel is the wiringPi name for the chip select (or chip enable) pin.
// // Set this to 0 or 1, depending on how it's connected.
// static int SPI_CHANNEL = 0;


// static int CS = 17;

// //int start_up = 1;

// // mutex dataMutex[2];
// mutex dataMutex[3];
// mutex writeMutex;
// condition_variable condWrite;


// uint64_t getMicrosTimeStamp() 
// {
// 	struct timeval tv;
// 	gettimeofday(&tv,NULL);
// 	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
// }

// // UDP Send

// void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
//     sendto(sockfd, (const int *)reading, len,
//            MSG_CONFIRM, (const struct sockaddr *) &servaddr,
//            sizeof(servaddr));
// }


// class SonicSole {       // The class
// public:             // Access specifier
//     SonicSole() {
//         startTime = getMicrosTimeStamp();
//         wiringPiSetupGpio() ;
//         pinMode(CS, OUTPUT) ;
//         digitalWrite(CS,HIGH);
//         cout << "Initializing SPI" << endl ;

//         // INITIALIZING UART1
//         printf("Initializing UART0...\n\n");

//         // INITIALIZING GPIO (Use wPi pins, not BCM)
//         printf("Initializing GPIO...\n\n");
//         if (wiringPiSetupGpio () == -1)
//         {
//             fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
//         }
//         printf("GPIO initialized successfully!\n\n");


//         int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
//         if (fd == -1)
//         {
//           std::cout << "Failed to init SPI communication.\n";
//         }
//         std::cout << "SPI communication successfully setup.\n";

//         this_thread::sleep_for(chrono::milliseconds(500));
//     }

    // bool mode = true;
    // double maxHeelPressure = 0;
    // double minHeelPressure = 1000;

    // double maxCombinedPressure = 0;
    // double minCombinedPressure = 1000;

    // double currHeelPressure = 0;
    // double prevHeelPressure = 0;

    // double currForePressure = 0;
    // double prevForePressure = 0;

    // double currCombinedPressure = 0;
    // double prevCombinedPressure = 0;

    // uint64_t startTime = 0;
    // uint64_t currentTime = 0;


//     void motorVibrate() {
//         digitalWrite(23, HIGH); //Turn motors on and off to show device is on
//         digitalWrite(20, HIGH);
//         delay(1000);
//         digitalWrite(23, LOW);
//         digitalWrite(20, LOW);
//     }
//     void detectModeChange() {
//         if (detectHeelThreshold() && heelThresholdInterval < 1)
//             mode = !mode;
//             string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
//     }
//     void runSoundMode() {
//         if (detectCombinedThreshold()) {
//             playSound();
//         }
//     }

//     void runVibrateMode() {
//         if (detectCombinedThreshold()) {
//             motorVibrate();
//         }
//     }
//     void toCSV() {
//         return;
//     }

//     uint64_t getRunningTime() {
//         return currentTime - startTime;
//     }

//     void updateCurrentTime() {
//         currentTime = getCurrentTime();
//     }

//     void updatePressure() {
//         prevHeelPressure = currHeelPressure;
//         prevForePressure =  currForePressure;
//         prevCombinedPressure = currCombinedPressure;
        
//         currHeelPressure = getSensorReadings(160);
//         currForePressure = getSensorReadings(224);
//         currCombinedPressure = currHeelPressure + currForePressure;

//         if (currCombinedPressure < minCombinedPressure)
//             minCombinedPressure = currCombinedPressure;

//         if (currCombinedPressure > maxCombinedPressure)
//             maxCombinedPressure = currCombinedPressure;

//         if (currHeelPressure < minHeelPressure)
//             minHeelPressure = currHeelPressure;

//         if (currHeelPressure > maxHeelPressure)
//             maxHeelPressure = currHeelPressure;

//         cout << "currHeelPressure: " << currHeelPressure << endl;
//         cout << "currForePressure: " << currForePressure << endl;

//     }

//     int getSensorReadings(unsigned char signal) {
//         // ADC channels
//         digitalWrite(CS,LOW);
//         SPIbuff[0] = 1;
//         SPIbuff[1] = signal;
//         SPIbuff[2] = 0;
//         wiringPiSPIDataRW(SPI_CHANNEL,SPIbuff,3);
//         int sensorReading = SPIbuff[1] << 8 | SPIbuff[2];
//         digitalWrite(CS, HIGH);
//         return sensorReading;
//     }

//     bool getMode() {
//         return mode;
//     }

// private: 

//     double heelThresholdInterval = 0;
//     double previousHeelThresholdTime = 0;
//     double currentHeelThresholdTime = 0;
//     unsigned char SPIbuff[3];

//     uint64_t getCurrentTime() {
//         return getMicrosTimeStamp();
//     }
//     void updateHeelThresholdInterval() {
//         previousHeelThresholdTime = currentHeelThresholdTime;
//         currentHeelThresholdTime = getCurrentTime();
//         heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;
//     }

//     bool detectThreshold(int prevReading, int currReading, int minReading, int maxReading) {
//         double threshold = 0.1 * (maxReading - minReading) + minReading;
//         if (prevReading < threshold && currReading > threshold) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     bool detectHeelThreshold(){
//         bool thresholdDetected = detectThreshold(prevHeelPressure, currHeelPressure, minHeelPressure, maxHeelPressure);
//         if (thresholdDetected)
//             updateHeelThresholdInterval();
//         return thresholdDetected;
//     }

//     bool detectCombinedThreshold(){
//         return detectThreshold(prevCombinedPressure, currCombinedPressure, minCombinedPressure, maxCombinedPressure);
//     }

//     void playSound() {
//         // UDPSend();
//         cout << "Played Sound" << endl;
//     }

// };

////////////////////////////////////////////////////
// MAIN CODE

int main(int argc, char* argv[])
{
    SonicSole* sole = new SonicSole();
    cout << "SonicSole Class Initialized" << endl;

    sole->motorVibrate();
    cout << "Motor Vibrated" << endl;


    while (true) {
      sole->updateCurrentTime();
      cout << "\nUpdate Pressure:" << endl;
      sole->updatePressure();

      sole->detectModeChange();

      if (sole->getMode()) {
          sole->runSoundMode();
      } else {
          sole->runVibrateMode();
      }
      

      sole->toCSV();

      cout << "\n";
      delay(100);
      // if (sole->getRunningTime() > MAX_RUN_TIME) {
      //    cout << sole->getRunningTime() << endl;
      //    cout << MAX_RUN_TIME << endl;
      //    return 0;
      // }
    }
    return 0;
}