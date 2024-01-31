#include "SonicSole.h"
// #define PORT 25000;

uint64_t getMicrosTimeStamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

uint64_t getSecondsTimeStamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec;
}

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
    sendto(sockfd, (const int *)reading, len,
           MSG_CONFIRM, (const struct sockaddr *) &servaddr,
           sizeof(servaddr));
}

SonicSole::SonicSole() {
    startTime = getSecondsTimeStamp();
    previousHeelThresholdTime = getSecondsTimeStamp();
    wiringPiSetupGpio() ;
    pinMode(CS, OUTPUT) ;
    digitalWrite(CS,HIGH);

    // INITIALIZING SPI
    printf("Initializing SPI...\n\n");
    int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
    if (fd == -1) {
        printf ("Failed to init SPI communication.\n") ;
        //std::cout << ("Failed to init SPI communication.\n");
    }
    else {
        //std::cout << "SPI communication successfully setup.\n";
        printf("SPI communication successfully setup.\n\n");
    }

    this_thread::sleep_for(chrono::milliseconds(500));

    // INITIALIZING UART1
    printf("Initializing UART0...\n\n");
    if ((IMU = serialOpen ("/dev/ttyS0", 115200)) < 0) {
  		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
  	}
    else {
        printf("UART1 initialized successfully!\n\n");
    }


    // INITIALIZING GPIO (Use wPi pins, not BCM)
    printf("Initializing GPIO...\n\n");
    if (wiringPiSetupGpio () == -1)
    {
        fprintf (stderr, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    }
    else {
        printf("GPIO initialized successfully!\n\n"); 
    }

	// CONFIGURING IMU
    try {
        printf("Configuring IMU...\n\n");
        YEIsettingsHeader(IMU);
        YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
        this_thread::sleep_for(chrono::milliseconds(1000));
        YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
        YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
        YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
        YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS,AXIS_XR_YF_ZU);
        YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE);
        YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);
        YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
        this_thread::sleep_for(chrono::milliseconds(500));
        YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
        this_thread::sleep_for(chrono::milliseconds(500));
        YEIsetStreamingMode(IMU, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

        YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);
        printf("IMU configured successfully!\n\n");
    }
    catch (...) {
        printf("IMU not configured successfully: Error. \n\n");
    }

	bool recordState = true;
}

void SonicSole::detectModeChange() {
    startInterval = getSecondsTimeStamp();
    detectHeelThreshold();    
    if (thresholdCross == 3 && heelThresholdInterval < 3) { //&& (endInterval - startInterval < 1)) {
        mode = !mode;
        thresholdCross = 0;
        string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
    }
}

void SonicSole::toCSV() {
    return;
}

uint64_t SonicSole::getRunningTime() {
    return currentTime - startTime;
}

void SonicSole::updateCurrentTime() {
    currentTime = getCurrentTime();
}

void SonicSole::updatePressure() {
    prevHeelPressure = currHeelPressure;
    prevForePressure =  currForePressure;
    prevCombinedPressure = currCombinedPressure;
    
    currHeelPressure = getSensorReadings(160);
    currForePressure = getSensorReadings(224);
    currCombinedPressure = currHeelPressure + currForePressure;

    if (currCombinedPressure < minCombinedPressure)
        minCombinedPressure = currCombinedPressure;

    if (currCombinedPressure > maxCombinedPressure)
        maxCombinedPressure = currCombinedPressure;

    if (currHeelPressure < minHeelPressure)
        minHeelPressure = currHeelPressure;

    if (currHeelPressure > maxHeelPressure)
        maxHeelPressure = currHeelPressure;

    cout << "currHeelPressure: " << currHeelPressure << endl;
    cout << "currForePressure: " << currForePressure << endl; 

}

int SonicSole::getCurrHeelPressure() {
    return currHeelPressure;
}

int SonicSole::getCurrForePressure() {
    return currForePressure;
}

int SonicSole::getSensorReadings(unsigned char signal) {
    // ADC channels
    digitalWrite(CS,LOW);
    SPIbuff[0] = 1;
    SPIbuff[1] = signal;
    SPIbuff[2] = 0;
    wiringPiSPIDataRW(SPI_CHANNEL,SPIbuff,3);
    int sensorReading = SPIbuff[1] << 8 | SPIbuff[2];
    digitalWrite(CS, HIGH);
    return sensorReading;
}

bool SonicSole::getMode() {
    return mode;
}

uint64_t SonicSole::getCurrentTime() {
    return getSecondsTimeStamp();
}

void SonicSole::updateHeelThresholdInterval() {
    previousHeelThresholdTime = currentHeelThresholdTime;
    currentHeelThresholdTime = getRunningTime();
    heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;    
}

bool SonicSole::detectThreshold(int prevReading, int currReading, int minReading, int maxReading) {
    double threshold = 0.4 * (maxReading - minReading) + minReading;
    if (prevReading < threshold && currReading > threshold) {
        endInterval = getSecondsTimeStamp();
        return true;
    } else {
        return false;
    }
}

bool SonicSole::detectHeelThreshold() {
    bool thresholdDetected = detectThreshold(prevHeelPressure, currHeelPressure, minHeelPressure, maxHeelPressure);
    if (thresholdDetected) {
        updateHeelThresholdInterval();
        updateThresholdCounter();
    }
    return thresholdDetected;
} 

void SonicSole::updateThresholdCounter() {
  thresholdCross++;
}

bool SonicSole::detectCombinedThreshold() {
    return detectThreshold(prevCombinedPressure, currCombinedPressure, minCombinedPressure, maxCombinedPressure);
}

void SonicSole::runSoundMode() {
    cout << "Sound Mode on" << endl;
}

void SonicSole::playSound() {
    cout << "Played Sound" << endl;
}

void SonicSole::runVibrateMode() {
    // if (detectCombinedThreshold()) {
    //     playSound();
    // }
    cout << "Vibrate Mode on" << endl;
}

void SonicSole::motorVibrate() {
    digitalWrite(23, HIGH); //Turn motors on and off to show device is on
    digitalWrite(20, HIGH);
    delay(1000);
    digitalWrite(23, LOW);
    digitalWrite(20, LOW);
}

void SonicSole::readIMU() {
    // https://www.telesens.co/2017/03/11/imu-sampling-using-the-raspberry-pi/
    // look at documentation later, has some useful code
    // ADC - MCP3221
    // #define ADCAddress 0x4D

    structComponentQuaternion dataQuat;
    structComponentLinearAcceleration dataAcce;
    structComponentRawGyro dataGyro;
    structComponentRawAcceleration dataRAcc;
    uint8_t dataIMUPacket[IMU_PACKET_LENGTH];
    
    // YEIgetStreamingBatch(uStreamingDataIMU);
    // YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH); // didnt really do anything
    // read(IMU, dataIMUPacket, IMU_PACKET_LENGTH); // slows down everything, only allows reading every 10 seconds
    reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc); // important

    // printf("Raw IMU packet: \n");
    // for(int i=0; i<MAX_YEI_DATA_PACKET; i++) {
    //     printf("%02X ", YEIdataPacket[i]);
    // }

    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n\n", dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
    printf("\nIMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);
    printf("IMU Gyroscope Vector: %0.2f , %0.2f , %0.2f \n", dataGyro.gx, dataGyro.gy, dataGyro.gz);
    printf("IMU Quaternion Vector: %0.2f , %0.2f , %0.2f, %0.2f \n", dataQuat.qw, dataQuat.qx, dataQuat.qy, dataQuat.qz);
    return;

    // some old code that might be useful later
    // check out YEIgetStreamingBatch later
    // YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
    // if(serialDataAvail(IMU))
    // {
    //     uint64_t timeRead = getMicrosTimeStamp() - timestamp_start;

    //     read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);

    //     reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);
    // }

    // reconstructBinaryPacketBinary_test(dataIMUPacket, dataAcce);
    // reconstructBinaryPacketBinary_test(dataIMUPacket, ax, ay, az);
    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", ax, ay,az);

    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);

    // float deltaTime = (float)(getMicrosTimeStamp() - timeRead) / 1000000.0f;
    // float freq = 1/deltaTime;
    // float currenttime_micros = (float)(getMicrosTimeStamp() - timestamp_start);

    // printf("Time obtained!\n");
    // printf("IMU Gyroscope Vector: %0.2f , %0.2f , %0.2f \n", dataGyro.gx, dataGyro.gy, dataGyro.gz);
    // printf("Time: %0.3f secs \n", deltaTime);
}

void SonicSole::sendFlexSensorData(int flexSensorData) {
    int sockfd;
    struct sockaddr_in serverAddr;

    // UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // localhost

    /*
    if (sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error sending data" << std::endl;
    } else {
        std::cout << "Flex sensor data sent successfully!" << std::endl;
    }
    */

    try {
        UDPSend(sockfd, &flexSensorData, sizeof(flexSensorData), serverAddr);
        // sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    }
    catch (...) {
        std:cout << "Error: UDPSend cannot send data" << endl;
    }

    std::cout << "Data sent to UDP" << endl;
    close(sockfd);
}