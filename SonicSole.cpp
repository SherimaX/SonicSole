#include "SonicSole.h"
// #define PORT 2000

uint64_t getMicrosTimeStamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

uint64_t getSecondsTimeStamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec;
    // return tv.tv_sec+tv.tv_usec;
}

//void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
  //  sendto(sockfd, (const int *)reading, len,
    //       MSG_CONFIRM, (const struct sockaddr *) &servaddr,
      //     sizeof(servaddr));
//}

void UDPSendArray(int sockfd, const float* data, size_t numElements, const struct sockaddr_in& servaddr) {
    size_t numBytes = numElements * sizeof(float);

    ssize_t sent = sendto(sockfd, data, numBytes, 0,
                          (const struct sockaddr*)&servaddr,
                          sizeof(servaddr));

    if (sent == -1) {
        std::cerr << "UDPSend error: Failed to send data" << std::endl;
    } else {
        std::cout << "UDPSend: Sent " << sent << " bytes" << std::endl;
    }
}

SonicSole::SonicSole() {
    // startTime = getSecondsTimeStamp();
    // previousHeelThresholdTime = getSecondsTimeStamp();
    startTime = getMicrosTimeStamp();
    currentTime = startTime; 
    previousHeelThresholdTime = getMicrosTimeStamp();
    wiringPiSetupGpio();
    pinMode(CS, OUTPUT);
    digitalWrite(CS,HIGH);

    pinMode(20, OUTPUT);
    pinMode(3, OUTPUT);
   
    // INITIALIZING SPI (SPI used for pressure, UART used for IMU)
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
    if ((IMU = serialOpen("/dev/ttyS0", 115200)) < 0) {  //set to original port (ttyS0)
    //if ((IMU = serialOpen("/dev/ttyAMA0", 115200)) < 0) { //The factory default baud rate is 115200
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

	// CONFIGURING IMU (UART)
    try {
        printf("Configuring IMU...\n\n");
        YEIsettingsHeader(IMU);
        YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
        this_thread::sleep_for(chrono::milliseconds(1000));
        YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
        YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
        YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);

        //YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS,AXIS_XR_YF_ZU); //original
        YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS,AXIS_XF_YU_ZL); //standard operation/natural axes

        YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE); // default true
        YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);
        YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
        this_thread::sleep_for(chrono::milliseconds(500));
        YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
        this_thread::sleep_for(chrono::milliseconds(500));
        YEIsetStreamingMode(IMU, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, READ_CORRECTED_GYROSCOPE_VECTOR, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
        YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);

        printf("Current Streaming Interval: %d", sStreamingTime.interval);
        sStreamingTime.interval = 1000; //4000
        sStreamingTime.duration = 0xFFFFFFFF;
        sStreamingTime.delay = 0;
        printf("Updated Streaming Interval: %d", sStreamingTime.interval);
        YEIsetStreamingTime(IMU);

        printf("IMU configured successfully!\n\n");
    }
    catch (...) {
        printf("IMU not configured successfully: Error. \n\n");
    }
	bool recordState = true; 
}

SonicSole::~SonicSole() {
    closeCSVFile();
}

string generateFileName() {
    string directory = "SoleData";

    // Create the directory if it doesn't exist
    struct stat info;
    if (stat(directory.c_str(), &info) != 0) {
        mkdir(directory.c_str(), 0777);
    }

    time_t now = time(0);
    tm* localTime = localtime(&now);

    ostringstream filenameStream;
    filenameStream << directory << "/sole_data_"
                   << (localTime->tm_year + 1900) << "_"
                   << setw (2) << setfill ('0') << (localTime->tm_mon + 1) << "_"
                   << setw (2) << setfill ('0') << localTime->tm_mday << "_"
                   << setw (2) << setfill ('0') << localTime->tm_hour << ":"
                   << setw (2) << setfill ('0') << localTime->tm_min << ":"
                   << setw (2) << setfill ('0') << localTime->tm_sec
                   << ".csv";
    return filenameStream.str();
}

void SonicSole::openCSVFile() {
    string filename = generateFileName();
    outFile.open(filename, ios::out | ios::app); 
    if (!outFile.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    if (outFile.tellp() == 0) {
        outFile << "time, " << "heel pressure, " << "forefoot pressure, " << "az" << endl;
    }
}

void SonicSole::closeCSVFile() {
    if (outFile.is_open()) {
        outFile.close();
    }
}

void SonicSole::toCSV(double time, double heelpresh, double forepresh, float az) {
    if (!outFile.is_open()) {
        cerr << "File stream is not open" << endl;
        return;
    }

    outFile << time << ", " << heelpresh << ", " << forepresh << ", " << az << endl;
}

double SonicSole::getRunningTime() {
    // return static_cast<double>(currentTime - startTime) / 1000000.0;
    return ((double) currentTime - (double) startTime) / 1000000.0;
}

void SonicSole::updateCurrentTime() {
    // currentTime = getSecondsTimeStamp();
    currentTime = getMicrosTimeStamp();
}

void SonicSole::updatePressure() {
    prevHeelPressure = currHeelPressure;
    prevForePressure =  currForePressure;
    prevCombinedPressure = currCombinedPressure;
    
    currHeelPressure = getSensorReadings(heelSensorAddr);
    currForePressure = getSensorReadings(foreSensorAddr);
    currCombinedPressure = currHeelPressure + currForePressure;

    // not sure if these are still used
    if (currCombinedPressure < minCombinedPressure)
        minCombinedPressure = currCombinedPressure;

    if (currCombinedPressure > maxCombinedPressure)
        maxCombinedPressure = currCombinedPressure;

    if (currHeelPressure < minHeelPressure)
        minHeelPressure = currHeelPressure;

    if (currHeelPressure > maxHeelPressure)
        maxHeelPressure = currHeelPressure;

    if (currForePressure < minForePressure)
        minForePressure = currForePressure;

    if (currForePressure > maxForePressure)
        maxForePressure = currForePressure;

    // cout << "currHeelPressure: " << 4096 - currHeelPressure << endl;
    // cout << "maxHeelPressure: " << 4096 - maxHeelPressure << endl;
    // cout << "currForePressure: " << 4096 - currForePressure << endl;
    // cout << "maxForePressure: " << 4096 - maxForePressure << endl;

    // cout << "currHeelPressure: " << currHeelPressure << endl;
    // cout << "maxHeelPressure: " << maxHeelPressure << endl;
    // cout << "currForePressure: " << currForePressure << endl;
    // cout << "maxForePressure: " << maxForePressure << endl;

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

// void SonicSole::detectModeChange() { //no longer in use
//     startInterval = getSecondsTimeStamp();
//     detectHeelThreshold();    
//     if (thresholdCross == 3 && heelThresholdInterval < 3) { //&& (endInterval - startInterval < 1)) {
//         mode = !mode; 
//         thresholdCross = 0;
//         string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
//     }
// }

// void SonicSole::switchMode() { //no longer in use
//     mode = !mode;
//     thresholdCross = 0;
//     string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
// }

// bool SonicSole::getMode() { //no longer in use
//     return mode;
// }

// void SonicSole::updateHeelThresholdInterval() { //no longer in use
//     previousHeelThresholdTime = currentHeelThresholdTime;
//     currentHeelThresholdTime = getRunningTime();
//     heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;    
// }

// bool SonicSole::detectThreshold(int prevReading, int currReading, int minReading, int maxReading) { //no longer in use
//     double threshold = 0.1 * (maxReading - minReading) + minReading; // threshold of device is 10% or 0.1
//     if ((prevReading < threshold) && (currReading > threshold)) {
//         // endInterval = getSecondsTimeStamp();
//         return true;
//     }
//     return false;
// }

// bool SonicSole::detectHeelThreshold() { //no longer in use
//     bool thresholdDetected = detectThreshold(prevHeelPressure, currHeelPressure, minHeelPressure, maxHeelPressure);
//     if (thresholdDetected) {
//         updateThresholdCounter();
//     }
//     return thresholdDetected;
// }

// void SonicSole::updateThresholdCounter() { //no longer in use
//     thresholdCross++;
// }

// bool SonicSole::detectCombinedThreshold() { //no longer in use
//     return detectThreshold(prevCombinedPressure, currCombinedPressure, minCombinedPressure, maxCombinedPressure);
// }

// void SonicSole::runSoundMode() { //no longer in use
//     cout << "Sound Mode on" << endl;
// }

// void SonicSole::playSound() { //no longer in use
//     cout << "Played Sound" << endl;
// }

// void SonicSole::runVibrateMode() { //no longer in use
//     // if (detectCombinedThreshold()) {
//     //     playSound();
//     // }
//     cout << "Vibrate Mode on" << endl;
// }

// void SonicSole::motorVibrate() { //no longer in use
//     pinMode(20, OUTPUT);
//     pinMode(3, OUTPUT);
//     digitalWrite(3, LOW); //Turn motors on and off to show device is on
//     digitalWrite(20, LOW);
//     delay(1000);
// }

void SonicSole::readIMU() {
    // https://www.telesens.co/2017/03/11/imu-sampling-using-the-raspberry-pi/
    // https://yostlabs.com/product/3-space-embedded-lx/ 
    // look at documentation later, has some useful code
    // ADC - MCP3221
    // #define ADCAddress 0x4D   

    structComponentQuaternion dataQuat;
    structComponentLinearAcceleration dataAcce;
    structComponentRawGyro dataGyro;
    structComponentRawAcceleration dataRAcc;

    for (int i = 0 ; i < sizeof(dataIMUPacket) ; i++) dataIMUPacket[i] = 0x00; 

      // FILL UP BUFFER BLOCK
    //   for (int i = 0; i < NUMBER_BUFFER_PACKET; i++)
    //   {
        // GET IMU DATA
        YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
        while(serialDataAvail(IMU) < IMU_PACKET_LENGTH)
        {
           // If no IMU data received, do nothing (print 0 infinitely)
        }

        //timeout (waits for 100 for data and if no data arrives, it prints timeout so the pressure data can still send, introduces potential delay of up to 1 second per read depending on how quickly data becomes available)
        // int waitCounter = 0;
        // while (serialDataAvail(IMU) < IMU_PACKET_LENGTH) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //     waitCounter++;
        //     if (waitCounter > 100) {
        //         std::cerr << "Timeout waiting for IMU data" << std::endl;
        //         return;
        //     }
        // }

      	read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);
        reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);

        // uint64_t currentTime = getMicrosTimeStamp() - timestampStart;
        // currentTime = (getMicrosTimeStamp() - timestampStart) / 1000;
        // currentTime = getMicrosTimeStamp() / 1000;
    //   }

    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n\n", dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
    ax = dataAcce.ax;
    ay = dataAcce.ay;
    az = dataAcce.az;
   
    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);
    // printf("IMU Gyroscope Vector: %0.2f , %0.2f , %0.2f \n", dataGyro.gx, dataGyro.gy, dataGyro.gz);
   
    // printf("IMU Quaternion Vector: %0.2f , %0.2f , %0.2f, %0.2f \n", dataQuat.qw, dataQuat.qx, dataQuat.qy, dataQuat.qz);
    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", ax, ay,az);
    // printf("Time obtained!\n");
    // printf("Time: %0.3f secs \n", deltaTime);
}

// void SonicSole::getAccelVectorData(float ax, float ay, float az, vector<float>& axVector, 
//                                         vector<float>& ayVector, vector<float>& azVector) 
// {
//   axVector.push_back(ax);
//   ayVector.push_back(ay);
//   azVector.push_back(az);
// }

void SonicSole::getAccelVectorData(float az, vector<float>& azVector) 
{
  azVector.push_back(az);
}

float SonicSole::vectorIntegral(vector<float> v) {
  if (v.size() < 2) {
    return 0; 
  }

  int deltaX = v.size() / 20;
  int sumOfPoints = v[0];

  for (int i = 1; i < v.size()-1; i++) {
    sumOfPoints += (2*v[i]);
  }
  sumOfPoints += v[v.size()-1];

  float vectInt = 0.5 * deltaX * (sumOfPoints);
  return vectInt;
}

void SonicSole::sendSensorData(float flexSensorData[], int port, size_t numElements){
    int sockfd;
    struct sockaddr_in serverAddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr("192.168.0.101"); // pi ip on tplink
    //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // localhost
    //serverAddr.sin_addr.s_addr = inet_addr("192.168.50.109"); // ip on wrsl wifi
    //serverAddr.sin_addr.s_addr = inet_addr("192.168.0.100"); //ip on tplink

    try {
        UDPSendArray(sockfd, flexSensorData, numElements, serverAddr);
    }
    catch (...) {
        std::cout << "Error: UDPSend cannot send data" << std::endl;
    }

    std::cout << "Data sent to UDP" << std::endl;
    close(sockfd);
}

void SonicSole::sendFlexSensorData(int flexSensorData, int port) { //no longer in use. see sendSensorData
    int sockfd;
    struct sockaddr_in serverAddr;

    // UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // localhost
    serverAddr.sin_addr.s_addr = inet_addr("192.168.0.101"); //ip of the pi on tplink
    //serverAddr.sin_addr.s_addr = inet_addr("192.168.50.109"); // ip on wrsl wifi
    //serverAddr.sin_addr.s_addr = inet_addr("192.168.0.100"); //ip on tp link
    /*
    if (sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error sending data" << std::endl;
    } else {
        std::cout << "Flex sensor data sent successfully!" << std::endl;
    }
    */

    try {
        float flexSensorDataFloat = static_cast<float>(flexSensorData);
        UDPSendArray(sockfd, &flexSensorDataFloat, 1, serverAddr);

       // UDPSendArray(sockfd, &flexSensorData, 1, serverAddr);

        //  UDPSend(sockfd, &flexSensorData, sizeof(flexSensorData), serverAddr);
        // sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    }
    catch (...) {
        //std:cout << "Error: UDPSend cannot send data" << endl;
        std::cout << "Error: UDPSend cannot send data" << endl;
    }

    std::cout << "Data sent to UDP" << endl;
    close(sockfd);
}
/*
void SonicSole::sendSensorData(int flexSensorData[], int port, size_t numElements){
    int sockfd;
    struct sockaddr_in serverAddr;

    // UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // localhost
    //serverAddr.sin_addr.s_addr = inet_addr("192.168.0.101");
    serverAddr.sin_addr.s_addr = inet_addr("192.168.50.109");

    
    //if (sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
      //  std::cerr << "Error sending data" << std::endl;
    //} else {
      //  std::cout << "Flex sensor data sent successfully!" << std::endl;
    //}
    

    try {
        // sendto(sockfd, flexSensorData, sizeof(flexSensorData), serverAddr);
        UDPSendArray(sockfd, flexSensorData, numElements, serverAddr);
        // sendto(sockfd, &flexData, sizeof(flexData), 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    }
    catch (...) {
       // std:cout << "Error: UDPSend cannot send data" << endl;
        std::cout << "Error: UDPSend cannot send data" << endl;
    }

    std::cout << "Data sent to UDP" << endl;
    close(sockfd);
}*/