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

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
    sendto(sockfd, (const int *)reading, len,
           MSG_CONFIRM, (const struct sockaddr *) &servaddr,
           sizeof(servaddr));
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
    if ((IMU = serialOpen ("/dev/ttyS0", 921600)) < 0) {
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

SonicSole::~SonicSole() {
    closeCSVFile();
}

string generateFileName() {
    time_t now = time(nullptr);
    tm* localTime = localtime(&now);

    ostringstream filenameStream;
    filenameStream << "sole_data_"
                   << (localTime->tm_year + 1900) << "_"
                   << (localTime->tm_mon + 1) << "_"
                   << localTime->tm_mday << "_"
                   << localTime->tm_hour << "_"
                   << localTime->tm_min << "_"
                   << localTime->tm_sec
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

// void SonicSole::detectModeChange() {
//     startInterval = getSecondsTimeStamp();
//     detectHeelThreshold();    
//     if (thresholdCross == 3 && heelThresholdInterval < 3) { //&& (endInterval - startInterval < 1)) {
//         mode = !mode; 
//         thresholdCross = 0;
//         string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
//     }
// }

void SonicSole::switchMode() {
    mode = !mode;
    thresholdCross = 0;
    string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
}

bool SonicSole::getMode() {
    return mode;
}

// void SonicSole::updateHeelThresholdInterval() {
//     previousHeelThresholdTime = currentHeelThresholdTime;
//     currentHeelThresholdTime = getRunningTime();
//     heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;    
// }

bool SonicSole::detectThreshold(int prevReading, int currReading, int minReading, int maxReading) {
    double threshold = 0.1 * (maxReading - minReading) + minReading; // threshold of device is 10% or 0.1
    if ((prevReading < threshold) && (currReading > threshold)) {
        // endInterval = getSecondsTimeStamp();
        return true;
    }
    return false;
}

bool SonicSole::detectHeelThreshold() {
    bool thresholdDetected = detectThreshold(prevHeelPressure, currHeelPressure, minHeelPressure, maxHeelPressure);
    if (thresholdDetected) {
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
    
    pinMode(20, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW); //Turn motors on and off to show device is on
    digitalWrite(20, LOW);
    delay(1000);
}



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
      for (int i = 0; i < NUMBER_BUFFER_PACKET; i++)
      {
        // GET IMU DATA
        YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
        while(serialDataAvail(IMU) < IMU_PACKET_LENGTH)
        {
          // If no IMU data received, do nothing (print 0 infinitely)
          // cout << serialDataAvail(IMU) << endl; 
        }
      	read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);
        reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);

        // uint64_t currentTime = getMicrosTimeStamp() - timestampStart;
        // currentTime = (getMicrosTimeStamp() - timestampStart) / 1000;
        // currentTime = getMicrosTimeStamp() / 1000;
      }

    // printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n\n", dataRAcc.r_ax, dataRAcc.r_ay, dataRAcc.r_az);
    ax = dataAcce.ax;
    ay = dataAcce.ay;
    az = dataAcce.az;
    printf("IMU Acceleration Vector: %0.2f , %0.2f , %0.2f \n", dataAcce.ax, dataAcce.ay, dataAcce.az);
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

void SonicSole::sendFlexSensorData(int flexSensorData, int port) {
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