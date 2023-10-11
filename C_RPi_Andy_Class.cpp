#include "C_RPi_Andy_Class.h"

uint64_t getMicrosTimeStamp() {
    struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void UDPSend(int sockfd, const int *reading, socklen_t len, struct sockaddr_in servaddr) {
    sendto(sockfd, (const int *)reading, len,
           MSG_CONFIRM, (const struct sockaddr *) &servaddr,
           sizeof(servaddr));
}

C_RPi_Andy_Class::SonicSole() {
    startTime = getMicrosTimeStamp();
    wiringPiSetupGpio() ;
    pinMode(CS, OUTPUT) ;
    digitalWrite(CS,HIGH);
    cout << "Initializing SPI" << endl ;

    // INITIALIZING UART1
    printf("Initializing UART0...\n\n");

    // INITIALIZING GPIO (Use wPi pins, not BCM)
    printf("Initializing GPIO...\n\n");
    if (wiringPiSetupGpio () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    }
    printf("GPIO initialized successfully!\n\n");


    int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
    if (fd == -1)
    {
        std::cout << "Failed to init SPI communication.\n";
    }
    std::cout << "SPI communication successfully setup.\n";

    this_thread::sleep_for(chrono::milliseconds(500));
}


void C_RPi_Andy_Class::motorVibrate() {
    digitalWrite(23, HIGH); //Turn motors on and off to show device is on
    digitalWrite(20, HIGH);
    delay(1000);
    digitalWrite(23, LOW);
    digitalWrite(20, LOW);
}

void C_RPi_Andy_Class::detectModeChange() {
    if (detectHeelThreshold() && heelThresholdInterval < 1)
        mode = !mode;
        string text = mode ? "Switched to Sound Mode" : "Switched to Vibration Mode";
}

void C_RPi_Andy_Class::runVibrateMode() {
    if (detectCombinedThreshold()) {
        playSound();
    }
}

void C_RPi_Andy_Class::toCSV() {
    return;
}

uint64_t C_RPi_Andy_Class::getRunningTime() {
    return currentTime - startTime;
}

void C_RPi_Andy_Class::updateCurrentTime() {
    currentTime = getCurrentTime();
}

void C_RPi_Andy_Class::updatePressure() {
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

int C_RPi_Andy_Class::getSensorReadings(unsigned char signal) {
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

bool C_RPi_Andy_Class::getMode() {
    return mode;
}

uint64_t C_RPi_Andy_Class::getCurrentTime() {
    return getMicrosTimeStamp();
}

void C_RPi_Andy_Class::updateHeelThresholdInterval() {
    previousHeelThresholdTime = currentHeelThresholdTime;
    currentHeelThresholdTime = getCurrentTime();
    heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;    
}

bool C_RPi_Andy_Class::detectThreshold(int prevReading, int currReading, int minReading, int maxReading) {
    double threshold = 0.1 * (maxReading - minReading) + minReading;
    if (prevReading < threshold && currReading > threshold) {
        return true;
    } else {
        return false;
    }
}

bool C_RPi_Andy_Class::detectHeelThreshold() {
    bool thresholdDetected = detectThreshold(prevHeelPressure, currHeelPressure, minHeelPressure, maxHeelPressure);
    if (thresholdDetected)
        updateHeelThresholdInterval();
    return thresholdDetected;
}

bool C_RPi_Andy_Class::detectCombinedThreshold() {
    return detectThreshold(prevCombinedPressure, currCombinedPressure, minCombinedPressure, maxCombinedPressure);
}

void C_RPi_Andy_Class::playSound() {
    cout << "Played Sound" << endl;
}


 