#include "SonicSole.h"

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

void SonicSole::detectModeChange() {
    detectHeelThreshold();
    if (thresholdCross == 3 && heelThresholdInterval < 3){ //&& heelThresholdInterval < 1) {//&& detectHeelThreshold() && heelThresholdInterval < 1)
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
    currentHeelThresholdTime = getCurrentTime();
    heelThresholdInterval = currentHeelThresholdTime - previousHeelThresholdTime;    
}

bool SonicSole::detectThreshold(int prevReading, int currReading, int minReading, int maxReading) {
    double threshold = 0.5 * (maxReading - minReading) + minReading;
    if (prevReading < threshold && currReading > threshold) {
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
