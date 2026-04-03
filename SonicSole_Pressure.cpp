#include "SonicSole.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

void SonicSole::updatePressure()
{
    prevHeelPressure = currHeelPressure;
    prevForePressure = currForePressure;
    prevCombinedPressure = currCombinedPressure;

    currHeelPressure = getSensorReadings(heelSensorAddr);
    currForePressure = getSensorReadings(foreSensorAddr);
    currCombinedPressure = currHeelPressure + currForePressure;

    if (currCombinedPressure < minCombinedPressure) {
        minCombinedPressure = currCombinedPressure;
    }

    if (currCombinedPressure > maxCombinedPressure) {
        maxCombinedPressure = currCombinedPressure;
    }

    if (currHeelPressure < minHeelPressure) {
        minHeelPressure = currHeelPressure;
    }

    if (currHeelPressure > maxHeelPressure) {
        maxHeelPressure = currHeelPressure;
    }

    if (currForePressure < minForePressure) {
        minForePressure = currForePressure;
    }

    if (currForePressure > maxForePressure) {
        maxForePressure = currForePressure;
    }
}

int SonicSole::getCurrHeelPressure()
{
    return currHeelPressure;
}

int SonicSole::getCurrForePressure()
{
    return currForePressure;
}

int SonicSole::getSensorReadings(unsigned char signal)
{
    digitalWrite(CS, LOW);
    SPIbuff[0] = 1;
    SPIbuff[1] = signal;
    SPIbuff[2] = 0;
    wiringPiSPIDataRW(SPI_CHANNEL, SPIbuff, 3);
    const int sensorReading = SPIbuff[1] << 8 | SPIbuff[2];
    digitalWrite(CS, HIGH);
    return sensorReading;
}
