#include "SonicSole.h"

#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.h"

void SonicSole::readIMU()
{
    structComponentQuaternion dataQuat;
    structComponentLinearAcceleration dataAcce;
    structComponentRawGyro dataGyro;
    structComponentRawAcceleration dataRAcc;

    for (std::size_t i = 0; i < sizeof(dataIMUPacket); ++i) {
        dataIMUPacket[i] = 0x00;
    }

    YEIwriteCommandNoDelay(IMU, CMD_GET_STREAMING_BATCH);
    while (serialDataAvail(IMU) < IMU_PACKET_LENGTH) {
    }

    read(IMU, dataIMUPacket, IMU_PACKET_LENGTH);
    reconstructIMUPacket(dataIMUPacket, dataQuat, dataAcce, dataGyro, dataRAcc);

    ax = dataAcce.ax;
    ay = dataAcce.ay;
    az = dataAcce.az;
    qx = dataQuat.qx;
    qy = dataQuat.qy;
    qz = dataQuat.qz;
    qw = dataQuat.qw;
}

void SonicSole::getAccelVectorData(float az, std::vector<float>& azVector)
{
    azVector.push_back(az);
}

float SonicSole::vectorIntegral(std::vector<float> v)
{
    if (v.size() < 2) {
        return 0;
    }

    const int deltaX = v.size() / 20;
    int sumOfPoints = v[0];

    for (std::size_t i = 1; i < v.size() - 1; ++i) {
        sumOfPoints += (2 * v[i]);
    }
    sumOfPoints += v[v.size() - 1];

    const float vectInt = 0.5f * deltaX * sumOfPoints;
    return vectInt;
}
