#include "SonicSole.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.h"

namespace {

constexpr auto kImuReadTimeout = std::chrono::milliseconds(250);
constexpr auto kImuPollInterval = std::chrono::milliseconds(2);
constexpr std::size_t kQuaternionPacketLength = 16;
constexpr std::size_t kCorrectedSensorPacketLength = 9 * sizeof(float);

void logImuTimeout(const char* requestName)
{
    static auto lastWarningTime = std::chrono::steady_clock::time_point::min();
    const auto now = std::chrono::steady_clock::now();
    if (now - lastWarningTime >= std::chrono::seconds(1)) {
        std::cerr << "Timeout waiting for IMU response to " << requestName
                  << "; continuing with previous IMU values"
                  << std::endl;
        lastWarningTime = now;
    }
}

float decodeBigEndianFloat(const uint8_t* bytes)
{
    float value = 0.0f;
    auto* valueBytes = reinterpret_cast<uint8_t*>(&value);
    valueBytes[3] = bytes[0];
    valueBytes[2] = bytes[1];
    valueBytes[1] = bytes[2];
    valueBytes[0] = bytes[3];
    return value;
}

bool readExactBytes(int serial, uint8_t* buffer, std::size_t numBytes, const char* requestName)
{
    std::size_t totalBytesRead = 0;
    const auto waitStart = std::chrono::steady_clock::now();

    while (totalBytesRead < numBytes) {
        const int availableBytes = serialDataAvail(serial);
        if (availableBytes <= 0) {
            if (std::chrono::steady_clock::now() - waitStart >= kImuReadTimeout) {
                logImuTimeout(requestName);
                return false;
            }
            std::this_thread::sleep_for(kImuPollInterval);
            continue;
        }

        const std::size_t bytesToRead = std::min<std::size_t>(
            static_cast<std::size_t>(availableBytes),
            numBytes - totalBytesRead
        );
        const ssize_t bytesRead = read(serial, buffer + totalBytesRead, bytesToRead);
        if (bytesRead > 0) {
            totalBytesRead += static_cast<std::size_t>(bytesRead);
            continue;
        }

        if (std::chrono::steady_clock::now() - waitStart >= kImuReadTimeout) {
            logImuTimeout(requestName);
            return false;
        }
        std::this_thread::sleep_for(kImuPollInterval);
    }

    return true;
}

bool requestImuPacket(int serial, uint8_t command, uint8_t* buffer, std::size_t numBytes, const char* requestName)
{
    tcflush(serial, TCIFLUSH);
    YEIwriteCommandNoDelay(serial, command);
    return readExactBytes(serial, buffer, numBytes, requestName);
}

} // namespace

void SonicSole::readIMU()
{
    if (IMU < 0) {
        return;
    }

    std::array<uint8_t, kQuaternionPacketLength> quaternionPacket = {};
    if (!requestImuPacket(
            IMU,
            READ_TARED_ORIENTATION_AS_QUATERNION,
            quaternionPacket.data(),
            quaternionPacket.size(),
            "tared quaternion")) {
        return;
    }

    std::array<uint8_t, kCorrectedSensorPacketLength> correctedSensorPacket = {};
    if (!requestImuPacket(
            IMU,
            CMD_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA,
            correctedSensorPacket.data(),
            correctedSensorPacket.size(),
            "corrected sensor data")) {
        return;
    }

    qx = decodeBigEndianFloat(quaternionPacket.data());
    qy = decodeBigEndianFloat(quaternionPacket.data() + 4);
    qz = decodeBigEndianFloat(quaternionPacket.data() + 8);
    qw = decodeBigEndianFloat(quaternionPacket.data() + 12);

    ax = decodeBigEndianFloat(correctedSensorPacket.data() + (3 * sizeof(float)));
    ay = decodeBigEndianFloat(correctedSensorPacket.data() + (4 * sizeof(float)));
    az = decodeBigEndianFloat(correctedSensorPacket.data() + (5 * sizeof(float)));
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
