#include "SonicSole.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <errno.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <thread>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>

#include "RPi_Raj_Header.h"
#include "RPi_combined_Header.h"

std::uint64_t getMicrosTimeStamp()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * static_cast<std::uint64_t>(1000000) + tv.tv_usec;
}

std::uint64_t getSecondsTimeStamp()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec;
}

namespace {

std::string generateFileName()
{
    const std::string directory = "SoleData";

    struct stat info;
    if (stat(directory.c_str(), &info) != 0) {
        mkdir(directory.c_str(), 0777);
    }

    const std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    std::ostringstream filenameStream;
    filenameStream << directory << "/sole_data_"
                   << (localTime->tm_year + 1900) << "_"
                   << std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "_"
                   << std::setw(2) << std::setfill('0') << localTime->tm_mday << "_"
                   << std::setw(2) << std::setfill('0') << localTime->tm_hour << ":"
                   << std::setw(2) << std::setfill('0') << localTime->tm_min << ":"
                   << std::setw(2) << std::setfill('0') << localTime->tm_sec
                   << ".csv";
    return filenameStream.str();
}

} // namespace

SonicSole::SonicSole()
{
    startTime = getMicrosTimeStamp();
    currentTime = startTime;
    previousHeelThresholdTime = getMicrosTimeStamp();
    wiringPiSetupGpio();
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);

    pinMode(20, OUTPUT);
    pinMode(3, OUTPUT);

    printf("Initializing SPI...\n\n");
    const int fd = wiringPiSPISetupMode(SPI_CHANNEL, 1000000, 0);
    if (fd == -1) {
        printf("Failed to init SPI communication.\n");
    } else {
        printf("SPI communication successfully setup.\n\n");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    printf("Initializing UART0...\n\n");
    if ((IMU = serialOpen("/dev/ttyS0", 115200)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
    } else {
        printf("UART1 initialized successfully!\n\n");
    }

    printf("Initializing GPIO...\n\n");
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Unable to start wiringPi: %s\n", strerror(errno));
    } else {
        printf("GPIO initialized successfully!\n\n");
    }

    try {
        printf("Configuring IMU...\n\n");
        YEIsettingsHeader(IMU);
        YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
        YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
        YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
        YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS, AXIS_XF_YU_ZL);
        YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
        YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE);
        YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);
        YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        YEIsetStreamingMode(
            IMU,
            READ_TARED_ORIENTATION_AS_QUATERNION,
            READ_CORRECTED_LINEAR_ACCELERATION,
            READ_CORRECTED_GYROSCOPE_VECTOR,
            READ_CORRECTED_ACCELEROMETER_VECTOR,
            NO_SLOT,
            NO_SLOT,
            NO_SLOT,
            NO_SLOT
        );
        YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);

        printf("Current Streaming Interval: %d", sStreamingTime.interval);
        sStreamingTime.interval = 1000;
        sStreamingTime.duration = 0xFFFFFFFF;
        sStreamingTime.delay = 0;
        printf("Updated Streaming Interval: %d", sStreamingTime.interval);
        YEIsetStreamingTime(IMU);

        printf("IMU configured successfully!\n\n");
    } catch (...) {
        printf("IMU not configured successfully: Error. \n\n");
    }

    recordState = true;
}

SonicSole::~SonicSole()
{
    closeCSVFile();
}

void SonicSole::openCSVFile()
{
    const std::string filename = generateFileName();
    outFile.open(filename, std::ios::out | std::ios::app);
    if (!outFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    if (outFile.tellp() == 0) {
        outFile << "time, " << "heel pressure, " << "forefoot pressure, " << "az" << std::endl;
    }
}

void SonicSole::closeCSVFile()
{
    if (outFile.is_open()) {
        outFile.close();
    }
}

void SonicSole::toCSV(double time, double heelpresh, double forepresh, float az)
{
    if (!outFile.is_open()) {
        std::cerr << "File stream is not open" << std::endl;
        return;
    }

    outFile << time << ", " << heelpresh << ", " << forepresh << ", " << az << std::endl;
}

double SonicSole::getRunningTime()
{
    return (static_cast<double>(currentTime) - static_cast<double>(startTime)) / 1000000.0;
}

void SonicSole::updateCurrentTime()
{
    currentTime = getMicrosTimeStamp();
}
