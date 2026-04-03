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
#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.h"

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

    IMU = -1;
    printf("IMU serial will be initialized on first read.\n\n");

    printf("Initializing GPIO...\n\n");
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Unable to start wiringPi: %s\n", strerror(errno));
    } else {
        printf("GPIO initialized successfully!\n\n");
    }

    recordState = true;
}

SonicSole::~SonicSole()
{
    if (IMU >= 0) {
        close(IMU);
        IMU = -1;
    }
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
        outFile << "time, "
                << "heel pressure, "
                << "forefoot pressure, "
                << "ax, "
                << "ay, "
                << "az" << std::endl;
    }
}

void SonicSole::closeCSVFile()
{
    if (outFile.is_open()) {
        outFile.close();
    }
}

void SonicSole::toCSV(
    double time,
    double heelPressure,
    double forePressure,
    float ax,
    float ay,
    float az)
{
    if (!outFile.is_open()) {
        std::cerr << "File stream is not open" << std::endl;
        return;
    }

    outFile << time << ", "
            << heelPressure << ", "
            << forePressure << ", "
            << ax << ", "
            << ay << ", "
            << az << std::endl;
}

double SonicSole::getRunningTime()
{
    return (static_cast<double>(currentTime) - static_cast<double>(startTime)) / 1000000.0;
}

void SonicSole::updateCurrentTime()
{
    currentTime = getMicrosTimeStamp();
}
