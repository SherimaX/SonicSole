#ifndef SONICSOLE_H
#define SONICSOLE_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

inline constexpr int MAX_RUN_TIME = 600000000;
inline constexpr int SPI_CHANNEL = 0;
inline constexpr int CS = 17;

std::uint64_t getMicrosTimeStamp();
std::uint64_t getSecondsTimeStamp();

class SonicSole {
public:
    bool mode = true;
    int thresholdCounter = 0;
    int thresholdCross = 0;

    int heelSensorAddr = 224;
    int foreSensorAddr = 160;

    double minHeelPressure = 0;
    double minForePressure = 0;
    double minCombinedPressure = 0;

    double maxHeelPressure = 0;
    double maxForePressure = 0;
    double maxCombinedPressure = 0;

    double currHeelPressure = 0;
    double currForePressure = 0;
    double currCombinedPressure = 0;

    double prevHeelPressure = 0;
    double prevForePressure = 0;
    double prevCombinedPressure = 0;

    std::uint64_t startTime = 0;
    std::uint64_t currentTime = 0;

    bool recordState = true;

    SonicSole();
    ~SonicSole();

    void toCSV(double time, double heelPressure, double forePressure);
    void openCSVFile();
    void closeCSVFile();

    double getRunningTime();
    void updateCurrentTime();
    void updatePressure();
    int getSensorReadings(unsigned char signal);
    int getCurrForePressure();
    int getCurrHeelPressure();
    void sendFlexSensorData(int flexSensorData, int port);

    void sendSensorData(float flexSensorData[], int port, std::size_t numElements);

private:
    double heelThresholdInterval = 0;
    double previousHeelThresholdTime = 0;
    double currentHeelThresholdTime = 0;
    unsigned char SPIbuff[3];

    std::ofstream outFile;
};

#endif // SONICSOLE_H
