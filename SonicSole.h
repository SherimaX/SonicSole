#ifndef SONICSOLE_H
#define SONICSOLE_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

inline constexpr int MAX_RUN_TIME = 600000000;
inline constexpr int SPI_CHANNEL = 0;
inline constexpr int CS = 17;
inline constexpr int IMU_PACKET_LENGTH = 52;

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

    double ax = 0;
    double ay = 0;
    double az = 0;
    double qx = 0;
    double qy = 0;
    double qz = 0;
    double qw = 1;

    std::uint64_t startTime = 0;
    std::uint64_t currentTime = 0;
    // uint64_t startInterval = 0;
    // uint64_t endInterval = 0;

    std::uint8_t dataIMUPacket[IMU_PACKET_LENGTH] = {};
    bool imuConfigured = false;
    int imuPendingBytes = 0;
    int imuBaudRate = 0;
    int imuConsecutiveFailures = 0;
    int imuRequestedBytes = 0;
    int imuBytesRead = 0;
    int imuReadChunks = 0;
    int imuPendingBeforeRequest = 0;
    int imuPendingAfterRead = 0;
    bool imuReadComplete = false;
    std::string imuPort;
    std::string imuState = "idle";
    std::string imuLastEvent = "not initialized";
    std::string imuLastRequest = "none";
    std::string imuPacketPreview = "n/a";
    bool recordState = true;

    // tss_device_id sensor_id;

    //vector<int> timeArr;
    //int timeArr[200];

    SonicSole();
    ~SonicSole();
    // void motorVibrate(); //no longer in use
    // void detectModeChange(); //no longer in use
    // void runSoundMode(); //no longer in use
    // void runVibrateMode(); //no longer in use

    // void toCSV();
    void toCSV(double time, double heelPressure, double forePressure, float ax, float ay, float az);
    void openCSVFile();
    void closeCSVFile();

    void readIMU();
    double getRunningTime();
    void updateCurrentTime();
    void updatePressure();
    int getSensorReadings(unsigned char signal);
    //bool getMode(); //no longer in use
    //void switchMode(); //no longer in use
    int getCurrForePressure();
    int getCurrHeelPressure();
    void sendFlexSensorData(int flexSensorData, int port);
    //bool detectHeelThreshold(); //no longer in use
    //void updateThresholdCounter(); //no longer in use

    // void getAccelVectorData(float ax, float ay, float az, vector<float>& axVector, vector<float>& ayVector, vector<float>& azVector);
    void getAccelVectorData(float az, std::vector<float>& azVector);
    float vectorIntegral(std::vector<float> v);

    void sendSensorData(float flexSensorData[], int port, std::size_t numElements);


private:
    double heelThresholdInterval = 0;
    double previousHeelThresholdTime = 0;
    double currentHeelThresholdTime = 0;
    unsigned char SPIbuff[3];

    // void updateHeelThresholdInterval(); //no longer in use
    std::ofstream outFile;
    // bool detectThreshold(int prevReading, int currReading, int minReading, int maxReading); //no longer in use
    // bool detectCombinedThreshold(); //no longer in use
    // void playSound(); //no longer in use
    // uint64_t getCurrentTime();
};

#endif // SONICSOLE_H
