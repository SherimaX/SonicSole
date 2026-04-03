#include "SonicSole.h"

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <vector>

namespace {

void sendCurrentSensorPacket(SonicSole& sole)
{
    float sensorData[] = {
        static_cast<float>(sole.currForePressure),
        static_cast<float>(sole.currHeelPressure),
        static_cast<float>(sole.ax),
        static_cast<float>(sole.ay),
        static_cast<float>(sole.az),
        static_cast<float>(sole.qx),
        static_cast<float>(sole.qy),
        static_cast<float>(sole.qz),
        static_cast<float>(sole.qw)
    };

    sole.sendSensorData(sensorData, 21000, 9);
}

} // namespace

int main(int argc, char* argv[])
{
    SonicSole sole;
    std::cout << "SonicSole Class Initialized" << std::endl;

    sole.openCSVFile();

    int cycle = 0;
    std::vector<float> azData;

    while (true) {
        sole.updateCurrentTime();
        sole.updatePressure();

        const auto imuReadStart = std::chrono::steady_clock::now();
        sole.readIMU();
        const auto imuReadElapsed = std::chrono::steady_clock::now() - imuReadStart;

        const double time = sole.getRunningTime();
        std::cout << "\ntime: " << time << std::endl;

        const double elapsedSeconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(imuReadElapsed).count();
        std::cout << "\ntime (since last): " << elapsedSeconds << " s" << std::endl;
        std::cout << "Cycle: " << cycle << std::endl;
        ++cycle;

        sole.getAccelVectorData(sole.az, azData);
        sole.toCSV(time, sole.currForePressure, sole.currHeelPressure, sole.az);

        std::cout << "\nFore Pressure: " << sole.currForePressure << std::endl;
        std::cout << "Heel Pressure: " << sole.currHeelPressure << std::endl;
        std::cout << "IMU Data (g) (ax, ay, az): "
                  << sole.ax << ", " << sole.ay << ", " << sole.az << std::endl;

        sendCurrentSensorPacket(sole);
    }

    return 0;
}
