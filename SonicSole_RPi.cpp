#include "SonicSole.h"

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

        sole.readIMU();

        const double time = sole.getRunningTime();
        std::cout << "\ntime: " << time << std::endl;

        struct timeval tvStart;
        struct timeval tvEnd;
        gettimeofday(&tvStart, nullptr);
        sole.readIMU();
        gettimeofday(&tvEnd, nullptr);

        const long elapsedMicroseconds =
            (tvEnd.tv_sec - tvStart.tv_sec) * 1000000L + (tvEnd.tv_usec - tvStart.tv_usec);
        const double elapsedSeconds = elapsedMicroseconds / 1e6;
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
