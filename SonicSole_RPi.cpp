#include "SonicSole.h"
#include "SonicSole_Activity.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sys/time.h>
#include <vector>

namespace {

constexpr int kDefaultActivityPort = 21010;
const char* const kActivityPortEnv = "SONICSOLE_ACTIVITY_PORT";

int resolveActivityPort()
{
    const char* raw = std::getenv(kActivityPortEnv);
    if (raw == nullptr || *raw == '\0') {
        return kDefaultActivityPort;
    }
    char* endPtr = nullptr;
    const long parsed = std::strtol(raw, &endPtr, 10);
    if (endPtr == raw || *endPtr != '\0' || parsed <= 0 || parsed > 65535) {
        std::cerr << "Ignoring invalid " << kActivityPortEnv << "=" << raw
                  << ", falling back to " << kDefaultActivityPort << std::endl;
        return kDefaultActivityPort;
    }
    return static_cast<int>(parsed);
}

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

int main(int /*argc*/, char* /*argv*/[])
{
    SonicSole sole;
    std::cout << "SonicSole Class Initialized" << std::endl;

    ActivityManager activityManager;
    activityManager.registerActivity(std::make_unique<BalanceActivity>());
    // Add more activities here (jump, reaction, precision) by registering
    // additional subclasses of ActivityBase. The command/response protocol
    // and client wiring in the webapp are shared across activities.
    if (!activityManager.start(resolveActivityPort())) {
        std::cerr << "Activity control listener disabled; sensor streaming only."
                  << std::endl;
    }

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
        sole.toCSV(
            time,
            sole.currHeelPressure,
            sole.currForePressure,
            static_cast<float>(sole.ax),
            static_cast<float>(sole.ay),
            static_cast<float>(sole.az));

        std::cout << "IMU Debug: state=" << sole.imuState
                  << ", configured=" << (sole.imuConfigured ? "yes" : "no")
                  << ", port=" << (sole.imuPort.empty() ? "<unset>" : sole.imuPort)
                  << ", baud=" << sole.imuBaudRate
                  << ", pending=" << sole.imuPendingBytes
                  << ", failures=" << sole.imuConsecutiveFailures
                  << std::endl;
        std::cout << "IMU Event: " << sole.imuLastEvent << std::endl;
        std::cout << "IMU Read: request=" << sole.imuLastRequest
                  << ", requested=" << sole.imuRequestedBytes
                  << ", read=" << sole.imuBytesRead
                  << ", chunks=" << sole.imuReadChunks
                  << ", complete=" << (sole.imuReadComplete ? "yes" : "no")
                  << ", pending_before=" << sole.imuPendingBeforeRequest
                  << ", pending_after=" << sole.imuPendingAfterRead
                  << std::endl;
        std::cout << "IMU Data (g) (ax, ay, az): "
                  << sole.ax << ", " << sole.ay << ", " << sole.az << std::endl;
        std::cout << "IMU Packet Preview: " << sole.imuPacketPreview << std::endl;

        if (activityManager.hasActiveActivity()) {
            std::cout << "Activity: " << activityManager.activeActivityName()
                      << " in progress" << std::endl;
        }

        activityManager.tick(sole);

        sendCurrentSensorPacket(sole);
    }

    return 0;
}
