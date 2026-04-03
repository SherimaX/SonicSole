#include "SonicSole.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <wiringPi.h>

#include "RPi_combined_Header.h"
#include "RPi_Raj_Header.h"

namespace {

constexpr auto kImuReadTimeout = std::chrono::milliseconds(50);
constexpr auto kImuProbeTimeout = std::chrono::milliseconds(150);
constexpr auto kImuPollInterval = std::chrono::milliseconds(1);
constexpr auto kImuResetDelay = std::chrono::milliseconds(500);
constexpr auto kImuAutocalDelay = std::chrono::milliseconds(1000);
constexpr auto kImuFilterResetDelay = std::chrono::milliseconds(500);
constexpr auto kImuPostTareDelay = std::chrono::milliseconds(100);
constexpr auto kImuInitRetryInterval = std::chrono::seconds(2);
constexpr int kMaxConsecutiveReadFailures = 5;
constexpr int kProbeAttemptsPerBaud = 3;
const char* const kImuPortEnv = "SONICSOLE_IMU_PORT";
const char* const kImuBaudEnv = "SONICSOLE_IMU_BAUD";

bool gImuConfigured = false;
int gImuActiveBaud = 0;
std::string gImuActivePort;
int gConsecutiveReadFailures = 0;
auto gLastInitAttemptTime = std::chrono::steady_clock::time_point::min();
auto gLastFailureLogTime = std::chrono::steady_clock::time_point::min();

std::string readEnvString(const char* name)
{
    const char* value = std::getenv(name);
    return value == nullptr ? std::string() : std::string(value);
}

std::vector<std::string> getCandidatePorts()
{
    const std::string configuredPort = readEnvString(kImuPortEnv);
    if (!configuredPort.empty()) {
        return {configuredPort};
    }

    return {"/dev/ttyS0"};
}

std::vector<int> getCandidateBaudRates()
{
    const std::string configuredBaud = readEnvString(kImuBaudEnv);
    if (!configuredBaud.empty()) {
        char* endPtr = nullptr;
        const long parsedBaud = std::strtol(configuredBaud.c_str(), &endPtr, 10);
        if (endPtr != configuredBaud.c_str() && *endPtr == '\0' && parsedBaud > 0) {
            return {static_cast<int>(parsedBaud)};
        }
    }

    return {921600, 115200};
}

bool baudRateToConstant(int baudRate, speed_t& baudConstant)
{
    switch (baudRate) {
    case 115200:
        baudConstant = B115200;
        return true;
    case 230400:
        baudConstant = B230400;
        return true;
    case 460800:
        baudConstant = B460800;
        return true;
    case 921600:
        baudConstant = B921600;
        return true;
    default:
        return false;
    }
}

int availableBytes(int serial)
{
    int bytesAvailable = 0;
    if (ioctl(serial, FIONREAD, &bytesAvailable) == -1) {
        return 0;
    }
    return std::max(bytesAvailable, 0);
}

void closeImuPort()
{
    if (IMU >= 0) {
        close(IMU);
        IMU = -1;
    }
    gImuConfigured = false;
    gImuActiveBaud = 0;
    gImuActivePort.clear();
}

void clearInputBuffer(int serial)
{
    if (serial >= 0) {
        tcflush(serial, TCIFLUSH);
    }
}

void logImuTimeout(const char* requestName)
{
    const auto now = std::chrono::steady_clock::now();
    if (now - gLastFailureLogTime < std::chrono::seconds(1)) {
        return;
    }

    const int bytesPending = IMU >= 0 ? availableBytes(IMU) : 0;
    std::cerr << "Timeout waiting for IMU " << requestName
              << " on " << (gImuActivePort.empty() ? "<unbound-port>" : gImuActivePort)
              << " @ " << (gImuActiveBaud == 0 ? -1 : gImuActiveBaud)
              << " baud"
              << " (pending bytes: " << bytesPending << ")"
              << std::endl;
    gLastFailureLogTime = now;
}

void logProbeResult(
    const std::string& port,
    int baudRate,
    const std::string& message)
{
    std::cout << "[IMU probe] " << port << " @ " << baudRate << ": " << message << std::endl;
}

bool openImuPort(const std::string& port, int baudRate)
{
    speed_t baudConstant = B0;
    if (!baudRateToConstant(baudRate, baudConstant)) {
        logProbeResult(port, baudRate, "unsupported baud");
        return false;
    }

    const int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        logProbeResult(port, baudRate, std::string("open failed: ") + std::strerror(errno));
        return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        logProbeResult(port, baudRate, std::string("tcgetattr failed: ") + std::strerror(errno));
        close(fd);
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, baudConstant) != 0 || cfsetospeed(&tty, baudConstant) != 0) {
        logProbeResult(port, baudRate, std::string("cfset speed failed: ") + std::strerror(errno));
        close(fd);
        return false;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        logProbeResult(port, baudRate, std::string("tcsetattr failed: ") + std::strerror(errno));
        close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    IMU = fd;
    gImuActivePort = port;
    gImuActiveBaud = baudRate;
    return true;
}

bool readExactBytes(
    int serial,
    uint8_t* buffer,
    std::size_t numBytes,
    std::chrono::milliseconds timeout,
    const char* requestName)
{
    std::size_t totalBytesRead = 0;
    const auto waitStart = std::chrono::steady_clock::now();

    while (totalBytesRead < numBytes) {
        const int bytesAvailable = availableBytes(serial);
        if (bytesAvailable <= 0) {
            if (std::chrono::steady_clock::now() - waitStart >= timeout) {
                logImuTimeout(requestName);
                return false;
            }
            std::this_thread::sleep_for(kImuPollInterval);
            continue;
        }

        const std::size_t bytesToRead = std::min<std::size_t>(
            static_cast<std::size_t>(bytesAvailable),
            numBytes - totalBytesRead
        );
        const ssize_t bytesRead = ::read(serial, buffer + totalBytesRead, bytesToRead);
        if (bytesRead > 0) {
            totalBytesRead += static_cast<std::size_t>(bytesRead);
            continue;
        }

        if (std::chrono::steady_clock::now() - waitStart >= timeout) {
            logImuTimeout(requestName);
            return false;
        }
        std::this_thread::sleep_for(kImuPollInterval);
    }

    return true;
}

bool requestStreamingBatch(
    int serial,
    uint8_t* buffer,
    std::chrono::milliseconds timeout,
    const char* requestName)
{
    clearInputBuffer(serial);
    YEIwriteCommandNoDelay(serial, CMD_GET_STREAMING_BATCH);
    return readExactBytes(serial, buffer, IMU_PACKET_LENGTH, timeout, requestName);
}

bool packetLooksPlausible(const uint8_t* packet)
{
    bool nonZeroByteSeen = false;
    for (int index = 0; index < IMU_PACKET_LENGTH; ++index) {
        if (packet[index] != 0) {
            nonZeroByteSeen = true;
            break;
        }
    }

    if (!nonZeroByteSeen) {
        return false;
    }

    structComponentQuaternion dataQuat {};
    structComponentLinearAcceleration dataAcce {};
    structComponentRawGyro dataGyro {};
    structComponentRawAcceleration dataRAcc {};
    reconstructIMUPacket(const_cast<uint8_t*>(packet), dataQuat, dataAcce, dataGyro, dataRAcc);

    const auto finite = [](float value) {
        return std::isfinite(value) && std::fabs(value) < 1000.0f;
    };

    return finite(dataQuat.qw) &&
           finite(dataQuat.qx) &&
           finite(dataQuat.qy) &&
           finite(dataQuat.qz) &&
           finite(dataAcce.ax) &&
           finite(dataAcce.ay) &&
           finite(dataAcce.az);
}

void updateSoleFromStreamingPacket(SonicSole& sole, const uint8_t* packet)
{
    structComponentQuaternion dataQuat {};
    structComponentLinearAcceleration dataAcce {};
    structComponentRawGyro dataGyro {};
    structComponentRawAcceleration dataRAcc {};
    reconstructIMUPacket(const_cast<uint8_t*>(packet), dataQuat, dataAcce, dataGyro, dataRAcc);

    sole.ax = dataAcce.ax;
    sole.ay = dataAcce.ay;
    sole.az = dataAcce.az;
    sole.qx = dataQuat.qx;
    sole.qy = dataQuat.qy;
    sole.qz = dataQuat.qz;
    sole.qw = dataQuat.qw;
}

bool configureStreamingBatchMode()
{
    if (IMU < 0) {
        return false;
    }

    std::cout << "Configuring IMU streaming on " << gImuActivePort
              << " @ " << gImuActiveBaud << " baud" << std::endl;

    clearInputBuffer(IMU);
    YEIsettingsHeader(IMU);
    clearInputBuffer(IMU);

    YEIwriteCommandNoDelay(IMU, CMD_STOP_STREAMING);
    std::this_thread::sleep_for(kImuResetDelay);
    clearInputBuffer(IMU);

    YEIwriteCommandValue(IMU, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_8G);
    YEIwriteCommandValue(IMU, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
    YEIwriteCommandValue(IMU, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
    YEIwriteCommandValue(IMU, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
    YEIwriteCommandValue(IMU, CMD_SET_AXIS_DIRECTIONS, AXIS_XR_YF_ZU);
    YEIwriteCommandValue(IMU, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
    YEIwriteCommandValue(IMU, CMD_SET_COMPASS_ENABLE, FALSE);
    YEIwriteCommandValue(IMU, CMD_SET_FILTER_MODE, FILTER_KALMAN);

    YEIwriteCommandNoDelay(IMU, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
    std::this_thread::sleep_for(kImuAutocalDelay);
    clearInputBuffer(IMU);

    YEIwriteCommandNoDelay(IMU, CMD_RESET_FILTER);
    std::this_thread::sleep_for(kImuFilterResetDelay);
    clearInputBuffer(IMU);

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

    sStreamingTime.interval = 4000;
    sStreamingTime.duration = 0xFFFFFFFF;
    sStreamingTime.delay = 0;
    YEIsetStreamingTime(IMU);

    YEIwriteCommandNoDelay(IMU, CMD_TARE_WITH_CURRENT_ORIENTATION);
    std::this_thread::sleep_for(kImuPostTareDelay);
    clearInputBuffer(IMU);
    return true;
}

bool probeConfiguredImu(SonicSole& sole)
{
    std::array<uint8_t, IMU_PACKET_LENGTH> packet = {};
    for (int attempt = 0; attempt < kProbeAttemptsPerBaud; ++attempt) {
        if (!requestStreamingBatch(IMU, packet.data(), kImuProbeTimeout, "streaming batch probe")) {
            continue;
        }
        if (!packetLooksPlausible(packet.data())) {
            logProbeResult(gImuActivePort, gImuActiveBaud, "received implausible batch payload");
            continue;
        }

        std::copy(packet.begin(), packet.end(), sole.dataIMUPacket);
        updateSoleFromStreamingPacket(sole, packet.data());
        logProbeResult(gImuActivePort, gImuActiveBaud, "batch probe succeeded");
        return true;
    }

    const int bytesPending = availableBytes(IMU);
    std::ostringstream reason;
    reason << "no valid batch after configuration";
    if (bytesPending > 0) {
        reason << " (pending bytes: " << bytesPending << ")";
    }
    logProbeResult(gImuActivePort, gImuActiveBaud, reason.str());
    return false;
}

bool ensureImuConfigured(SonicSole& sole)
{
    if (gImuConfigured && IMU >= 0) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - gLastInitAttemptTime < kImuInitRetryInterval) {
        return false;
    }
    gLastInitAttemptTime = now;

    closeImuPort();

    for (const std::string& port : getCandidatePorts()) {
        for (const int baudRate : getCandidateBaudRates()) {
            if (!openImuPort(port, baudRate)) {
                continue;
            }

            logProbeResult(port, baudRate, "opened serial port");
            if (!configureStreamingBatchMode()) {
                closeImuPort();
                continue;
            }

            if (probeConfiguredImu(sole)) {
                gImuConfigured = true;
                gConsecutiveReadFailures = 0;
                return true;
            }

            closeImuPort();
        }
    }

    std::cerr << "Unable to establish IMU communication on any tested port/baud."
              << " Set " << kImuPortEnv << " or " << kImuBaudEnv
              << " if your Nano is on a different UART configuration."
              << std::endl;
    return false;
}

} // namespace

void SonicSole::readIMU()
{
    if (!ensureImuConfigured(*this)) {
        return;
    }

    std::fill(std::begin(dataIMUPacket), std::end(dataIMUPacket), 0);
    if (!requestStreamingBatch(IMU, dataIMUPacket, kImuReadTimeout, "streaming batch read")) {
        ++gConsecutiveReadFailures;
        if (gConsecutiveReadFailures >= kMaxConsecutiveReadFailures) {
            std::cerr << "Lost IMU batch stream; will reinitialize IMU link." << std::endl;
            closeImuPort();
        }
        return;
    }

    if (!packetLooksPlausible(dataIMUPacket)) {
        ++gConsecutiveReadFailures;
        logImuTimeout("plausible streaming batch payload");
        if (gConsecutiveReadFailures >= kMaxConsecutiveReadFailures) {
            std::cerr << "Discarding IMU session after repeated invalid packets." << std::endl;
            closeImuPort();
        }
        return;
    }

    gConsecutiveReadFailures = 0;
    updateSoleFromStreamingPacket(*this, dataIMUPacket);
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
