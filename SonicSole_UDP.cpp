#include "SonicSole.h"

#include <arpa/inet.h>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

const char* const kLegacyServerIp = "192.168.137.77";
const char* const kServerIpEnv = "SONICSOLE_SERVER_IP";
const char* const kServerHostEnv = "SONICSOLE_SERVER_HOST";
const char* const kDiscoveryPortEnv = "SONICSOLE_DISCOVERY_PORT";
const char* const kDiscoveryTimeoutMsEnv = "SONICSOLE_DISCOVERY_TIMEOUT_MS";
const char* const kDiscoveryRequest = "SONICSOLE_DISCOVER_SERVER_V1";
const char* const kDiscoveryResponse = "SONICSOLE_SERVER_V1";
constexpr int kDefaultDiscoveryPort = 21001;
constexpr int kDefaultDiscoveryTimeoutMs = 500;
constexpr int kResolveRetrySeconds = 5;

struct CachedUdpTarget {
    std::string ipAddress;
    bool usingFallback = true;
    std::chrono::steady_clock::time_point lastResolveAttempt =
        std::chrono::steady_clock::time_point::min();
};

std::string readEnvString(const char* name)
{
    const char* value = std::getenv(name);
    return value == nullptr ? std::string() : std::string(value);
}

int readEnvInt(const char* name, int defaultValue)
{
    const std::string rawValue = readEnvString(name);
    if (rawValue.empty()) {
        return defaultValue;
    }

    char* endPtr = nullptr;
    const long parsedValue = std::strtol(rawValue.c_str(), &endPtr, 10);
    if (endPtr == rawValue.c_str() || *endPtr != '\0' || parsedValue <= 0) {
        return defaultValue;
    }

    return static_cast<int>(parsedValue);
}

bool fillServerAddress(const std::string& ipAddress, int port, struct sockaddr_in& serverAddr)
{
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    return inet_pton(AF_INET, ipAddress.c_str(), &serverAddr.sin_addr) == 1;
}

bool resolveHostnameToIpv4(const std::string& hostname, std::string& resolvedIp)
{
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo* result = nullptr;
    if (getaddrinfo(hostname.c_str(), nullptr, &hints, &result) != 0) {
        return false;
    }

    for (struct addrinfo* entry = result; entry != nullptr; entry = entry->ai_next) {
        const auto* ipv4Address =
            reinterpret_cast<const struct sockaddr_in*>(entry->ai_addr);
        char ipBuffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &ipv4Address->sin_addr, ipBuffer, sizeof(ipBuffer)) != nullptr) {
            resolvedIp = ipBuffer;
            freeaddrinfo(result);
            return true;
        }
    }

    freeaddrinfo(result);
    return false;
}

bool discoverServerIp(std::string& discoveredIp)
{
    const int discoveryPort = readEnvInt(kDiscoveryPortEnv, kDefaultDiscoveryPort);
    const int timeoutMs = readEnvInt(kDiscoveryTimeoutMsEnv, kDefaultDiscoveryTimeoutMs);

    const int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        return false;
    }

    int broadcastEnabled = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnabled, sizeof(broadcastEnabled));

    struct timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in broadcastAddr;
    std::memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(discoveryPort);
    broadcastAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    const ssize_t bytesSent = sendto(
        sockfd,
        kDiscoveryRequest,
        std::strlen(kDiscoveryRequest),
        0,
        reinterpret_cast<const struct sockaddr*>(&broadcastAddr),
        sizeof(broadcastAddr)
    );

    if (bytesSent == -1) {
        close(sockfd);
        return false;
    }

    char responseBuffer[128] = {};
    struct sockaddr_in responderAddr;
    socklen_t responderAddrLen = sizeof(responderAddr);
    const ssize_t bytesReceived = recvfrom(
        sockfd,
        responseBuffer,
        sizeof(responseBuffer) - 1,
        0,
        reinterpret_cast<struct sockaddr*>(&responderAddr),
        &responderAddrLen
    );

    close(sockfd);

    if (bytesReceived <= 0) {
        return false;
    }

    responseBuffer[bytesReceived] = '\0';
    if (std::string(responseBuffer) != kDiscoveryResponse) {
        return false;
    }

    char ipBuffer[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &responderAddr.sin_addr, ipBuffer, sizeof(ipBuffer)) == nullptr) {
        return false;
    }

    discoveredIp = ipBuffer;
    return true;
}

std::string resolveServerIp(bool& usedFallback)
{
    const std::string configuredIp = readEnvString(kServerIpEnv);
    if (!configuredIp.empty()) {
        usedFallback = false;
        std::cout << "Using SonicSole server IP from " << kServerIpEnv
                  << ": " << configuredIp << std::endl;
        return configuredIp;
    }

    const std::string configuredHost = readEnvString(kServerHostEnv);
    if (!configuredHost.empty()) {
        std::string resolvedIp;
        if (resolveHostnameToIpv4(configuredHost, resolvedIp)) {
            usedFallback = false;
            std::cout << "Resolved SonicSole server host " << configuredHost
                      << " to " << resolvedIp << std::endl;
            return resolvedIp;
        }

        std::cout << "Could not resolve SonicSole server host " << configuredHost
                  << ", trying UDP discovery" << std::endl;
    }

    std::string discoveredIp;
    if (discoverServerIp(discoveredIp)) {
        usedFallback = false;
        std::cout << "Discovered SonicSole server at " << discoveredIp << std::endl;
        return discoveredIp;
    }

    usedFallback = true;
    std::cout << "Falling back to legacy SonicSole server IP "
              << kLegacyServerIp << std::endl;
    return kLegacyServerIp;
}

bool configureServerAddress(struct sockaddr_in& serverAddr, int port)
{
    static CachedUdpTarget cachedTarget;

    const auto now = std::chrono::steady_clock::now();
    if (cachedTarget.ipAddress.empty() ||
        (cachedTarget.usingFallback &&
         (now - cachedTarget.lastResolveAttempt) >= std::chrono::seconds(kResolveRetrySeconds))) {
        cachedTarget.ipAddress = resolveServerIp(cachedTarget.usingFallback);
        cachedTarget.lastResolveAttempt = now;
    }

    if (!fillServerAddress(cachedTarget.ipAddress, port, serverAddr)) {
        std::cerr << "Invalid SonicSole server IP: " << cachedTarget.ipAddress << std::endl;
        cachedTarget.ipAddress.clear();
        return false;
    }

    return true;
}

void sendUdpArray(
    int sockfd,
    const float* data,
    std::size_t numElements,
    const struct sockaddr_in& serverAddr
)
{
    const std::size_t numBytes = numElements * sizeof(float);

    const ssize_t sent = sendto(
        sockfd,
        data,
        numBytes,
        0,
        reinterpret_cast<const struct sockaddr*>(&serverAddr),
        sizeof(serverAddr)
    );

    if (sent == -1) {
        std::cerr << "UDPSend error: Failed to send data" << std::endl;
    }
}

int getPersistentSendSocket()
{
    static int cachedSockfd = -1;
    if (cachedSockfd >= 0) {
        return cachedSockfd;
    }

    cachedSockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (cachedSockfd == -1) {
        std::cerr << "Error creating persistent UDP send socket" << std::endl;
        return -1;
    }
    return cachedSockfd;
}

} // namespace

void SonicSole::sendSensorData(float flexSensorData[], int port, std::size_t numElements)
{
    const int sockfd = getPersistentSendSocket();
    if (sockfd < 0) {
        return;
    }

    struct sockaddr_in serverAddr;
    if (!configureServerAddress(serverAddr, port)) {
        return;
    }

    try {
        sendUdpArray(sockfd, flexSensorData, numElements, serverAddr);
    } catch (...) {
        std::cout << "Error: UDPSend cannot send data" << std::endl;
    }
}

void SonicSole::sendFlexSensorData(int flexSensorData, int port)
{
    const int sockfd = getPersistentSendSocket();
    if (sockfd < 0) {
        return;
    }

    struct sockaddr_in serverAddr;
    if (!configureServerAddress(serverAddr, port)) {
        return;
    }

    try {
        const float flexSensorDataFloat = static_cast<float>(flexSensorData);
        sendUdpArray(sockfd, &flexSensorDataFloat, 1, serverAddr);
    } catch (...) {
        std::cout << "Error: UDPSend cannot send data" << std::endl;
    }
}
