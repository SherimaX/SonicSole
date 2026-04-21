#include "SonicSole_Activity.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr std::size_t kCommandBufferSize = 512;
constexpr int kMaxCommandsPerTick = 8;

std::string trimCopy(const std::string& input)
{
    auto isNotSpace = [](unsigned char ch) { return !std::isspace(ch); };
    const auto beginIt = std::find_if(input.begin(), input.end(), isNotSpace);
    const auto endIt = std::find_if(input.rbegin(), input.rend(), isNotSpace).base();
    if (beginIt >= endIt) {
        return {};
    }
    return std::string(beginIt, endIt);
}

std::vector<std::string> splitWhitespace(const std::string& input)
{
    std::vector<std::string> tokens;
    std::istringstream stream(input);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string upper(const std::string& input)
{
    std::string output = input;
    for (char& ch : output) {
        ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return output;
}

int currentPeakPressure(const SonicSole& sole)
{
    const int heel = static_cast<int>(sole.currHeelPressure);
    const int fore = static_cast<int>(sole.currForePressure);
    return heel > fore ? heel : fore;
}

int readEnvInt(const char* name, int fallback)
{
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* endPtr = nullptr;
    const long parsed = std::strtol(value, &endPtr, 10);
    if (endPtr == value || *endPtr != '\0') {
        return fallback;
    }
    return static_cast<int>(parsed);
}

double readEnvDouble(const char* name, double fallback)
{
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') {
        return fallback;
    }
    char* endPtr = nullptr;
    const double parsed = std::strtod(value, &endPtr);
    if (endPtr == value || *endPtr != '\0') {
        return fallback;
    }
    return parsed;
}

std::string formatClient(const struct sockaddr_in& addr)
{
    char buffer[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &addr.sin_addr, buffer, sizeof(buffer));
    std::ostringstream stream;
    stream << buffer << ':' << ntohs(addr.sin_port);
    return stream.str();
}

} // namespace

// BalanceActivity ---------------------------------------------------------

BalanceActivity::BalanceActivity()
{
    threshold = readEnvInt("SONICSOLE_BALANCE_THRESHOLD", threshold);
    liftWaitSec = readEnvDouble("SONICSOLE_BALANCE_LIFT_WAIT", liftWaitSec);
    maxDurationSec = readEnvDouble("SONICSOLE_BALANCE_MAX_DURATION", maxDurationSec);
}

void BalanceActivity::onStart(SonicSole& /*sole*/)
{
    phase_ = Phase::WaitLift;
    phaseStartMicros_ = getMicrosTimeStamp();
    liftMicros_ = 0;
    std::cout << "[Activity] balance started, waiting for foot lift"
              << " (threshold=" << threshold
              << ", lift_wait=" << liftWaitSec << "s"
              << ", max_duration=" << maxDurationSec << "s)"
              << std::endl;
}

bool BalanceActivity::onStep(SonicSole& sole, ActivityResult& result)
{
    const int peak = currentPeakPressure(sole);
    const std::uint64_t nowMicros = getMicrosTimeStamp();

    if (phase_ == Phase::WaitLift) {
        const double elapsedSec = static_cast<double>(nowMicros - phaseStartMicros_) / 1e6;
        if (peak < threshold) {
            liftMicros_ = nowMicros;
            phase_ = Phase::Timing;
            std::cout << "[Activity] balance: foot lifted after "
                      << elapsedSec << "s, timing" << std::endl;
            return false;
        }
        if (elapsedSec > liftWaitSec) {
            result.activityName = name();
            result.success = false;
            result.reason = "lift_timeout";
            std::cout << "[Activity] balance: lift timeout after "
                      << elapsedSec << "s" << std::endl;
            return true;
        }
        return false;
    }

    const double elapsedSec = static_cast<double>(nowMicros - liftMicros_) / 1e6;
    if (peak >= threshold) {
        result.activityName = name();
        result.success = true;
        result.score = elapsedSec;
        result.reason.clear();
        std::cout << "[Activity] balance: foot replanted, score="
                  << elapsedSec << "s (peak=" << peak << ")" << std::endl;
        return true;
    }
    if (elapsedSec >= maxDurationSec) {
        result.activityName = name();
        result.success = true;
        result.score = elapsedSec;
        result.reason = "max_duration";
        std::cout << "[Activity] balance: max duration reached, score="
                  << elapsedSec << "s" << std::endl;
        return true;
    }
    return false;
}

void BalanceActivity::onCancel(SonicSole& /*sole*/, ActivityResult& result)
{
    result.activityName = name();
    result.success = false;
    result.reason = "cancelled";
    phase_ = Phase::WaitLift;
}

// ActivityManager ---------------------------------------------------------

ActivityManager::ActivityManager() = default;

ActivityManager::~ActivityManager()
{
    stop();
}

const std::string& ActivityManager::activeActivityName() const
{
    static const std::string empty;
    return currentActivity_ != nullptr ? currentActivity_->name() : empty;
}

void ActivityManager::registerActivity(std::unique_ptr<ActivityBase> activity)
{
    if (!activity) {
        return;
    }
    if (findActivity(activity->name()) != nullptr) {
        std::cerr << "[Activity] Ignoring duplicate registration for '"
                  << activity->name() << "'" << std::endl;
        return;
    }
    std::cout << "[Activity] Registered activity '" << activity->name() << "'" << std::endl;
    activities_.push_back(std::move(activity));
}

bool ActivityManager::start(int listenPort)
{
    stop();
    listenPort_ = listenPort;

    listenSock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (listenSock_ == -1) {
        std::cerr << "[Activity] Failed to create control socket: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    int reuse = 1;
    setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    const int flags = fcntl(listenSock_, F_GETFL, 0);
    if (flags == -1 || fcntl(listenSock_, F_SETFL, flags | O_NONBLOCK) == -1) {
        std::cerr << "[Activity] Failed to set non-blocking control socket: "
                  << std::strerror(errno) << std::endl;
        close(listenSock_);
        listenSock_ = -1;
        return false;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(listenPort));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(listenSock_, reinterpret_cast<const struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        std::cerr << "[Activity] Failed to bind control socket on port "
                  << listenPort << ": " << std::strerror(errno) << std::endl;
        close(listenSock_);
        listenSock_ = -1;
        return false;
    }

    std::cout << "[Activity] Listening for commands on UDP " << listenPort << std::endl;
    return true;
}

void ActivityManager::stop()
{
    if (listenSock_ != -1) {
        close(listenSock_);
        listenSock_ = -1;
    }
    currentActivity_ = nullptr;
    hasClient_ = false;
}

ActivityBase* ActivityManager::findActivity(const std::string& name)
{
    for (auto& activity : activities_) {
        if (activity->name() == name) {
            return activity.get();
        }
    }
    return nullptr;
}

void ActivityManager::tick(SonicSole& sole)
{
    pollCommands(sole);

    if (currentActivity_ == nullptr) {
        return;
    }

    ActivityResult result;
    if (currentActivity_->onStep(sole, result)) {
        if (hasClient_) {
            sendResult(result, currentClient_);
        }
        currentActivity_ = nullptr;
        hasClient_ = false;
    }
}

void ActivityManager::pollCommands(SonicSole& sole)
{
    if (listenSock_ == -1) {
        return;
    }

    char buffer[kCommandBufferSize];
    for (int i = 0; i < kMaxCommandsPerTick; ++i) {
        struct sockaddr_in sender {};
        socklen_t senderLen = sizeof(sender);
        const ssize_t received = recvfrom(
            listenSock_, buffer, sizeof(buffer) - 1, 0,
            reinterpret_cast<struct sockaddr*>(&sender), &senderLen);
        if (received <= 0) {
            return;
        }

        buffer[received] = '\0';
        const std::string command = trimCopy(std::string(buffer, static_cast<std::size_t>(received)));
        if (command.empty()) {
            continue;
        }

        std::cout << "[Activity] Received command '" << command
                  << "' from " << formatClient(sender) << std::endl;

        const auto tokens = splitWhitespace(command);
        if (tokens.empty()) {
            continue;
        }

        const std::string verb = upper(tokens[0]);
        const std::string activityName = tokens.size() >= 2 ? tokens[1] : std::string();

        if (verb == "START") {
            if (activityName.empty()) {
                ActivityResult err;
                err.activityName = "unknown";
                err.reason = "missing_activity";
                sendResult(err, sender);
                continue;
            }
            handleStart(sole, activityName, sender);
        } else if (verb == "CANCEL" || verb == "STOP") {
            handleCancel(sole, activityName, sender);
        } else if (verb == "PING") {
            handlePing(activityName, sender);
        } else {
            ActivityResult err;
            err.activityName = activityName.empty() ? "unknown" : activityName;
            err.reason = "unknown_verb";
            sendResult(err, sender);
        }
    }
}

void ActivityManager::handleStart(
    SonicSole& sole,
    const std::string& activityName,
    const struct sockaddr_in& sender)
{
    ActivityBase* activity = findActivity(activityName);
    if (activity == nullptr) {
        ActivityResult err;
        err.activityName = activityName;
        err.reason = "unknown_activity";
        sendResult(err, sender);
        return;
    }

    if (currentActivity_ != nullptr) {
        ActivityResult cancelResult;
        currentActivity_->onCancel(sole, cancelResult);
        if (hasClient_) {
            sendResult(cancelResult, currentClient_);
        }
    }

    currentActivity_ = activity;
    currentClient_ = sender;
    hasClient_ = true;
    activity->onStart(sole);
}

void ActivityManager::handleCancel(
    SonicSole& sole,
    const std::string& activityName,
    const struct sockaddr_in& sender)
{
    if (currentActivity_ == nullptr) {
        ActivityResult err;
        err.activityName = activityName.empty() ? "unknown" : activityName;
        err.reason = "not_running";
        sendResult(err, sender);
        return;
    }

    if (!activityName.empty() && currentActivity_->name() != activityName) {
        ActivityResult err;
        err.activityName = activityName;
        err.reason = "activity_mismatch";
        sendResult(err, sender);
        return;
    }

    ActivityResult runningResult;
    currentActivity_->onCancel(sole, runningResult);
    if (hasClient_) {
        sendResult(runningResult, currentClient_);
    }

    ActivityResult ack;
    ack.activityName = currentActivity_->name();
    ack.success = true;
    ack.reason = "cancelled";
    sendResult(ack, sender);

    currentActivity_ = nullptr;
    hasClient_ = false;
}

void ActivityManager::handlePing(
    const std::string& activityName,
    const struct sockaddr_in& sender)
{
    ActivityResult pong;
    pong.success = true;
    pong.activityName = activityName.empty() ? "ping" : activityName;
    pong.score = 0.0;
    pong.reason = "pong";
    sendResult(pong, sender);
}

void ActivityManager::sendResult(const ActivityResult& result, const struct sockaddr_in& client)
{
    if (listenSock_ == -1) {
        return;
    }

    std::ostringstream message;
    if (result.success) {
        message << "OK " << result.activityName << ' '
                << std::fixed << std::setprecision(4) << result.score;
        if (!result.reason.empty()) {
            message << ' ' << result.reason;
        }
    } else {
        message << "ERR "
                << (result.activityName.empty() ? "unknown" : result.activityName) << ' '
                << (result.reason.empty() ? "unknown" : result.reason);
    }

    const std::string payload = message.str();
    const ssize_t sent = sendto(
        listenSock_, payload.data(), payload.size(), 0,
        reinterpret_cast<const struct sockaddr*>(&client), sizeof(client));

    if (sent == -1) {
        std::cerr << "[Activity] Failed to send result to "
                  << formatClient(client) << ": " << std::strerror(errno) << std::endl;
    } else {
        std::cout << "[Activity] Sent '" << payload << "' to "
                  << formatClient(client) << std::endl;
    }
}
