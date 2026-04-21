#ifndef SONICSOLE_ACTIVITY_H
#define SONICSOLE_ACTIVITY_H

#include "SonicSole.h"

#include <cstdint>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <vector>

struct ActivityResult {
    bool success = false;
    std::string activityName;
    double score = 0.0;
    std::string reason;
};

class ActivityBase {
public:
    virtual ~ActivityBase() = default;
    virtual const std::string& name() const = 0;
    virtual void onStart(SonicSole& sole) = 0;
    virtual bool onStep(SonicSole& sole, ActivityResult& result) = 0;
    virtual void onCancel(SonicSole& sole, ActivityResult& result) = 0;
};

class BalanceActivity : public ActivityBase {
public:
    BalanceActivity();
    const std::string& name() const override { return kName; }
    void onStart(SonicSole& sole) override;
    bool onStep(SonicSole& sole, ActivityResult& result) override;
    void onCancel(SonicSole& sole, ActivityResult& result) override;

    int threshold = 100;
    double liftWaitSec = 30.0;
    double maxDurationSec = 120.0;

private:
    enum class Phase { WaitLift, Timing };
    const std::string kName = "balance";
    Phase phase_ = Phase::WaitLift;
    std::uint64_t phaseStartMicros_ = 0;
    std::uint64_t liftMicros_ = 0;
};

class ActivityManager {
public:
    ActivityManager();
    ~ActivityManager();

    ActivityManager(const ActivityManager&) = delete;
    ActivityManager& operator=(const ActivityManager&) = delete;

    void registerActivity(std::unique_ptr<ActivityBase> activity);
    bool start(int listenPort);
    void stop();
    void tick(SonicSole& sole);

    bool hasActiveActivity() const { return currentActivity_ != nullptr; }
    const std::string& activeActivityName() const;

private:
    int listenSock_ = -1;
    int listenPort_ = 0;

    std::vector<std::unique_ptr<ActivityBase>> activities_;
    ActivityBase* currentActivity_ = nullptr;
    struct sockaddr_in currentClient_ {};
    bool hasClient_ = false;

    ActivityBase* findActivity(const std::string& name);
    void pollCommands(SonicSole& sole);
    void sendResult(const ActivityResult& result, const struct sockaddr_in& client);
    void handleStart(SonicSole& sole, const std::string& activityName, const struct sockaddr_in& sender);
    void handleCancel(SonicSole& sole, const std::string& activityName, const struct sockaddr_in& sender);
    void handlePing(const std::string& activityName, const struct sockaddr_in& sender);
};

#endif // SONICSOLE_ACTIVITY_H
