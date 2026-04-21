#include "SonicSole_Audio.h"
#include "SonicSole.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

bool fileExists(const std::string& path)
{
    std::ifstream probe(path);
    return probe.good();
}

void ignoreChildExits()
{
    // Reap children automatically so fire-and-forget `aplay` calls don't
    // leave zombie processes hanging around between rounds.
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sa.sa_flags = SA_NOCLDWAIT;
    sigaction(SIGCHLD, &sa, nullptr);
}

} // namespace

AudioPlayer::AudioPlayer()
{
    ignoreChildExits();
}

AudioPlayer::~AudioPlayer()
{
    close();
}

bool AudioPlayer::loadAndOpen(const std::string& wavPath, const std::string& alsaDevice)
{
    if (!fileExists(wavPath)) {
        lastError_ = "cannot open " + wavPath;
        std::cerr << "[Audio] " << lastError_ << std::endl;
        open_ = false;
        return false;
    }

    wavPath_ = wavPath;
    alsaDevice_ = alsaDevice.empty() ? "default" : alsaDevice;
    open_ = true;

    std::cout << "[Audio] aplay backend ready: " << wavPath_
              << " on device " << alsaDevice_ << std::endl;
    return true;
}

void AudioPlayer::close()
{
    open_ = false;
}

std::uint64_t AudioPlayer::play()
{
    const std::uint64_t dispatchedAt = getMicrosTimeStamp();
    if (!open_) {
        return dispatchedAt;
    }

    const pid_t pid = fork();
    if (pid < 0) {
        lastError_ = std::string("fork failed: ") + std::strerror(errno);
        std::cerr << "[Audio] " << lastError_ << std::endl;
        return dispatchedAt;
    }

    if (pid == 0) {
        // Child: silence aplay's stdout/stderr and exec it.
        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            ::close(devnull);
        }
        execlp(
            "aplay",
            "aplay",
            "-q",
            "-D", alsaDevice_.c_str(),
            wavPath_.c_str(),
            static_cast<char*>(nullptr)
        );
        _exit(127);
    }

    // Parent: don't wait. SIGCHLD is ignored with SA_NOCLDWAIT so the child
    // won't become a zombie.
    return dispatchedAt;
}
