#include "SonicSole_Audio.h"
#include "SonicSole.h"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {

void ignoreChildExits()
{
    // SA_NOCLDWAIT lets the aplay child be auto-reaped on exit so we
    // don't need a dedicated waiter. (We don't care about the status.)
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sa.sa_flags = SA_NOCLDWAIT;
    sigaction(SIGCHLD, &sa, nullptr);
}

bool loadPcmWav(
    const std::string& path,
    std::vector<std::uint8_t>& outSamples,
    unsigned int& outChannels,
    unsigned int& outSampleRate,
    unsigned int& outBitsPerSample,
    std::string& outError)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        outError = "cannot open " + path;
        return false;
    }

    char riff[4] = {};
    char wave[4] = {};
    std::uint32_t riffSize = 0;
    file.read(riff, 4);
    file.read(reinterpret_cast<char*>(&riffSize), 4);
    file.read(wave, 4);
    if (std::memcmp(riff, "RIFF", 4) != 0 || std::memcmp(wave, "WAVE", 4) != 0) {
        outError = "not a RIFF/WAVE file";
        return false;
    }

    bool fmtFound = false;
    bool dataFound = false;
    std::uint16_t audioFormat = 0;
    std::uint16_t channels = 0;
    std::uint32_t sampleRate = 0;
    std::uint16_t bitsPerSample = 0;

    while (file && !dataFound) {
        char chunkId[4] = {};
        std::uint32_t chunkBytes = 0;
        if (!file.read(chunkId, 4)) break;
        if (!file.read(reinterpret_cast<char*>(&chunkBytes), 4)) break;

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            std::uint16_t blockAlign = 0;
            std::uint32_t byteRate = 0;
            file.read(reinterpret_cast<char*>(&audioFormat), 2);
            file.read(reinterpret_cast<char*>(&channels), 2);
            file.read(reinterpret_cast<char*>(&sampleRate), 4);
            file.read(reinterpret_cast<char*>(&byteRate), 4);
            file.read(reinterpret_cast<char*>(&blockAlign), 2);
            file.read(reinterpret_cast<char*>(&bitsPerSample), 2);
            if (chunkBytes > 16) {
                file.seekg(chunkBytes - 16, std::ios::cur);
            }
            fmtFound = true;
        } else if (std::memcmp(chunkId, "data", 4) == 0) {
            outSamples.resize(chunkBytes);
            file.read(reinterpret_cast<char*>(outSamples.data()), chunkBytes);
            dataFound = true;
        } else {
            file.seekg(chunkBytes, std::ios::cur);
        }
    }

    if (!fmtFound) { outError = "missing fmt chunk"; return false; }
    if (!dataFound) { outError = "missing data chunk"; return false; }
    if (audioFormat != 1) {
        std::ostringstream message;
        message << "unsupported audioFormat=" << audioFormat << " (PCM only)";
        outError = message.str();
        return false;
    }
    if (bitsPerSample != 16 && bitsPerSample != 8) {
        std::ostringstream message;
        message << "unsupported bitsPerSample=" << bitsPerSample;
        outError = message.str();
        return false;
    }

    outChannels = channels;
    outSampleRate = sampleRate;
    outBitsPerSample = bitsPerSample;
    return true;
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

bool AudioPlayer::parseWav(const std::string& path)
{
    std::string err;
    if (!loadPcmWav(path, pcmData_, channels_, sampleRate_, bitsPerSample_, err)) {
        lastError_ = "loadPcmWav: " + err;
        return false;
    }
    return true;
}

bool AudioPlayer::spawnAplay(const std::string& alsaDevice)
{
    int pipeFds[2] = {-1, -1};
    if (pipe(pipeFds) < 0) {
        lastError_ = std::string("pipe: ") + std::strerror(errno);
        return false;
    }

    const std::string fmtStr = (bitsPerSample_ == 16) ? "S16_LE" : "U8";
    const std::string chStr = std::to_string(channels_);
    const std::string rateStr = std::to_string(sampleRate_);
    const std::string device = alsaDevice.empty() ? std::string("default") : alsaDevice;

    const pid_t pid = fork();
    if (pid < 0) {
        ::close(pipeFds[0]);
        ::close(pipeFds[1]);
        lastError_ = std::string("fork: ") + std::strerror(errno);
        return false;
    }

    if (pid == 0) {
        // Child: aplay reads raw PCM from our pipe.
        if (dup2(pipeFds[0], STDIN_FILENO) < 0) {
            _exit(126);
        }
        ::close(pipeFds[0]);
        ::close(pipeFds[1]);

        const int devnull = ::open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            ::close(devnull);
        }

        // Small buffer keeps aplay's own internal latency tight.
        execlp(
            "aplay",
            "aplay",
            "-q",
            "-t", "raw",
            "-c", chStr.c_str(),
            "-r", rateStr.c_str(),
            "-f", fmtStr.c_str(),
            "-D", device.c_str(),
            "--buffer-time=50000",
            "--period-time=10000",
            static_cast<char*>(nullptr)
        );
        _exit(127);
    }

    ::close(pipeFds[0]);
    aplayStdin_ = pipeFds[1];
    aplayPid_ = pid;
    return true;
}

bool AudioPlayer::loadAndOpen(const std::string& wavPath, const std::string& alsaDevice)
{
    close();

    if (!parseWav(wavPath)) {
        std::cerr << "[Audio] " << lastError_ << std::endl;
        return false;
    }
    if (!spawnAplay(alsaDevice)) {
        std::cerr << "[Audio] " << lastError_ << std::endl;
        return false;
    }

    std::cout << "[Audio] Persistent aplay ready: " << wavPath
              << " (" << sampleRate_ << " Hz, " << channels_ << " ch, "
              << bitsPerSample_ << "-bit, " << pcmData_.size() << " PCM bytes) on "
              << (alsaDevice.empty() ? "default" : alsaDevice) << std::endl;

    // Pre-warm: ~200 ms of silence so aplay opens the ALSA device and
    // its startup click fires before the first reaction shot. Without
    // this, the first play() landed a faint click before the beep.
    const unsigned int frameBytes = channels_ * (bitsPerSample_ / 8);
    if (frameBytes > 0) {
        const std::size_t warmBytes =
            (static_cast<std::size_t>(sampleRate_) * frameBytes) / 5;
        std::vector<std::uint8_t> silence(warmBytes, 0);
        blockingWrite(silence.data(), silence.size());
    }
    return true;
}

long AudioPlayer::blockingWrite(const std::uint8_t* data, std::size_t len)
{
    if (aplayStdin_ < 0) return -1;
    std::size_t written = 0;
    while (written < len) {
        const ssize_t n = ::write(aplayStdin_, data + written, len - written);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            lastError_ = std::string("write: ") + std::strerror(errno);
            return -1;
        }
        written += static_cast<std::size_t>(n);
    }
    return static_cast<long>(written);
}

std::uint64_t AudioPlayer::play()
{
    const std::uint64_t ts = getMicrosTimeStamp();
    if (aplayStdin_ < 0 || pcmData_.empty()) {
        return ts;
    }

    // Copy fd + PCM into the writer thread so close() is always safe.
    const int fdCopy = aplayStdin_;
    std::vector<std::uint8_t> dataCopy = pcmData_;
    std::thread(
        [fdCopy, dataCopy = std::move(dataCopy)]() {
            std::size_t written = 0;
            while (written < dataCopy.size()) {
                const ssize_t n = ::write(
                    fdCopy,
                    dataCopy.data() + written,
                    dataCopy.size() - written);
                if (n < 0) {
                    if (errno == EINTR || errno == EAGAIN) continue;
                    std::cerr << "[Audio] write failed: "
                              << std::strerror(errno) << std::endl;
                    return;
                }
                written += static_cast<std::size_t>(n);
            }
        }
    ).detach();

    return ts;
}

void AudioPlayer::close()
{
    if (aplayStdin_ >= 0) {
        ::close(aplayStdin_);
        aplayStdin_ = -1;
    }
    // Child is auto-reaped because of SA_NOCLDWAIT; it will exit when
    // it hits EOF on stdin (stdin closed above).
    aplayPid_ = -1;
    pcmData_.clear();
}
