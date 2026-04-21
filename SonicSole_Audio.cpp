#include "SonicSole_Audio.h"
#include "SonicSole.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {

// Parses a PCM .wav file into `outSamples`. Fills channel / sample rate /
// bits-per-sample metadata. Only 8- or 16-bit integer PCM is supported.
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
    std::uint32_t chunkSize = 0;
    file.read(riff, 4);
    file.read(reinterpret_cast<char*>(&chunkSize), 4);
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
        if (!file.read(chunkId, 4)) {
            break;
        }
        if (!file.read(reinterpret_cast<char*>(&chunkBytes), 4)) {
            break;
        }

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            std::uint32_t fmtRead = 0;
            std::uint16_t blockAlign = 0;
            std::uint32_t byteRate = 0;
            file.read(reinterpret_cast<char*>(&audioFormat), 2);
            file.read(reinterpret_cast<char*>(&channels), 2);
            file.read(reinterpret_cast<char*>(&sampleRate), 4);
            file.read(reinterpret_cast<char*>(&byteRate), 4);
            file.read(reinterpret_cast<char*>(&blockAlign), 2);
            file.read(reinterpret_cast<char*>(&bitsPerSample), 2);
            fmtRead = 16;
            if (chunkBytes > fmtRead) {
                file.seekg(chunkBytes - fmtRead, std::ios::cur);
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

    if (!fmtFound) {
        outError = "missing fmt chunk";
        return false;
    }
    if (!dataFound) {
        outError = "missing data chunk";
        return false;
    }
    if (audioFormat != 1) {
        std::ostringstream message;
        message << "unsupported audioFormat=" << audioFormat << " (only PCM is handled)";
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

AudioPlayer::AudioPlayer() = default;

AudioPlayer::~AudioPlayer()
{
    close();
}

bool AudioPlayer::loadAndOpen(const std::string& wavPath, const std::string& alsaDevice)
{
    close();

    std::string loadError;
    if (!loadPcmWav(wavPath, samples_, channels_, sampleRate_, bitsPerSample_, loadError)) {
        lastError_ = "loadPcmWav: " + loadError;
        std::cerr << "[Audio] " << lastError_ << std::endl;
        return false;
    }

    format_ = (bitsPerSample_ == 16) ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_U8;
    const unsigned int frameBytes = channels_ * (bitsPerSample_ / 8);
    if (frameBytes == 0) {
        lastError_ = "invalid frame size computed from WAV header";
        std::cerr << "[Audio] " << lastError_ << std::endl;
        return false;
    }
    frames_ = samples_.size() / frameBytes;

    int openErr = snd_pcm_open(&pcm_, alsaDevice.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (openErr < 0) {
        lastError_ = std::string("snd_pcm_open(") + alsaDevice + "): " + snd_strerror(openErr);
        std::cerr << "[Audio] " << lastError_ << std::endl;
        pcm_ = nullptr;
        return false;
    }

    snd_pcm_hw_params_t* params = nullptr;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_, params);
    snd_pcm_hw_params_set_access(pcm_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_, params, format_);
    snd_pcm_hw_params_set_channels(pcm_, params, channels_);
    unsigned int requestedRate = sampleRate_;
    snd_pcm_hw_params_set_rate_near(pcm_, params, &requestedRate, nullptr);
    sampleRate_ = requestedRate;

    // Small buffer keeps "write returns" close to "sound is audible".
    snd_pcm_uframes_t bufferFrames = 2048;
    snd_pcm_hw_params_set_buffer_size_near(pcm_, params, &bufferFrames);
    snd_pcm_uframes_t periodFrames = 512;
    snd_pcm_hw_params_set_period_size_near(pcm_, params, &periodFrames, nullptr);

    int paramsErr = snd_pcm_hw_params(pcm_, params);
    if (paramsErr < 0) {
        lastError_ = std::string("snd_pcm_hw_params: ") + snd_strerror(paramsErr);
        std::cerr << "[Audio] " << lastError_ << std::endl;
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
        return false;
    }

    std::cout << "[Audio] Ready: " << wavPath
              << " (" << sampleRate_ << " Hz, "
              << channels_ << " ch, "
              << bitsPerSample_ << "-bit, "
              << frames_ << " frames) on " << alsaDevice
              << std::endl;

    stopping_ = false;
    playPending_ = false;
    worker_ = std::thread(&AudioPlayer::workerLoop, this);
    return true;
}

void AudioPlayer::close()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        playPending_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    if (pcm_ != nullptr) {
        snd_pcm_close(pcm_);
        pcm_ = nullptr;
    }
    samples_.clear();
}

std::uint64_t AudioPlayer::play()
{
    const std::uint64_t ts = getMicrosTimeStamp();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        playPending_ = true;
    }
    cv_.notify_one();
    return ts;
}

void AudioPlayer::workerLoop()
{
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return playPending_ || stopping_.load(); });
        if (stopping_.load()) {
            return;
        }
        playPending_ = false;
        lock.unlock();

        if (!writeSamplesBlocking()) {
            std::cerr << "[Audio] playback failed: " << lastError_ << std::endl;
        }
    }
}

bool AudioPlayer::writeSamplesBlocking()
{
    if (pcm_ == nullptr || samples_.empty() || frames_ == 0) {
        lastError_ = "player not initialised";
        return false;
    }

    int prepErr = snd_pcm_prepare(pcm_);
    if (prepErr < 0) {
        lastError_ = std::string("snd_pcm_prepare: ") + snd_strerror(prepErr);
        return false;
    }

    const unsigned int frameBytes = channels_ * (bitsPerSample_ / 8);
    snd_pcm_uframes_t remainingFrames = frames_;
    const std::uint8_t* cursor = samples_.data();

    while (remainingFrames > 0) {
        snd_pcm_sframes_t written = snd_pcm_writei(pcm_, cursor, remainingFrames);
        if (written == -EPIPE) {
            snd_pcm_prepare(pcm_);
            continue;
        }
        if (written == -EAGAIN) {
            continue;
        }
        if (written < 0) {
            int recoverErr = snd_pcm_recover(pcm_, static_cast<int>(written), 1);
            if (recoverErr < 0) {
                lastError_ = std::string("snd_pcm_recover: ") + snd_strerror(recoverErr);
                return false;
            }
            continue;
        }
        cursor += static_cast<std::size_t>(written) * frameBytes;
        remainingFrames -= static_cast<snd_pcm_uframes_t>(written);
    }

    snd_pcm_drain(pcm_);
    return true;
}
