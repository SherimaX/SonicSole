#ifndef SONICSOLE_AUDIO_H
#define SONICSOLE_AUDIO_H

#include <alsa/asoundlib.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Low-latency WAV player that keeps an ALSA PCM handle open and pre-loads
// PCM samples into memory. play() returns immediately — a background
// worker thread writes the samples to the device.
//
// Designed for the reaction-game use case where we need the gap between
// "hit play" and "user hears the tone" to be short and predictable.
class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;

    // Loads `wavPath` into memory and opens the ALSA PCM device.
    // `alsaDevice` is the ALSA device name (e.g. "default" or "plughw:0,0").
    bool loadAndOpen(const std::string& wavPath, const std::string& alsaDevice);

    // Closes the ALSA device and stops the worker thread.
    void close();

    bool isOpen() const { return pcm_ != nullptr; }

    // Returns a timestamp (microseconds) captured immediately before
    // signalling the worker thread, and kicks off playback asynchronously.
    std::uint64_t play();

    const std::string& lastError() const { return lastError_; }

private:
    void workerLoop();
    bool writeSamplesBlocking();

    snd_pcm_t* pcm_ = nullptr;
    std::vector<std::uint8_t> samples_;
    unsigned int channels_ = 0;
    unsigned int sampleRate_ = 0;
    unsigned int bitsPerSample_ = 0;
    snd_pcm_format_t format_ = SND_PCM_FORMAT_UNKNOWN;
    snd_pcm_uframes_t frames_ = 0;

    std::thread worker_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stopping_{false};
    bool playPending_ = false;

    std::string lastError_;
};

#endif // SONICSOLE_AUDIO_H
