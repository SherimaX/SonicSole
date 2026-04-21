#ifndef SONICSOLE_AUDIO_H
#define SONICSOLE_AUDIO_H

#include <cstdint>
#include <string>

// Lightweight WAV player built on `aplay`.
//
// We fork + execvp `aplay` so we don't depend on libasound dev headers
// being installable on the target (Raspbian Buster's repositories are
// EOL, and matching `libasound2-dev` is hard to get onto those images).
// The audible lag is dominated by aplay's startup (~50-150 ms on a
// Pi 3/4). For the reaction-game use case this is a constant per-run
// bias rather than a precision issue.
class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;

    // Validates that `wavPath` exists and remembers the paired ALSA
    // device (passed through as `aplay -D <device>`).
    bool loadAndOpen(const std::string& wavPath, const std::string& alsaDevice);

    void close();
    bool isOpen() const { return open_; }

    // Forks aplay asynchronously. Returns the micro-timestamp captured
    // just before forking, so the caller can anchor a stopwatch against
    // the moment we dispatched the beep.
    std::uint64_t play();

    const std::string& lastError() const { return lastError_; }

private:
    std::string wavPath_;
    std::string alsaDevice_ = "default";
    bool open_ = false;
    std::string lastError_;
};

#endif // SONICSOLE_AUDIO_H
