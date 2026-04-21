#ifndef SONICSOLE_AUDIO_H
#define SONICSOLE_AUDIO_H

#include <cstdint>
#include <string>
#include <vector>

// Low-latency WAV player backed by a long-running `aplay` child.
//
// Approach:
//   1. Parse the WAV file once at startup into raw PCM + format metadata.
//   2. Fork `aplay -q -t raw ...` reading from a pipe we keep open.
//   3. Pre-warm the pipe with a short silence so the ALSA device opens
//      (and its click/pop fires) before the participant is listening.
//   4. play() just writes the cached PCM bytes into the pipe — no
//      fork/exec, no WAV parse, no ALSA open on the hot path.
//
// Compared with fork+exec'ing aplay on every shot this cuts audible
// latency from ~50-150 ms down to a few ms, and removes the startup
// click that was landing right before the real beep.
class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    AudioPlayer(const AudioPlayer&) = delete;
    AudioPlayer& operator=(const AudioPlayer&) = delete;

    // Loads `wavPath` into memory and spawns the persistent aplay
    // worker. After this returns true, the ALSA device is already open
    // and a short silence has been pushed through it.
    bool loadAndOpen(const std::string& wavPath, const std::string& alsaDevice);

    // Shuts down the aplay worker; the child drains whatever is in
    // flight and then exits when it sees EOF on stdin.
    void close();

    bool isOpen() const { return aplayStdin_ >= 0; }

    // Non-blocking. Returns the micro-timestamp captured just before we
    // hand off the PCM bytes to the background writer thread.
    std::uint64_t play();

    const std::string& lastError() const { return lastError_; }

private:
    bool parseWav(const std::string& path);
    bool spawnAplay(const std::string& alsaDevice);
    // Writes `len` bytes into the aplay stdin pipe. Returns number of
    // bytes actually written; -1 on fatal error.
    long blockingWrite(const std::uint8_t* data, std::size_t len);

    std::vector<std::uint8_t> pcmData_;
    unsigned int channels_ = 2;
    unsigned int sampleRate_ = 44100;
    unsigned int bitsPerSample_ = 16;

    int aplayStdin_ = -1;
    int aplayPid_ = -1;

    std::string lastError_;
};

#endif // SONICSOLE_AUDIO_H
