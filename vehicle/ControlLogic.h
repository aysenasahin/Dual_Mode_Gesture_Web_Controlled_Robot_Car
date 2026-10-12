#pragma once
#include <cstdint>

namespace robot {
enum class Motion { Stop, Forward, Reverse, Left, Right };
enum class Mode { Web, Gesture };

inline bool decodeCommand(char input, Motion &result) {
    switch (input) {
        case 'S': result = Motion::Stop; return true;
        case 'F': result = Motion::Forward; return true;
        case 'B': result = Motion::Reverse; return true;
        case 'L': result = Motion::Left; return true;
        case 'R': result = Motion::Right; return true;
        default: return false;
    }
}
inline char encodeCommand(Motion value) {
    switch (value) {
        case Motion::Forward: return 'F';
        case Motion::Reverse: return 'B';
        case Motion::Left: return 'L';
        case Motion::Right: return 'R';
        default: return 'S';
    }
}
struct WheelDirections { int left; int right; };
inline WheelDirections wheelDirections(Motion value) {
    switch (value) {
        case Motion::Forward: return {1, 1};
        case Motion::Reverse: return {-1, -1};
        case Motion::Left: return {0, 1};
        case Motion::Right: return {1, 0};
        default: return {0, 0};
    }
}
class Controller {
public:
    explicit Controller(std::uint32_t timeoutMs = 500) : timeoutMs_(timeoutMs) {}
    Mode mode() const { return mode_; }
    Motion motion() const { return motion_; }
    bool halted() const { return halted_; }
    bool needsNeutral() const { return mode_ == Mode::Gesture && !gestureArmed_; }
    std::uint32_t modeVersion() const { return modeVersion_; }

    // Only the physical button calls this in the vehicle firmware.
    void toggleMode() { selectMode(mode_ == Mode::Web ? Mode::Gesture : Mode::Web); }
    void selectMode(Mode selected) {
        mode_ = selected;
        motion_ = Motion::Stop;
        halted_ = false;
        gestureArmed_ = selected == Mode::Web;
        ++modeVersion_;
    }
    // The web STOP button latches a stop in either mode.
    // A physical mode-button press is required to clear the latch.
    void halt() { halted_ = true; motion_ = Motion::Stop; }

    bool accept(Mode source, Motion value, std::uint32_t now) {
        if (source != mode_ || halted_) return false;
        if (mode_ == Mode::Gesture && !gestureArmed_) {
            if (value != Motion::Stop) return false;
            gestureArmed_ = true;
        }
        motion_ = value;
        lastCommandMs_ = now;
        return true;
    }
    void tick(std::uint32_t now) {
        if (motion_ != Motion::Stop &&
            static_cast<std::uint32_t>(now - lastCommandMs_) >= timeoutMs_)
            motion_ = Motion::Stop;
    }
private:
    Mode mode_ = Mode::Web;
    Motion motion_ = Motion::Stop;
    bool halted_ = false;
    bool gestureArmed_ = true;
    std::uint32_t modeVersion_ = 0;
    std::uint32_t lastCommandMs_ = 0;
    std::uint32_t timeoutMs_;
};
// Active-low momentary button: one event per debounced press, never per hold.
class DebouncedButton {
public:
    explicit DebouncedButton(std::uint32_t debounceMs = 40) : debounceMs_(debounceMs) {}
    bool update(bool pressed, std::uint32_t now) {
        if (pressed != raw_) { raw_ = pressed; changedAt_ = now; }
        if (raw_ != stable_ && static_cast<std::uint32_t>(now - changedAt_) >= debounceMs_) {
            stable_ = raw_;
            return stable_;
        }
        return false;
    }
private:
    bool raw_ = false;
    bool stable_ = false;
    std::uint32_t changedAt_ = 0;
    std::uint32_t debounceMs_;
};
// UART protocol: one ASCII command + LF; CRLF is accepted too.
class CommandParser {
public:
    bool feed(char input, Motion &result) {
        if (input == '\r') return false;
        if (input == '\n') {
            const bool valid = length_ == 1 && !overflow_ && decodeCommand(first_, result);
            reset();
            return valid;
        }
        if (length_ == 0) first_ = input;
        if (length_ < 2) ++length_;
        else overflow_ = true;
        return false;
    }
    void reset() { first_ = 0; length_ = 0; overflow_ = false; }
private:
    char first_ = 0;
    unsigned int length_ = 0;
    bool overflow_ = false;
};
} // namespace robot
