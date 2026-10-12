#pragma once
#include <cmath>

namespace robot {
enum class Motion { Stop, Forward, Reverse, Left, Right };

inline char encodeCommand(Motion value) {
    switch (value) {
        case Motion::Forward: return 'F';
        case Motion::Reverse: return 'B';
        case Motion::Left: return 'L';
        case Motion::Right: return 'R';
        default: return 'S';
    }
}
// Components in g; the larger absolute component wins diagonal gestures.
// Threshold and axis signs are configured for the wearable sensor orientation.
inline Motion classifyTilt(float forwardG, float rightG, float deadzoneG = 0.30f) {
    if (!std::isfinite(forwardG) || !std::isfinite(rightG) ||
        !std::isfinite(deadzoneG) || deadzoneG <= 0) return Motion::Stop;
    if (forwardG > deadzoneG && forwardG >= std::fabs(rightG)) return Motion::Forward;
    if (forwardG < -deadzoneG && -forwardG >= std::fabs(rightG)) return Motion::Reverse;
    if (rightG > deadzoneG) return Motion::Right;
    if (rightG < -deadzoneG) return Motion::Left;
    return Motion::Stop;
}
} // namespace robot
