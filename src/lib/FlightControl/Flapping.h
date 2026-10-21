#pragma once
#include <cmath>
#include <stdint.h>

struct FlappingWing {
    uint8_t output = 1; // One-based physical PWM output.
    uint16_t upUs = 1500, downUs = 1500, parkUs = 1500;
};

struct FlappingSettings {
    bool enabled = false;
    uint8_t throttleChannel = 3, steeringChannel = 1;
    float minHz = 0.5f, maxHz = 2.0f;
    float downstrokePercent = 35.0f, differentialPercent = 20.0f;
    bool reverseSteering = false;
    FlappingWing left, right;
    FlappingSettings() { right.output = 2; }
};

inline bool flappingSettingsValid(const FlappingSettings &s)
{
    if (s.throttleChannel < 1 || s.throttleChannel > 16 || s.steeringChannel < 1 ||
        s.steeringChannel > 16 || s.throttleChannel == s.steeringChannel ||
        !std::isfinite(s.minHz) || !std::isfinite(s.maxHz) || s.minHz < 0.1f ||
        s.maxHz > 10 || s.minHz > s.maxHz || !std::isfinite(s.downstrokePercent) ||
        s.downstrokePercent < 20 || s.downstrokePercent > 50 ||
        !std::isfinite(s.differentialPercent) || s.differentialPercent < 0 || s.differentialPercent > 50 ||
        s.left.output < 1 || s.left.output > 16 || s.right.output < 1 || s.right.output > 16 ||
        s.left.output == s.right.output) return false;
    const FlappingWing wings[] = {s.left, s.right};
    for (const auto &w : wings) {
        if (w.upUs < 500 || w.upUs > 2500 || w.downUs < 500 || w.downUs > 2500 ||
            w.parkUs < 500 || w.parkUs > 2500 || (s.enabled && w.upUs == w.downUs)) return false;
    }
    return true;
}

inline bool flappingSettingsEqual(const FlappingSettings &a, const FlappingSettings &b)
{
    return a.enabled == b.enabled && a.throttleChannel == b.throttleChannel &&
        a.steeringChannel == b.steeringChannel && a.minHz == b.minHz && a.maxHz == b.maxHz &&
        a.downstrokePercent == b.downstrokePercent && a.differentialPercent == b.differentialPercent &&
        a.reverseSteering == b.reverseSteering && a.left.output == b.left.output &&
        a.left.upUs == b.left.upUs && a.left.downUs == b.left.downUs && a.left.parkUs == b.left.parkUs &&
        a.right.output == b.right.output && a.right.upUs == b.right.upUs &&
        a.right.downUs == b.right.downUs && a.right.parkUs == b.right.parkUs;
}

struct FlappingOutputLimits {
    unsigned frequency = 0;
    uint16_t minUs = 1000, maxUs = 2000;
    bool available = false;
};

inline const char *flappingChangeError(bool connected, bool validRc, bool armed,
    unsigned oldThrottleUs, unsigned newThrottleUs, bool parked)
{
    if (!connected) return nullptr;
    if (!validRc) return "Wait for valid RC state before changing flapping";
    if (armed && (oldThrottleUs > 998 || newThrottleUs > 998 || !parked))
        return "Lower throttle to zero and wait for wings to park before changing flapping";
    return nullptr;
}

inline const char *flappingContextError(const FlappingSettings &s, bool armEnabled, unsigned armChannel,
    unsigned wifiChannel, bool driftEnabled, const FlappingOutputLimits *outputs, unsigned count)
{
    if (!flappingSettingsValid(s)) return "Invalid flapping parameters";
    if (!s.enabled) return nullptr;
    if (!armEnabled) return "Flapping requires an arm switch";
    if (driftEnabled) return "Flapping and drift assistance are mutually exclusive";
    if (s.throttleChannel == armChannel || s.steeringChannel == armChannel ||
        s.throttleChannel == wifiChannel || s.steeringChannel == wifiChannel)
        return "Flapping input channel conflicts with ARM or Wi-Fi";
    const FlappingWing wings[] = {s.left, s.right};
    for (const auto &w : wings) {
        if (w.output > count || !outputs[w.output - 1].available) return "Flapping requires two standard PWM servo outputs";
        const auto &o = outputs[w.output - 1];
        if (w.upUs < o.minUs || w.upUs > o.maxUs || w.downUs < o.minUs || w.downUs > o.maxUs ||
            w.parkUs < o.minUs || w.parkUs > o.maxUs) return "Flapping positions exceed PWM output limits";
        if (o.frequency * s.downstrokePercent * 0.01f + 0.0001f < 4 * s.maxHz)
            return "Flapping frequency needs at least four PWM frames per half-stroke";
    }
    return nullptr;
}

// Position commands only; no IMU, radio driver or PWM dependencies.
class FlappingController {
public:
    static float ease(float x) { return (1.0f - std::cos(3.14159265358979323846f * x)) * 0.5f; }
    const FlappingSettings &settings() const { return _settings; }
    bool ownsOutputs() const { return _initialized && (_settings.enabled || _moving); }
    bool parked() const { return !_moving && !_running && !_starting &&
        _left == _settings.left.parkUs && _right == _settings.right.parkUs; }
    uint16_t leftUs() const { return (uint16_t)std::lround(_left); }
    uint16_t rightUs() const { return (uint16_t)std::lround(_right); }
    float frequencyHz() const { return _running ? _frequency : 0; }
    const char *stateReason() const {
        if (_moving) return "parking";
        if (_starting) return "starting";
        if (_running) return "running";
        return _zeroSeen ? "idle" : "waiting_zero";
    }

    void update(const FlappingSettings &requested, uint32_t nowUs, bool allowed, float throttle, float steering)
    {
        if (!_initialized) {
            _settings = requested;
            _left = requested.left.parkUs; _right = requested.right.parkUs;
            _lastUs = nowUs; _initialized = true;
        }
        const uint32_t elapsedUs = nowUs - _lastUs; // unsigned subtraction handles clock wrap
        _lastUs = nowUs;
        const bool changed = !flappingSettingsEqual(_settings, requested);
        const bool wasParked = parked();
        const bool valid = std::isfinite(throttle) && std::isfinite(steering) && elapsedUs <= 100000U;
        if (!allowed || !valid || changed || !requested.enabled) {
            _zeroSeen = false;
            stop(nowUs);
            if (wasParked && parked() && changed) {
                _settings = requested;
                // New output mappings are acquired only after the old pair has parked.
                _left = requested.left.parkUs; _right = requested.right.parkUs;
            }
            return;
        }
        throttle = clamp(throttle, 0, 1); steering = clamp(steering, -1, 1);
        if (throttle <= 0.01f) {
            _zeroSeen = true;
            stop(nowUs);
            return;
        }
        if (!_zeroSeen) { stop(nowUs); return; }
        if (_moving) { stop(nowUs); return; }
        if (!_running && !_starting) {
            _starting = true; _transitionUs = nowUs;
            _fromLeft = _left; _fromRight = _right;
        }
        if (_starting) {
            const float x = clamp((nowUs - _transitionUs) / 300000.0f, 0, 1);
            _left = interpolate(_fromLeft, _settings.left.upUs, x);
            _right = interpolate(_fromRight, _settings.right.upUs, x);
            if (x < 1) return;
            _starting = false; _running = true; _phase = 0;
            latchSteering(steering);
            return;
        }
        _frequency = _settings.minHz + (_settings.maxHz - _settings.minHz) * ((throttle - 0.01f) / 0.99f);
        _phase += elapsedUs * 1e-6f * _frequency;
        if (_phase >= 1) { _phase -= std::floor(_phase); latchSteering(steering); }
        const float down = _settings.downstrokePercent * 0.01f;
        if (_phase < down) {
            _left = interpolate(_settings.left.upUs, _leftDown, _phase / down);
            _right = interpolate(_settings.right.upUs, _rightDown, _phase / down);
        } else {
            _left = interpolate(_leftDown, _settings.left.upUs, (_phase - down) / (1 - down));
            _right = interpolate(_rightDown, _settings.right.upUs, (_phase - down) / (1 - down));
        }
    }

    // Resume PWM testing from its last physical command rather than a stale wing position.
    void suspend(uint16_t left, uint16_t right, uint32_t nowUs)
    {
        _left = left; _right = right; _running = _starting = _moving = _zeroSeen = false;
        _lastUs = nowUs;
    }

private:
    static float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    static float interpolate(float a, float b, float x) { return a + (b - a) * ease(x); }
    void latchSteering(float steering)
    {
        if (_settings.reverseSteering) steering = -steering;
        const float gain = _settings.differentialPercent * 0.01f;
        _leftDown = _settings.left.upUs + (_settings.left.downUs - (float)_settings.left.upUs) * (1 - gain * clamp(-steering, 0, 1));
        _rightDown = _settings.right.upUs + (_settings.right.downUs - (float)_settings.right.upUs) * (1 - gain * clamp(steering, 0, 1));
    }
    void stop(uint32_t nowUs)
    {
        if (_running || _starting || (!_moving && (_left != _settings.left.parkUs || _right != _settings.right.parkUs))) {
            _running = _starting = false; _moving = true; _transitionUs = nowUs;
            _fromLeft = _left; _fromRight = _right;
        }
        if (_moving) {
            const float x = clamp((nowUs - _transitionUs) / 300000.0f, 0, 1);
            _left = interpolate(_fromLeft, _settings.left.parkUs, x);
            _right = interpolate(_fromRight, _settings.right.parkUs, x);
            if (x >= 1) _moving = false;
        }
    }
    FlappingSettings _settings;
    bool _initialized = false, _zeroSeen = false, _running = false, _starting = false, _moving = false;
    uint32_t _lastUs = 0, _transitionUs = 0;
    float _phase = 0, _frequency = 0, _left = 1500, _right = 1500;
    float _fromLeft = 1500, _fromRight = 1500, _leftDown = 1500, _rightDown = 1500;
};
