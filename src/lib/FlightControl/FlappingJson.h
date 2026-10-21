#pragma once
#include "Flapping.h"
#include <ArduinoJson.h>

inline void flappingSettingsToJson(JsonObject obj, const FlappingSettings &s)
{
    obj["enabled"] = s.enabled;
    obj["throttle_channel"] = s.throttleChannel; obj["steering_channel"] = s.steeringChannel;
    obj["min_hz"] = s.minHz; obj["max_hz"] = s.maxHz;
    obj["downstroke_percent"] = s.downstrokePercent; obj["differential_percent"] = s.differentialPercent;
    obj["reverse_steering"] = s.reverseSteering;
    const FlappingWing wings[] = {s.left, s.right};
    const char *keys[] = {"left", "right"};
    for (unsigned i = 0; i < 2; ++i) {
        JsonObject wing = obj[keys[i]].to<JsonObject>();
        wing["output"] = wings[i].output; wing["up_us"] = wings[i].upUs;
        wing["down_us"] = wings[i].downUs; wing["park_us"] = wings[i].parkUs;
    }
}

inline bool flappingSettingsFromJson(JsonVariantConst obj, FlappingSettings &s)
{
    if (!obj.is<JsonObjectConst>() || !obj["enabled"].is<bool>() || !obj["reverse_steering"].is<bool>()) return false;
    const char *channels[] = {"throttle_channel", "steering_channel"};
    for (const char *key : channels) if (!obj[key].is<int>() || obj[key].as<int>() < 1 || obj[key].as<int>() > 16) return false;
    const char *numbers[] = {"min_hz", "max_hz", "downstroke_percent", "differential_percent"};
    for (const char *key : numbers) if (!obj[key].is<float>()) return false;
    FlappingSettings next;
    next.enabled = obj["enabled"]; next.reverseSteering = obj["reverse_steering"];
    next.throttleChannel = obj["throttle_channel"]; next.steeringChannel = obj["steering_channel"];
    next.minHz = obj["min_hz"]; next.maxHz = obj["max_hz"];
    next.downstrokePercent = obj["downstroke_percent"]; next.differentialPercent = obj["differential_percent"];
    FlappingWing *wings[] = {&next.left, &next.right};
    const char *keys[] = {"left", "right"};
    for (unsigned i = 0; i < 2; ++i) {
        const auto wing = obj[keys[i]];
        if (!wing.is<JsonObjectConst>() || !wing["output"].is<int>() || wing["output"].as<int>() < 1 || wing["output"].as<int>() > 16) return false;
        const char *positions[] = {"up_us", "down_us", "park_us"};
        for (const char *key : positions) if (!wing[key].is<int>() || wing[key].as<int>() < 500 || wing[key].as<int>() > 2500) return false;
        wings[i]->output = wing["output"]; wings[i]->upUs = wing["up_us"];
        wings[i]->downUs = wing["down_us"]; wings[i]->parkUs = wing["park_us"];
    }
    if (!flappingSettingsValid(next)) return false;
    s = next; return true;
}
