#pragma once
#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "device.h"
#include "common.h"

#if (defined(PLATFORM_ESP32))
#include "DShotRMT.h"
#endif

extern device_t ServoOut_device;
#define HAS_SERVO_OUTPUT
#define OPT_HAS_SERVO_OUTPUT (GPIO_PIN_PWM_OUTPUTS_COUNT > 0)

// Notify this unit that new channel data has arrived
void servoNewChannelsAvailable();
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
bool flappingOutputsParked();
struct FlappingDebugSnapshot {
    const char *reason = "waiting";
    const char *configurationError = nullptr;
    bool armed = false, rcValid = false;
    uint8_t throttleChannel = 3, armChannel = 5;
    uint16_t throttleUs = 0, armUs = 0, leftUs = 0, rightUs = 0;
    float frequencyHz = 0;
};
FlappingDebugSnapshot flappingGetDebugSnapshot();
#endif
#else
inline void servoNewChannelsAvailable(){};
#endif
