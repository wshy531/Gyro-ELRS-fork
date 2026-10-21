#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "devServoOutput.h"
#include "PWM.h"
#include "CRSF.h"
#include "config.h"
#include "logging.h"
#include "rxtx_intf.h"
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
#include "devFlightControl.h"
#include "FlightControlConfig.h"
#if defined(PLATFORM_ESP32)
#include "devWIFI.h"
#endif
#endif

static int8_t servoPins[PWM_MAX_CHANNELS];
static pwm_channel_t pwmChannels[PWM_MAX_CHANNELS];
static uint16_t pwmChannelValues[PWM_MAX_CHANNELS];
static bool initialized = false;

#if (defined(PLATFORM_ESP32))
static DShotRMT *dshotInstances[PWM_MAX_CHANNELS] = {nullptr};
const uint8_t RMT_MAX_CHANNELS = 8;
#endif

// true when the RX has a new channels packet
static bool newChannelsAvailable;
// Absolute max failsafe time if no update is received, regardless of LQ
static constexpr uint32_t FAILSAFE_ABS_TIMEOUT_MS = 1000U;

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
uint16_t servoOutputModeToFrequency(eServoOutputMode mode);
static FlappingController flapping;
static uint16_t flappingPwmFrequencies[PWM_MAX_CHANNELS];
bool flappingOutputsParked() { return flapping.parked(); }
static FlappingDebugSnapshot flappingSnapshot;
#if defined(PLATFORM_ESP32)
static portMUX_TYPE flappingSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
#endif

FlappingDebugSnapshot flappingGetDebugSnapshot()
{
#if defined(PLATFORM_ESP32)
    portENTER_CRITICAL(&flappingSnapshotMux);
#endif
    const auto snapshot = flappingSnapshot;
#if defined(PLATFORM_ESP32)
    portEXIT_CRITICAL(&flappingSnapshotMux);
#endif
    return snapshot;
}

static bool flappingOwnsChannel(unsigned ch)
{
    const auto &s = flapping.settings();
    return flapping.ownsOutputs() && (ch + 1 == s.left.output || ch + 1 == s.right.output);
}

static void updateFlapping(uint32_t nowUs, bool rcFresh, bool suspended)
{
    const auto &s = flightControlConfig.GetFlapping();
    // Keep diagnostics available even when the mode is disabled.
    const uint32_t throttle = ChannelData[s.throttleChannel - 1];
    const uint32_t steering = ChannelData[s.steeringChannel - 1];
    const uint32_t arm = ChannelData[flightControlConfig.GetArmChannel()];
    const bool valid = rcFresh && connectionState == connected && connectionHasModelMatch && teamraceHasModelMatch &&
        throttle >= CRSF_CHANNEL_VALUE_MIN && throttle <= CRSF_CHANNEL_VALUE_MAX &&
        steering >= CRSF_CHANNEL_VALUE_MIN && steering <= CRSF_CHANNEL_VALUE_MAX &&
        arm >= CRSF_CHANNEL_VALUE_MIN && arm <= CRSF_CHANNEL_VALUE_MAX;
    const bool armed = flightControlConfig.GetArmMode() && FlightControlRangeIsActive(
        flightControlConfig.GetArmRange(), CRSF_to_US(arm));
    const char *configurationError = nullptr;
    if (s.enabled) {
        FlappingOutputLimits outputs[FC_PWM_OUTPUT_MAX_COUNT];
        const unsigned count = min((unsigned)GPIO_PIN_PWM_OUTPUTS_COUNT, (unsigned)FC_PWM_OUTPUT_MAX_COUNT);
        for (unsigned ch = 0; ch < count; ++ch) {
            const auto *pwm = config.GetPwmChannel(ch);
            auto &o = outputs[ch];
            const auto configuredFrequency = servoOutputModeToFrequency((eServoOutputMode)pwm->val.mode);
            // PWM allocation happens at startup. A newly saved refresh rate must
            // not make the frequency guard assume hardware has already changed.
            o.frequency = flappingPwmFrequencies[ch];
            o.available = servoPins[ch] != UNDEF_PIN && configuredFrequency >= 50 && configuredFrequency <= 400 &&
                o.frequency >= 50 && o.frequency <= 400 && !pwm->val.narrow;
            o.minUs = flightControlConfig.GetPwmOutputLimits(ch).minUs;
            o.maxUs = flightControlConfig.GetPwmOutputLimits(ch).maxUs;
        }
        configurationError = flappingContextError(s, flightControlConfig.GetArmMode(), flightControlConfig.GetArmChannel() + 1,
            flightControlConfig.GetWifiCoexistEnabled() ? flightControlConfig.GetWifiCoexistChannel() + 1 : 0,
            flightControlConfig.GetDriftAssist().enabled, outputs, count);
    }
    static uint32_t revision = UINT32_MAX;
    const bool configChanged = revision != flightControlConfig.GetRevision();
    revision = flightControlConfig.GetRevision();
    const bool ownedBefore = flapping.ownsOutputs();
    const unsigned oldLeftOutput = flapping.settings().left.output, oldRightOutput = flapping.settings().right.output;
    if (s.enabled || flapping.ownsOutputs())
        flapping.update(s, nowUs, valid && armed && !configurationError && !suspended && !configChanged,
            (CRSF_to_US(throttle) - 988.0f) / 1024.0f, (CRSF_to_US(steering) - 1500.0f) / 512.0f);
    const auto &active = flapping.settings();
    if (flapping.ownsOutputs() && (!ownedBefore || oldLeftOutput != active.left.output || oldRightOutput != active.right.output) &&
        active.left.output <= GPIO_PIN_PWM_OUTPUTS_COUNT && active.right.output <= GPIO_PIN_PWM_OUTPUTS_COUNT) {
        const auto actualOrPark = [](uint16_t actual, uint16_t park) { return actual >= 500 && actual <= 2500 ? actual : park; };
        flapping.suspend(actualOrPark(pwmChannelValues[active.left.output - 1], active.left.parkUs),
            actualOrPark(pwmChannelValues[active.right.output - 1], active.right.parkUs), nowUs);
    }
    FlappingDebugSnapshot snapshot;
    snapshot.rcValid = valid;
    snapshot.armed = valid && armed;
    snapshot.throttleChannel = s.throttleChannel;
    snapshot.armChannel = flightControlConfig.GetArmChannel() + 1;
    snapshot.throttleUs = CRSF_to_US(throttle);
    snapshot.armUs = CRSF_to_US(arm);
    snapshot.leftUs = flapping.leftUs(); snapshot.rightUs = flapping.rightUs();
    snapshot.frequencyHz = flapping.frequencyHz();
    snapshot.configurationError = configurationError;
    snapshot.reason = !s.enabled ? "disabled" :
        flightControlConfig.GetPwmOutputWifiEnabled() ? "pwm_test" :
        connectionState == wifiUpdate ? "wifi_only" :
        !valid ? "rc_unavailable" : !armed ? "disarmed" :
        configurationError ? "configuration_invalid" :
        configChanged ? "configuration_changed" : flapping.stateReason();
#if defined(PLATFORM_ESP32)
    portENTER_CRITICAL(&flappingSnapshotMux);
#endif
    flappingSnapshot = snapshot;
#if defined(PLATFORM_ESP32)
    portEXIT_CRITICAL(&flappingSnapshotMux);
#endif
}

static bool flightControlMotorOutputEnabled()
{
    if (!flightControlConfig.GetArmMode())
    {
        return true;
    }

    const uint8_t armChannel = flightControlConfig.GetArmChannel();
    return FlightControlRangeIsActive(
        flightControlConfig.GetArmRange(), CRSF_to_US(ChannelData[armChannel]));
}
#endif

void ICACHE_RAM_ATTR servoNewChannelsAvailable()
{
    newChannelsAvailable = true;
}

uint16_t servoOutputModeToFrequency(eServoOutputMode mode)
{
    switch (mode)
    {
    case som50Hz:
        return 50U;
    case som60Hz:
        return 60U;
    case som100Hz:
        return 100U;
    case som160Hz:
        return 160U;
    case som333Hz:
        return 333U;
    case som400Hz:
        return 400U;
    case som10KHzDuty:
        return 10000U;
    default:
        return 0;
    }
}

static void servoWrite(uint8_t ch, uint16_t us)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
#if defined(PLATFORM_ESP32)
    if ((eServoOutputMode)chConfig->val.mode == somDShot)
    {
        // DBGLN("Writing DShot output: us: %u, ch: %d", us, ch);
        if (dshotInstances[ch])
        {
            us = fmap(constrain(us, 1000, 2000), 1000, 2000, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX); // Convert PWM signal in us to DShot value
            dshotInstances[ch]->send_dshot_value(us);
        }
    }
    else
#endif
    if (servoPins[ch] != UNDEF_PIN && pwmChannelValues[ch] != us)
    {
        pwmChannelValues[ch] = us;
        if ((eServoOutputMode)chConfig->val.mode == somOnOff)
        {
            bool high = us > 1500;
            if (chConfig->val.signalPolarityInverted)
            {
                high = !high;
            }
            digitalWrite(servoPins[ch], high);
        }
        else if ((eServoOutputMode)chConfig->val.mode == som10KHzDuty)
        {
            uint16_t duty = constrain(us, 1000, 2000) - 1000;
            if (chConfig->val.signalPolarityInverted)
            {
                duty = 1000U - duty;
            }
            PWM.setDuty(pwmChannels[ch], duty);
        }
        else
        {
            const uint16_t pulseUs = us / (chConfig->val.narrow + 1);
            if (chConfig->val.signalPolarityInverted)
            {
                PWM.setMicrosecondsPolarityInverted(pwmChannels[ch], pulseUs);
            }
            else
            {
                PWM.setMicroseconds(pwmChannels[ch], pulseUs);
            }
        }
    }
}

static uint16_t failsafePositionUs(const rx_config_pwm_t *chConfig)
{
    constexpr unsigned SERVO_FAILSAFE_MIN = 988U;
    uint16_t us = chConfig->val.failsafe + SERVO_FAILSAFE_MIN;
    if (chConfig->val.inverted)
    {
        us = 3000U - us;
    }
    return us;
}

static void servosFailsafe()
{
    for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
        if (flappingOwnsChannel(ch)) continue;
#endif
        if (chConfig->val.failsafeMode == PWMFAILSAFE_SET_POSITION) {
            // Always write the failsafe position even if the servo has never been started,
            // so all the servos go to their expected position
            servoWrite(ch, failsafePositionUs(chConfig));
        }
        else if (chConfig->val.failsafeMode == PWMFAILSAFE_NO_PULSES) {
            servoWrite(ch, 0);
        }
        else if (chConfig->val.failsafeMode == PWMFAILSAFE_LAST_POSITION) {
            // do nothing
        }
    }
}

#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
static uint16_t mapServoOutput(uint8_t ch, uint16_t us)
{
    constexpr int32_t INPUT_MIN_US = 988;
    constexpr int32_t INPUT_CENTER_US = 1500;
    constexpr int32_t INPUT_MAX_US = 2012;
    const FlightControlPwmOutputLimits &limits = flightControlConfig.GetPwmOutputLimits(ch);
    const int32_t input = constrain(us, INPUT_MIN_US, INPUT_MAX_US);

    if (input <= INPUT_CENTER_US)
    {
        return limits.minUs +
            (input - INPUT_MIN_US) * (limits.centerUs - limits.minUs) /
            (INPUT_CENTER_US - INPUT_MIN_US);
    }
    return limits.centerUs +
        (input - INPUT_CENTER_US) * (limits.maxUs - limits.centerUs) /
        (INPUT_MAX_US - INPUT_CENTER_US);
}

static uint16_t invertMappedServoOutput(uint8_t ch, uint16_t us)
{
    const FlightControlPwmOutputLimits &limits = flightControlConfig.GetPwmOutputLimits(ch);
    const int32_t input = constrain(us, limits.minUs, limits.maxUs);

    if (input <= limits.centerUs)
    {
        return limits.maxUs -
            (input - limits.minUs) * (limits.maxUs - limits.centerUs) /
            (limits.centerUs - limits.minUs);
    }
    return limits.centerUs -
        (input - limits.centerUs) * (limits.centerUs - limits.minUs) /
        (limits.maxUs - limits.centerUs);
}

static bool usesConfigurablePwmLimits(const rx_config_pwm_t *chConfig)
{
    const uint16_t frequency = servoOutputModeToFrequency((eServoOutputMode)chConfig->val.mode);
    return frequency >= 50U && frequency <= 500U;
}
#endif

static void servosUpdate(unsigned long now)
{
    static uint32_t lastUpdate;
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
    const bool outputEnabled = flightControlConfig.GetPwmOutputWifiEnabled();
    static bool haveChannels = false;
    if (newChannelsAvailable) { lastUpdate = now; haveChannels = true; }
    const bool rcFresh = haveChannels && now - lastUpdate <= FAILSAFE_ABS_TIMEOUT_MS && getLq() > 0;
    const bool suspended = connectionState == wifiUpdate || outputEnabled;
    static uint32_t lastFlappingUs = 0;
    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastFlappingUs) >= 4000U) {
        lastFlappingUs = nowUs;
        updateFlapping(nowUs, rcFresh, suspended);
    }
    if (!outputEnabled && flapping.ownsOutputs()) {
        static uint32_t lastWingWriteUs[PWM_MAX_CHANNELS] = {};
        const auto &s = flapping.settings();
        const FlappingWing wings[] = {s.left, s.right};
        const uint16_t values[] = {flapping.leftUs(), flapping.rightUs()};
        for (unsigned i = 0; i < 2; ++i) {
            const unsigned ch = wings[i].output - 1;
            if (ch >= GPIO_PIN_PWM_OUTPUTS_COUNT || !usesConfigurablePwmLimits(config.GetPwmChannel(ch))) continue;
            const unsigned frequency = flappingPwmFrequencies[ch];
            if (!frequency || (uint32_t)(nowUs - lastWingWriteUs[ch]) < (1000000U + frequency - 1) / frequency) continue;
            // Calculate at 4 ms, but submit at most one command per physical PWM frame.
            lastWingWriteUs[ch] = nowUs;
            const auto &limits = flightControlConfig.GetPwmOutputLimits(ch);
            servoWrite(ch, constrain(values[i], limits.minUs, limits.maxUs));
        }
    }
    if (connectionState == wifiUpdate
#if defined(PLATFORM_ESP32)
        || (isFlightControlWifiCoexist() && outputEnabled)
#endif
    )
    {
        for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
            if (!outputEnabled && flappingOwnsChannel(ch)) continue;
            if (usesConfigurablePwmLimits(chConfig))
            {
                uint16_t us = outputEnabled
                    ? flightControlConfig.GetPwmOutputWifiValue(ch)
                    : 0U;
                if (outputEnabled && chConfig->val.inverted)
                {
                    us = invertMappedServoOutput(ch, us);
                }
                servoWrite(ch, us);
            }
        }
        if (outputEnabled && flapping.ownsOutputs()) {
            const auto &s = flapping.settings();
            if (s.left.output <= GPIO_PIN_PWM_OUTPUTS_COUNT && s.right.output <= GPIO_PIN_PWM_OUTPUTS_COUNT)
                flapping.suspend(
                    constrain(pwmChannelValues[s.left.output - 1], 500U, 2500U),
                    constrain(pwmChannelValues[s.right.output - 1], 500U, 2500U), nowUs);
        }
        return;
    }
#endif
    if (newChannelsAvailable)
    {
        newChannelsAvailable = false;
        lastUpdate = now;
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
        const FlightControlMixerOutput &mixerOutput = flightControlGetMixerOutput();
        const bool motorOutputEnabled = flightControlMotorOutputEnabled();
#endif
        for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
            if (flappingOwnsChannel(ch)) continue;
            // Per-channel mixer mode: use flight control mixer output
            if (chConfig->val.mixerMode)
            {
                if (ch < mixerOutput.motorCount || flightControlConfig.GetFlapping().enabled)
                {
                    const bool isServo = flightControlConfig.GetMixerOutputServo(ch);
                    if ((motorOutputEnabled && !flightControlConfig.GetFlapping().enabled) || isServo)
                    {
                        uint16_t us = flightControlConfig.GetFlapping().enabled
                            ? 1500U : mixerOutput.motorUs[ch];
                        if (chConfig->val.inverted)
                        {
                            us = 3000U - us;
                        }
                        if (usesConfigurablePwmLimits(chConfig))
                        {
                            us = mapServoOutput(ch, us);
                        }
                        servoWrite(ch, us);
                    }
                    else
                    {
                        if (chConfig->val.failsafeMode == PWMFAILSAFE_SET_POSITION) {
                            servoWrite(ch, failsafePositionUs(chConfig));
                        }
                        else if (chConfig->val.failsafeMode == PWMFAILSAFE_NO_PULSES) {
                            servoWrite(ch, 0);
                        }
                    }
                }
                continue;
            }
#endif
            // Passthrough mode: use RC channel data
            const uint8_t inputCh = chConfig->val.inputChannel;
            uint16_t us;
            const unsigned crsfVal = ChannelData[inputCh];
            if (crsfVal == 0)
            {
                continue;
            }
            us = CRSF_to_US(crsfVal);
            if (chConfig->val.inverted)
            {
                us = 3000U - us;
            }
            #if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
            if (usesConfigurablePwmLimits(chConfig))
            {
                us = mapServoOutput(ch, us);
            }
            #endif
            servoWrite(ch, us);
        } /* for each servo */
    }     /* if newChannelsAvailable */

    // LQ goes to 0 (100 packets missed in a row)
    // OR last update older than FAILSAFE_ABS_TIMEOUT_MS
    // go to failsafe
    else if (lastUpdate && connectionState != wifiUpdate && ((getLq() == 0) || (now - lastUpdate > FAILSAFE_ABS_TIMEOUT_MS)))
    {
        servosFailsafe();
        lastUpdate = 0;
    }
}

static void initialize()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return;
    }

#if defined(PLATFORM_ESP32)
    uint8_t rmtCH = 0;
#endif
    for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
        pwmChannelValues[ch] = UINT16_MAX;
        pwmChannels[ch] = -1;
        int8_t pin = GPIO_PIN_PWM_OUTPUTS[ch];
#if (defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)) && (defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32))
        // Disconnect the debug UART pins if DEBUG_LOG
        if (pin == U0RXD_GPIO_NUM || pin == U0TXD_GPIO_NUM)
        {
            pin = UNDEF_PIN;
        }
#endif
        // Mark servo pins that are being used for serial (or other purposes) as disconnected
        auto mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;
        if (mode >= somSerial)
        {
            pin = UNDEF_PIN;
        }
#if defined(PLATFORM_ESP32)
        else if (mode == somDShot)
        {
            if (rmtCH < RMT_MAX_CHANNELS)
            {
                auto gpio = (gpio_num_t)pin;
                auto rmtChannel = (rmt_channel_t)rmtCH;
                DBGLN("Initializing DShot: gpio: %u, ch: %d, rmtChannel: %u", gpio, ch, rmtChannel);
                pinMode(pin, OUTPUT);
                digitalWrite(pin, LOW);                
                dshotInstances[ch] = new DShotRMT(gpio, rmtChannel); // Initialize the DShotRMT instance
                rmtCH++;
            }
            pin = UNDEF_PIN;
        }
#endif
        servoPins[ch] = pin;
        // Initialize all servos to low ASAP
        if (pin != UNDEF_PIN)
        {
            if (mode == somOnOff)
            {
                DBGLN("Initializing digital output: ch: %d, pin: %d", ch, pin);
            }
            else
            {
                DBGLN("Initializing PWM output: ch: %d, pin: %d", ch, pin);
            }

            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
}

static int event()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return DURATION_NEVER;
    }
    if (!initialized)
    {
        initialized = true;
        for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
            const auto frequency = servoOutputModeToFrequency((eServoOutputMode)chConfig->val.mode);
            if (frequency && servoPins[ch] != UNDEF_PIN)
            {
                pwmChannels[ch] = PWM.allocate(servoPins[ch], frequency);
#if defined(HAS_BASIC_FLIGHT_CONTROL) && defined(TARGET_RX)
                flappingPwmFrequencies[ch] = pwmChannels[ch] >= 0 ? frequency : 0;
#endif
            }
#if defined(PLATFORM_ESP32)
            else if ((eServoOutputMode)chConfig->val.mode == somDShot)
            {
                dshotInstances[ch]->begin(DSHOT300, false); // Set DShot protocol and bidirectional dshot bool
            }
#endif
        }
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    servosUpdate(millis());
    return DURATION_IMMEDIATELY;
}

device_t ServoOut_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout,
};

#endif
