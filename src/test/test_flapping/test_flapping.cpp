#include <unity.h>
#include "Flapping.h"
#include "FlappingJson.h"
#include <limits>

void setUp() {}
void tearDown() {}

static FlappingSettings calibrated()
{
    FlappingSettings s;
    s.enabled = true; s.minHz = s.maxHz = 1;
    s.left.upUs = 1400; s.left.downUs = 1800;
    s.right.upUs = 1600; s.right.downUs = 1200;
    return s;
}

static void advance(FlappingController &c, const FlappingSettings &s, uint32_t &t, uint32_t duration,
    bool allowed = true, float throttle = 1, float steer = 0)
{
    while (duration) {
        const uint32_t step = duration < 4000 ? duration : 4000;
        t += step; duration -= step;
        c.update(s, t, allowed, throttle, steer);
    }
}

static void start(FlappingController &c, const FlappingSettings &s, uint32_t &t, float steer = 0)
{
    c.update(s, t, true, 0, steer);
    advance(c, s, t, 4000, true, 0, steer);
    advance(c, s, t, 304000, true, 1, steer);
    TEST_ASSERT_EQUAL_UINT16(s.left.upUs, c.leftUs());
}

static void curve_and_asymmetric_timing()
{
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0, FlappingController::ease(0));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.5f, FlappingController::ease(0.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 1, FlappingController::ease(1));
    auto s = calibrated(); FlappingController c; uint32_t t = 0; start(c, s, t);
    advance(c, s, t, 175000); TEST_ASSERT_UINT16_WITHIN(1, 1600, c.leftUs());
    advance(c, s, t, 175000); TEST_ASSERT_UINT16_WITHIN(1, 1800, c.leftUs());
    advance(c, s, t, 325000); TEST_ASSERT_UINT16_WITHIN(1, 1600, c.leftUs());
    advance(c, s, t, 325000); TEST_ASSERT_UINT16_WITHIN(1, 1400, c.leftUs());
}

static void differential_and_cycle_latching()
{
    auto s = calibrated(); FlappingController c; uint32_t t = 0; start(c, s, t, 1);
    advance(c, s, t, 350000, true, 1, -1); // New direction cannot move the current cycle endpoint.
    TEST_ASSERT_UINT16_WITHIN(1, 1800, c.leftUs());
    TEST_ASSERT_UINT16_WITHIN(1, 1280, c.rightUs());
    advance(c, s, t, 10000000, true, 1, 1);
    TEST_ASSERT_UINT16_WITHIN(1, 1800, c.leftUs());
    TEST_ASSERT_UINT16_WITHIN(1, 1280, c.rightUs()); // no accumulated differential
    s.reverseSteering = true;
    FlappingController reversed; t = 0; start(reversed, s, t, 1);
    advance(reversed, s, t, 350000, true, 1, 1);
    TEST_ASSERT_UINT16_WITHIN(1, 1720, reversed.leftUs());
    TEST_ASSERT_UINT16_WITHIN(1, 1200, reversed.rightUs());
}

static void zero_gate_stop_and_restart()
{
    auto s = calibrated(); FlappingController c; uint32_t t = 0;
    advance(c, s, t, 500000, true, 1); TEST_ASSERT_TRUE(c.parked());
    TEST_ASSERT_EQUAL_UINT16(1500, c.leftUs());
    start(c, s, t); advance(c, s, t, 175000);
    const auto from = c.leftUs();
    advance(c, s, t, 4000, false); TEST_ASSERT_EQUAL_UINT16(from, c.leftUs());
    advance(c, s, t, 150000, false); TEST_ASSERT_UINT16_WITHIN(1, (from + 1500) / 2, c.leftUs());
    advance(c, s, t, 150000, false); TEST_ASSERT_TRUE(c.parked());
    advance(c, s, t, 600000, true, 1); TEST_ASSERT_TRUE(c.parked());
    advance(c, s, t, 4000, true, 0.01f); // 1% is a stop request
    advance(c, s, t, 4000, true, 1); TEST_ASSERT_FALSE(c.parked());
}

static void frequency_change_keeps_phase()
{
    auto s = calibrated(); s.minHz = 1; s.maxHz = 2;
    FlappingController c; uint32_t t = 0; start(c, s, t);
    advance(c, s, t, 40000, true, 0.505f); // 1.5Hz
    const int before = c.leftUs();
    advance(c, s, t, 4000, true, 1);
    TEST_ASSERT_TRUE(c.leftUs() >= before && c.leftUs() - before < 30);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2, c.frequencyHz());
}

static void disabling_and_remapping_park_old_outputs()
{
    auto s = calibrated(); FlappingController c; uint32_t t = 0; start(c, s, t);
    advance(c, s, t, 175000);
    auto next = s; next.enabled = false; next.left.output = 3; next.right.output = 4;
    advance(c, next, t, 4000);
    TEST_ASSERT_TRUE(c.ownsOutputs()); TEST_ASSERT_EQUAL_UINT8(1, c.settings().left.output);
    advance(c, next, t, 300000); TEST_ASSERT_TRUE(c.parked());
    TEST_ASSERT_EQUAL_UINT16(1500, c.leftUs());
    advance(c, next, t, 4000); TEST_ASSERT_FALSE(c.ownsOutputs());
    TEST_ASSERT_EQUAL_UINT8(3, c.settings().left.output);
}

static void timer_wrap_and_stale_update()
{
    auto s = calibrated(); FlappingController c; uint32_t t = UINT32_MAX - 100000;
    start(c, s, t); advance(c, s, t, 350000);
    TEST_ASSERT_UINT16_WITHIN(1, 1800, c.leftUs());
    t += 200000; c.update(s, t, true, 1, 0);
    advance(c, s, t, 300000); TEST_ASSERT_TRUE(c.parked());
    advance(c, s, t, 500000); TEST_ASSERT_TRUE(c.parked());
}

static void pwm_test_return_and_invalid_input()
{
    auto s = calibrated(); FlappingController c; uint32_t t = 0; start(c, s, t);
    c.suspend(1700, 1300, t);
    advance(c, s, t, 4000, false); advance(c, s, t, 300000, false);
    TEST_ASSERT_TRUE(c.parked()); TEST_ASSERT_EQUAL_UINT16(1500, c.leftUs());
    start(c, s, t); advance(c, s, t, 50000);
    advance(c, s, t, 4000, true, std::numeric_limits<float>::quiet_NaN());
    advance(c, s, t, 300000); TEST_ASSERT_TRUE(c.parked());
}

static void settings_and_json_validation()
{
    FlappingSettings defaults; TEST_ASSERT_TRUE(flappingSettingsValid(defaults));
    defaults.enabled = true; TEST_ASSERT_FALSE(flappingSettingsValid(defaults));
    const auto s = calibrated(); JsonDocument doc;
    flappingSettingsToJson(doc.to<JsonObject>(), s);
    FlappingSettings loaded; TEST_ASSERT_TRUE(flappingSettingsFromJson(doc.as<JsonVariantConst>(), loaded));
    TEST_ASSERT_TRUE(flappingSettingsEqual(s, loaded));
    doc["left"]["output"] = 257; TEST_ASSERT_FALSE(flappingSettingsFromJson(doc.as<JsonVariantConst>(), loaded));
    TEST_ASSERT_TRUE(flappingSettingsEqual(s, loaded)); // failed parses are atomic
    doc["left"]["output"] = 1; doc["max_hz"] = 11;
    TEST_ASSERT_FALSE(flappingSettingsFromJson(doc.as<JsonVariantConst>(), loaded));
}

static void context_validation()
{
    auto s = calibrated(); FlappingOutputLimits outputs[2];
    for (auto &o : outputs) { o.available = true; o.frequency = 50; }
    TEST_ASSERT_NULL(flappingContextError(s, true, 5, 7, false, outputs, 2));
    TEST_ASSERT_NOT_NULL(flappingContextError(s, false, 5, 7, false, outputs, 2));
    TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 3, 7, false, outputs, 2));
    TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 5, 1, false, outputs, 2));
    TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 5, 7, true, outputs, 2));
    TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 5, 7, false, outputs, 1));
    outputs[0].available = false; TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 5, 7, false, outputs, 2));
    outputs[0].available = true; s.maxHz = 5;
    TEST_ASSERT_NOT_NULL(flappingContextError(s, true, 5, 7, false, outputs, 2));
    outputs[0].frequency = outputs[1].frequency = 100;
    TEST_ASSERT_NULL(flappingContextError(s, true, 5, 7, false, outputs, 2));
}

static void configuration_save_guard()
{
    TEST_ASSERT_NULL(flappingChangeError(false, false, true, 2012, 2012, false));
    TEST_ASSERT_NULL(flappingChangeError(true, true, false, 2012, 2012, false));
    TEST_ASSERT_NULL(flappingChangeError(true, true, true, 998, 988, true));
    TEST_ASSERT_NOT_NULL(flappingChangeError(true, false, false, 988, 988, true));
    TEST_ASSERT_NOT_NULL(flappingChangeError(true, true, true, 999, 988, true));
    TEST_ASSERT_NOT_NULL(flappingChangeError(true, true, true, 988, 2012, true));
    TEST_ASSERT_NOT_NULL(flappingChangeError(true, true, true, 988, 988, false));
}

static void runtime_state_explains_startup_and_parking()
{
    auto s = calibrated(); FlappingController c; uint32_t t = 0;
    advance(c, s, t, 4000, true, (1000.0f - 988) / 1024);
    TEST_ASSERT_EQUAL_STRING("waiting_zero", c.stateReason());
    advance(c, s, t, 4000, true, 0);
    TEST_ASSERT_EQUAL_STRING("idle", c.stateReason());
    advance(c, s, t, 4000, true, 1);
    TEST_ASSERT_EQUAL_STRING("starting", c.stateReason());
    advance(c, s, t, 304000, true, 1);
    TEST_ASSERT_EQUAL_STRING("running", c.stateReason());
    advance(c, s, t, 4000, false);
    TEST_ASSERT_EQUAL_STRING("parking", c.stateReason());
    advance(c, s, t, 304000, false);
    TEST_ASSERT_EQUAL_STRING("waiting_zero", c.stateReason());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(curve_and_asymmetric_timing);
    RUN_TEST(differential_and_cycle_latching);
    RUN_TEST(zero_gate_stop_and_restart);
    RUN_TEST(frequency_change_keeps_phase);
    RUN_TEST(disabling_and_remapping_park_old_outputs);
    RUN_TEST(timer_wrap_and_stale_update);
    RUN_TEST(pwm_test_return_and_invalid_input);
    RUN_TEST(settings_and_json_validation);
    RUN_TEST(context_validation);
    RUN_TEST(configuration_save_guard);
    RUN_TEST(runtime_state_explains_startup_and_parking);
    return UNITY_END();
}
