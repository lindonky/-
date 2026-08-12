/* Host test:
 *   cl /DCALIBRATION_CAPTURE_HOST_TEST /Ifirmware\components\control \
 *      firmware\tools\calibration_capture_test.c \
 *      firmware\components\control\calibration_capture.c
 */
#include "calibration_capture.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static calibration_capture_sample_t sample(uint32_t index)
{
    calibration_capture_sample_t s = {
        .nowMs = 1000U + index * 50U,
        .sequence = index + 10U,
        .imuValid = true,
        .stabilizeEnabled = true,
        .mode = 1,
        .rollDeg = -2.25f,
        .pitchDeg = 12.34f,
        .yawDeg = 3.5f,
        .motionDeg = 12.34f,
        .lateralDeg = -2.25f,
        .gyroXDps = 1.2f,
        .gyroYDps = 45.6f,
        .gyroZDps = -7.8f,
        .accelXG = 0.125f,
        .accelYG = -0.250f,
        .accelZG = 0.980f,
        .pitchOut = 0.0f,
        .rollOut = -123.0f,
        .t1PulseUs = 1414U,
        .t2PulseUs = 1500U,
        .t3PulseUs = 1586U,
    };
    return s;
}

int main(void)
{
    calibration_capture_status_t st;
    calibration_capture_record_t r;
    assert(sizeof(calibration_capture_record_t) == 44U);
    calibration_capture_init();
    assert(!calibration_capture_start(900U, false, "normal-lift"));
    assert(calibration_capture_start(900U, true, "normal-lift"));

    calibration_capture_sample_t invalid = sample(0U);
    invalid.imuValid = false;
    calibration_capture_feed(&invalid);
    calibration_capture_get_status(&st, 950U);
    assert(st.state == CAPTURE_STATE_RECORDING && st.sampleCount == 0U);
    assert(calibration_capture_stop());
    calibration_capture_get_status(&st, 950U);
    assert(st.state == CAPTURE_STATE_EMPTY && st.sampleCount == 0U);
    assert(calibration_capture_start(950U, true, "normal-lift"));

    for (uint32_t i = 0U; i < st.capacity; ++i) {
        calibration_capture_sample_t s = sample(i);
        calibration_capture_feed(&s);
    }
    calibration_capture_get_status(&st, 2000U);
    assert(st.state == CAPTURE_STATE_READY);
    assert(st.full && st.sampleCount == st.capacity);
    assert(strcmp(st.label, "normal-lift") == 0);
    assert(fabsf(st.sampleRateHz - 20.0f) < 0.01f);
    assert(!calibration_capture_start(2100U, true, "overwrite"));
    assert(calibration_capture_get_record(0U, &r));
    assert(r.pitchCentiDeg == 1234);
    assert(r.rollCentiDeg == -225);
    assert(r.gyroYDeciDps == 456);
    assert(r.accelZMilliG == 980);
    assert(r.rollOut == -123);
    assert(r.t1PulseUs == 1414U && r.t2PulseUs == 1500U && r.t3PulseUs == 1586U);
    assert((r.flags & CAPTURE_FLAG_STABILIZE) != 0U);
    assert(!calibration_capture_get_record(st.capacity, &r));
    assert(calibration_capture_begin_export(&st));
    assert(!calibration_capture_start(2100U, true, "export-overwrite"));
    assert(!calibration_capture_clear());
    calibration_capture_end_export();
    assert(calibration_capture_clear());
    calibration_capture_get_status(&st, 2000U);
    assert(st.state == CAPTURE_STATE_EMPTY && st.sampleCount == 0U);
    puts("calibration_capture_test: PASS");
    return 0;
}
