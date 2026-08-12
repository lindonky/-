/* Host replay test for the single-leg training state machine.
 * Build on the development PC with:
 *   gcc -DTRAINING_SESSION_HOST_TEST -Ifirmware/components/control \
 *       firmware/tools/training_session_replay.c \
 *       firmware/components/control/training_session.c -lm
 */
#include "training_session.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t s_now;

static void feed(float pitch, float roll, float gyroY, float accelG)
{
    const training_sample_t s = {
        .nowMs = s_now,
        .imuValid = true,
        .mode = 1,
        .pitchDeg = pitch,
        .rollDeg = roll,
        .gyroYDps = gyroY,
        .accelZG = accelG,
        .controlCap = 1000.0f,
        .correctionOut = roll * 20.0f,
    };
    training_session_feed(&s);
    s_now += 50U;
}

static void hold(float pitch, float roll, uint32_t ms)
{
    for (uint32_t t = 0U; t < ms; t += 50U) feed(pitch, roll, 0.0f, 1.0f);
}

static void cycle(float peakPitch, float maxRoll)
{
    for (int i = 0; i <= 10; i++) {
        const float f = (float)i / 10.0f;
        feed(peakPitch * f, maxRoll * f, 40.0f, 1.0f);
    }
    for (int i = 9; i >= 0; i--) {
        const float f = (float)i / 10.0f;
        feed(peakPitch * f, maxRoll * f, -40.0f, 1.0f);
    }
    hold(0.0f, 0.0f, 300U);
}

static void cycle_with_return_error(float peakPitch, float returnPitch)
{
    for (int i = 0; i <= 10; i++) {
        const float f = (float)i / 10.0f;
        feed(peakPitch * f, 0.0f, 40.0f, 1.0f);
    }
    for (int i = 9; i >= 0; i--) {
        const float f = (float)i / 10.0f;
        const float pitch = returnPitch + (peakPitch - returnPitch) * f;
        feed(pitch, 0.0f, -40.0f, 1.0f);
    }
    hold(returnPitch, 0.0f, 300U);
    hold(0.0f, 0.0f, 300U);
}

static training_status_t status(void)
{
    training_status_t s;
    training_session_get_status(&s, s_now);
    return s;
}

int main(void)
{
    training_session_init();
    training_status_t s = status();
    assert(!s.goalEnabled);
    assert(fabsf(s.goalRomMinDeg - 15.0f) < 0.1f);
    assert(fabsf(s.goalRomMaxDeg - 45.0f) < 0.1f);
    assert(!training_session_set_goal(true, 7.0f, 30.0f, 50.0f));
    assert(!training_session_set_goal(true, 20.0f, 61.0f, 50.0f));
    assert(!training_session_set_goal(true, 20.0f, 21.0f, 50.0f));
    assert(!training_session_set_goal(true, 20.0f, 30.0f, 80.0f));
    assert(training_session_set_goal(true, 20.0f, 30.0f, 50.0f));
    assert(training_session_set_height(170.0f));
    assert(training_session_start(s_now, 0.0f, 0.0f, 1, true, false));
    assert(!training_session_set_goal(false, 0.0f, 0.0f, 0.0f));

    hold(0.0f, 0.0f, 300U);
    cycle(25.0f, 4.0f);
    s = status();
    assert(s.goalEnabled);
    assert(fabsf(s.goalCadenceSpm - 50.0f) < 0.1f);
    assert(fabsf(s.goalCadenceToleranceSpm - 7.5f) < 0.1f);
    assert(s.steps == 1U);
    assert(s.qualifiedSteps == 1U);
    assert(s.lastRomDeg > 24.0f && s.lastRomDeg < 26.0f);
    assert(s.lastEstimatedStepCm > 17.0f && s.lastEstimatedStepCm < 19.0f);

    /* Less than the 8-degree start threshold must not count. */
    cycle(6.0f, 1.0f);
    s = status();
    assert(s.steps == 1U);

    /* A complete cycle with excessive lateral motion counts but does not qualify. */
    cycle(25.0f, 11.0f);
    s = status();
    assert(s.steps == 2U);
    assert(s.qualifiedSteps == 1U);
    assert((s.lastQualityFlags & TRAIN_QUALITY_LATERAL) != 0U);
    assert(fabsf(s.qualifiedPct - 50.0f) < 0.1f);

    /* Returning inside the 8-degree completion band but outside the 5-degree
     * quality target completes the cycle and records a return error. */
    cycle_with_return_error(25.0f, 6.5f);
    s = status();
    assert(s.steps == 3U);
    assert((s.lastQualityFlags & TRAIN_QUALITY_RETURN) != 0U);

    /* A new optional target is atomic, survives session start and contributes
     * cadence-specific quality flags without weakening hard safety limits. */
    assert(training_session_stop(s_now));
    assert(training_session_set_goal(true, 20.0f, 30.0f, 20.0f));
    assert(training_session_start(s_now, 0.0f, 0.0f, 1, true, false));
    hold(0.0f, 0.0f, 300U);
    cycle(25.0f, 0.0f);
    s = status();
    assert(s.steps == 1U);
    assert((s.lastQualityFlags & TRAIN_QUALITY_CADENCE_HIGH) != 0U);

    /* A lone impact without a sustained changed posture must not confirm a fall. */
    feed(0.0f, 0.0f, 150.0f, 2.5f);
    hold(0.0f, 0.0f, 2700U);
    s = status();
    assert(s.fallEvents == 0U);
    assert(!training_session_take_fall_event());

    /* Impact + large posture change + 0.8 s stable posture confirms once. */
    feed(0.0f, 0.0f, 150.0f, 2.5f);
    for (int i = 0; i < 20; i++) feed(50.0f, 0.0f, 0.0f, 1.0f);
    s = status();
    assert(s.fallEvents == 1U);
    assert(s.state == TRAIN_STATE_PAUSED);
    assert(training_session_take_fall_event());
    assert(!training_session_take_fall_event());

    puts("training_session_replay: PASS");
    return 0;
}
