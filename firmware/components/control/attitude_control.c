#include "attitude_control.h"

#include <math.h>

void attitude_ctrl_init(attitude_ctrl_t *c)
{
    *c = (attitude_ctrl_t) {0};
    c->tunings.kp           = CONFIG_STAB_KP;
    c->tunings.kd           = CONFIG_STAB_KD;
    c->tunings.deadbandDeg  = CONFIG_STAB_DEADBAND_DEG;
    c->tunings.maxAngleDeg  = CONFIG_STAB_MAX_ANGLE_DEG;
    c->tunings.maxOutput    = CONFIG_STAB_MAX_OUTPUT;
}

void attitude_ctrl_reset(attitude_ctrl_t *c)
{
    c->output    = 0.0f;
    c->inDeadband = false;
    c->angleStop  = false;
    c->errorDeg   = 0.0f;
}

bool attitude_ctrl_set_tunings(attitude_ctrl_t *c,
                               const attitude_ctrl_tunings_t *t)
{
    if ((c == NULL) || (t == NULL) || (t->kp < 0.0f) || (t->kd < 0.0f) ||
        (t->deadbandDeg < 0.0f) || (t->maxAngleDeg <= t->deadbandDeg) ||
        (t->maxOutput <= 0.0f)) {
        return false;
    }
    c->tunings = *t;
    attitude_ctrl_reset(c);
    return true;
}

void attitude_ctrl_set_target(attitude_ctrl_t *c, float targetDeg)
{
    if (c != NULL) c->targetDeg = targetDeg;
}

void attitude_ctrl_set_output_limit(attitude_ctrl_t *c, float maxOutput)
{
    if (c == NULL || maxOutput <= 0.0f) return;
    c->tunings.maxOutput = maxOutput;
}

void attitude_ctrl_update(attitude_ctrl_t *c, float measuredDeg, float gyroDps)
{
    const float err = c->targetDeg - measuredDeg;
    const float absErr = fabsf(err);
    float out;

    c->measuredDeg = measuredDeg;
    c->gyroDps     = gyroDps;
    c->errorDeg    = err;
    c->inDeadband  = false;
    c->angleStop   = false;

    if (absErr <= c->tunings.deadbandDeg) {
        /* 静止区：腿在正常范围，不输出 */
        c->inDeadband = true;
        c->output = 0.0f;
    } else if (absErr > c->tunings.maxAngleDeg) {
        /* 超角度：停止推进（安全） */
        c->angleStop = true;
        c->output = 0.0f;
    } else {
        /* 线性区：Kp*误差 - Kd*角速度（角速度阻尼，抗水下扰动） */
        out = c->tunings.kp * err - c->tunings.kd * gyroDps;
        if (out > c->tunings.maxOutput) {
            out = c->tunings.maxOutput;
        } else if (out < -c->tunings.maxOutput) {
            out = -c->tunings.maxOutput;
        }
        c->output = out;
    }
}
