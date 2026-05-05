#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"
#include <math.h>

typedef struct {
    float kp, ki, kd;
    float alpha;           // exponential filter
    float beta;            // gain for magnitude
    float max_theta;
    uint8_t use_tanh;      // 1 = tanh, 0 = linear

    float prev_out_x, prev_out_y;
    float prev_err_x, prev_err_y;
    float sum_err_x, sum_err_y;
    uint32_t last_time_ms;
} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd,
              float alpha, float beta, float max_theta, uint8_t use_tanh);

void PID_Compute(PIDController *pid, float target_x, float target_y,
                 float current_x, float current_y,
                 float *out_theta, float *out_phi, int reset_i);

#endif
