#include "pid.h"
#include "main.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd,
              float alpha, float beta, float max_theta, uint8_t use_tanh)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->alpha = alpha;
    pid->beta = beta;
    pid->max_theta = max_theta;
    pid->use_tanh = use_tanh;

    pid->prev_out_x = 0; pid->prev_out_y = 0;
    pid->prev_err_x = 0; pid->prev_err_y = 0;
    pid->sum_err_x = 0;  pid->sum_err_y = 0;
    pid->last_time_ms = HAL_GetTick();
}

void PID_Compute(PIDController *pid, float target_x, float target_y,
                 float current_x, float current_y,
                 float *out_theta, float *out_phi, int reset_i)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - pid->last_time_ms) / 1000.0f;

	float freq = 1/dt;
	int integer_part = (int)freq;
    char print_freq[16];
	int fractional_part = (int)((freq - integer_part) * 100);
	sprintf(print_freq, "freq: %d.%02dHz", integer_part, fractional_part);
	LCD_DrawString(4, 90, (uint8_t *)print_freq);

	if (reset_i == 1){
	    pid->sum_err_x = 0;
	    pid->sum_err_y = 0;
	}

    float err_x = current_x - target_x;
    float err_y = current_y - target_y;

    pid->sum_err_x += err_x * dt;
    pid->sum_err_y += err_y * dt;

    float d_err_x = (err_x - pid->prev_err_x) / dt;
    float d_err_y = (err_y - pid->prev_err_y) / dt;

    float pid_x = pid->kp * err_x + pid->ki * pid->sum_err_x + pid->kd * d_err_x;
    float pid_y = pid->kp * err_y + pid->ki * pid->sum_err_y + pid->kd * d_err_y;

    float filtered_x = pid->alpha * pid_x + (1.0f - pid->alpha) * pid->prev_out_x;
    float filtered_y = pid->alpha * pid_y + (1.0f - pid->alpha) * pid->prev_out_y;

    /* Spherical coordinates */
    *out_phi = atan2f(filtered_y, filtered_x) * (180.0f / M_PI);
    if (*out_phi < 0) *out_phi += 360.0f;

    float r = sqrtf(filtered_x*filtered_x + filtered_y*filtered_y);

    if (pid->use_tanh)
        *out_theta = fmaxf(0.0f, pid->max_theta * tanhf(pid->beta * r));
    else
        *out_theta = fminf(fmaxf(0.0f, pid->beta * r), pid->max_theta);

    pid->prev_err_x = err_x;
    pid->prev_err_y = err_y;
    pid->prev_out_x = filtered_x;
    pid->prev_out_y = filtered_y;
    pid->last_time_ms = now;
}
