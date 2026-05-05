#include "main.h"
#include "robot_controller.h"
#include <math.h>


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern float offset_pc8;
extern float offset_pb6;
extern float offset_pc9;
extern float offset_magnitude;
extern float offset_angle_deg;
extern volatile uint8_t offset_mode;

void Robot_Goto_Spherical(float theta_deg, float phi_deg, float base_offset)
{
    float rad = phi_deg * M_PI / 180.0f;

    // Main tilt command
    float servo0 = theta_deg * sinf(rad - 0.0f   * M_PI/180.0f);  // PC9  (0°)
    float servo1 = theta_deg * sinf(rad - 120.0f * M_PI/180.0f); // PB6  (120°)
    float servo2 = theta_deg * sinf(rad - 240.0f * M_PI/180.0f); // PC8  (240°)

    // === OFFSET VECTOR TO SERVO BIAS ===
    // Same geometry as main control
    float offset_rad = offset_angle_deg * M_PI / 180.0f;
    float offset_servo0 = offset_magnitude * sinf(offset_rad - 0.0f   * M_PI/180.0f);
    float offset_servo1 = offset_magnitude * sinf(offset_rad - 120.0f * M_PI/180.0f);
    float offset_servo2 = offset_magnitude * sinf(offset_rad - 240.0f * M_PI/180.0f);

	if (offset_mode)
	{
		char offset_motor[32];
		sprintf(offset_motor, "PC9:%d PB6:%d PC8:%d      ", (int)offset_servo0 * 5, (int)offset_servo1 * 5, (int)offset_servo2 * 5);
		LCD_DrawString(4, 150, (uint8_t*)offset_motor);
	}

    // Final pulse calculation with offset
    uint32_t pulse_pc9 = (uint32_t)(base_offset + servo0 * 100.0f + offset_servo0 * 5.0f + 20);   // PC9
    uint32_t pulse_pb6 = (uint32_t)(base_offset + servo1 * 100.0f + offset_servo1 * 5.0f - 85); // PB6
    uint32_t pulse_pc8 = (uint32_t)(base_offset + servo2 * 100.0f + offset_servo2 * 5.0f - 15); // PC8

    // Safety clamping
    if (pulse_pc9 < 750) pulse_pc9 = 750; if (pulse_pc9 > 1250) pulse_pc9 = 1250;
    if (pulse_pb6 < 750) pulse_pb6 = 750; if (pulse_pb6 > 1250) pulse_pb6 = 1250;
    if (pulse_pc8 < 750) pulse_pc8 = 750; if (pulse_pc8 > 1250) pulse_pc8 = 1250;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse_pc9);  // PC9
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_pb6);  // PB6
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_pc8);  // PC8
}
