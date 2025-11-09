/*
 * PID_motor.c
 *
 *  Created on: Nov 7, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#include "PID_control.h"

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Giới hạn đầu ra
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    pid->output = output;
    pid->prev_error = error;

    return output;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
