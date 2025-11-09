/*
 * PID_control.h
 *
 *  Created on: Nov 7, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include "stm32f1xx_hal.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float output;
    float out_min;
    float out_max;
} PID_TypeDef;

// Hàm khởi tạo PID
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max);

// Hàm tính toán PID (setpoint: tốc độ mong muốn, measured: tốc độ đo)
float PID_Compute(PID_TypeDef *pid, float setpoint, float measured, float dt);

// Hàm reset PID (nếu cần reset khi đổi hướng hoặc dừng)
void PID_Reset(PID_TypeDef *pid);

#endif /* INC_PID_CONTROL_H_ */
