/*
 * control_motor.h
 *
 *  Created on: Oct 22, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_

#include "main.h"

// ======= Thông số hệ thống (sửa khi cần) =======
#define PWM_MAX_DUTY       100           // Duty cycle theo % (0..100)
#define ENC_PULSE_PER_REV  1320.0f       // xung/vòng trục ra khi dùng 4x decoding (11*30*4)
#define WHEEL_DIAMETER_M   0.065f        // đường kính bánh (m) = 65 mm
#define WHEEL_CIRCUMFERENCE (3.14159265359f * WHEEL_DIAMETER_M) // C = π * D
#define DT_SAMPLE          0.01f         // thời gian mẫu (s) - nếu vòng lặp 10 ms -> 0.01

// ======= Cấu trúc lưu trữ trạng thái motor =======
typedef struct {
    int16_t last_counts;     // số xung đọc trong lần cập nhật gần nhất (có thể âm nếu quay ngược)
    int32_t total_counts;    // tổng xung (đếm tích lũy)
    float speed_mps;         // vận tốc hiện tại (m/s)
} Motor_t;

extern Motor_t Motor_Left;
extern Motor_t Motor_Right;

// ======= API =======
void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetPWM(int8_t left_percent, int8_t right_percent); // -100..100 (%)
void Motor_UpdateEncoder(void);    // đọc encoder và reset counter
void Motor_UpdateSpeed(float dt);  // tính speed_mps dựa trên last_counts và dt
void Motor_PrintSpeed(void);       // in speed qua UART1 (dùng print_uart)



#endif /* INC_CONTROL_MOTOR_H_ */
