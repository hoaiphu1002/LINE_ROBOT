/*
 * control_motor.c
 *
 *  Created on: Oct 22, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#include "control_motor.h"
#include <stdlib.h>   // abs()
#include <stdio.h>
#include <string.h>
#include "hienthi.h"
// ======= External peripheral handles (do CubeMX sinh) =======
extern TIM_HandleTypeDef htim1; // PWM (TIM1)
extern TIM_HandleTypeDef htim2; // Encoder left
extern TIM_HandleTypeDef htim4; // Encoder right
extern UART_HandleTypeDef huart1;

// ======= Biến toàn cục =======
Motor_t Motor_Left;
Motor_t Motor_Right;


// ======= Khởi tạo (start PWM + encoder) =======
void Motor_Init(void)
{
    // Start PWM channels (TIM1 CH1 = Right, CH2 = Left)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // Start encoder timers
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    // Reset counter để bắt đầu đo từ 0
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    Motor_Left.last_counts = 0;
    Motor_Left.total_counts = 0;
    Motor_Left.speed_mps = 0.0f;

    Motor_Right.last_counts = 0;
    Motor_Right.total_counts = 0;
    Motor_Right.speed_mps = 0.0f;

    Motor_Stop();
}

// ======= Dừng động cơ (zero PWM và tắt tín hiệu chiều) =======
void Motor_Stop(void)
{
    // Tắt chân điều khiển chiều (bi-direction pins)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET); // left dir pins
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET); // right dir pins

    // Set compare = 0 -> 0% duty
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

// ======= Set PWM theo phần trăm -100..100 =======
// left_percent, right_percent: dương => tiến; âm => lùi
void Motor_SetPWM(int8_t left_percent, int8_t right_percent)
{

    // Lấy ARR (AutoReload) hiện tại của timer PWM để quy đổi percent -> compare value
    // Formula: compare = (percent_abs / 100) * ARR
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); // ARR giá trị cấu hình trong CubeMX
    uint32_t left_compare  = (uint32_t)(( (uint32_t)abs(left_percent)  * arr) / PWM_MAX_DUTY);
    uint32_t right_compare = (uint32_t)(( (uint32_t)abs(right_percent) * arr) / PWM_MAX_DUTY);

    // Set chân chiều quay (direction pins)
    if (left_percent >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // BI1 = 1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // BI2 = 0
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    }

    if (right_percent >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // AI1 = 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // AI2 = 0
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }

    // Gán giá trị compare cho timer
    // Lưu ý: TIM_CHANNEL_1 dùng cho Right PWM (PA8), TIM_CHANNEL_2 cho Left PWM (PA7)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_compare);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, left_compare);
}

// ======= Đọc encoder: lưu last_counts và xóa counter để đo delta tiếp theo =======
void Motor_UpdateEncoder(void)
{
    // Đọc giá trị counter hiện tại (có thể âm nếu timer ở chế độ up/down? ở encoder mode counter là unsigned,
    // nhưng ta cast về int16_t để nhận giá trị âm nếu cần khi cấu hình đảo chiều bằng logic)
    // Thực tế TIM counter là unsigned 16-bit; khi ta reset sau đọc, delta = value đọc được.
    int16_t left_cnt  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t right_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    // Ghi lại và cộng tổng
    Motor_Left.last_counts  = left_cnt;
    Motor_Left.total_counts += left_cnt;

    Motor_Right.last_counts = right_cnt;
    Motor_Right.total_counts += right_cnt;

    // Reset counter để lần đọc tiếp theo đo delta mới
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

// ======= Tính vận tốc (m/s) từ last_counts và dt =======
// Công thức cơ bản:
//  - Số vòng quay trong dt = last_counts / ENC_PULSE_PER_REV
//  - Quãng đường in dt = vòng * C = (last_counts / ENC_PULSE_PER_REV) * C
//  - Vận tốc m/s = quãng đường / dt = (last_counts / ENC_PULSE_PER_REV) * (C / dt)
void Motor_UpdateSpeed(float dt)
{
    // Tránh chia cho 0
    if (dt <= 0) return;

    Motor_Left.speed_mps  = ( (float)Motor_Left.last_counts  / ENC_PULSE_PER_REV ) * ( WHEEL_CIRCUMFERENCE / dt );
    Motor_Right.speed_mps = ( (float)Motor_Right.last_counts / ENC_PULSE_PER_REV ) * ( WHEEL_CIRCUMFERENCE / dt );
}

// ======= In tốc độ qua UART (m/s và RPM) =======
void Motor_PrintSpeed(void)
{
    char buf[128];
    // RPM tính từ speed_mps:
    // rpm = speed_mps / circumference (m per rev) * 60 (s -> min)
    float left_rpm  = (Motor_Left.speed_mps  / WHEEL_CIRCUMFERENCE) * 60.0f;
    float right_rpm = (Motor_Right.speed_mps / WHEEL_CIRCUMFERENCE) * 60.0f;

    snprintf(buf, sizeof(buf),
             "L: %.3f m/s (%.1f RPM) | R: %.3f m/s (%.1f RPM)\r\n",
             Motor_Left.speed_mps, left_rpm,
             Motor_Right.speed_mps, right_rpm);
    print_uart(buf);
}
