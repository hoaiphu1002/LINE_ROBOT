/*
 * line_follow.h
 *
 *  Created on: Oct 16, 2025
 *      Author: TRƯƠNG VŨ HOÀI PHÚ
 */

#ifndef INC_LINE_FOLLOW_H_
#define INC_LINE_FOLLOW_H_

#include "stm32f1xx_hal.h"

#define NUM_SENSORS        5               // Số lượng cảm biến TCRT5000
#define SENSOR_SPACING_MM  13.0f           // Khoảng cách giữa hai cảm biến (mm)
#define CENTER_INDEX       2

extern ADC_HandleTypeDef hadc1;            // Khai báo ADC1 từ CubeMX

extern uint16_t adc_dma_buffer[NUM_SENSORS];
// Cấu trúc lưu dữ liệu sensor
typedef struct {
    uint16_t adc_raw[NUM_SENSORS];        // Giá trị ADC đọc được
    float position_mm;                    // Vị trí line tính bằng mm
    int16_t index_max;                    // Vị trí cảm biến có giá trị lớn nhất
} LineSensor_t;

void FOLLOW_ReadSensors(LineSensor_t *line);
float FOLLOW_ComputePosition(LineSensor_t *line);
float FOLLOW_GetPositionMM(void);
void FOLLOW_Init(void);
// Hàm kiểm tra độ phân giải cảm biến
void FOLLOW_TestSensors(void);

#endif /* INC_LINE_FOLLOW_H_ */
