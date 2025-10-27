#include "line_follow.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern void print_uart(const char *msg);
extern ADC_HandleTypeDef hadc1;

LineSensor_t LineData;    // Cấu trúc dữ liệu line sensor
extern uint16_t adc_dma_buffer[NUM_SENSORS]; // buffer DMA được khai báo trong main.c

// ================================================================
// 1️⃣ KHỞI TẠO
// ================================================================
void FOLLOW_Init(void)
{
    for (int i = 0; i < NUM_SENSORS; i++)
        LineData.adc_raw[i] = 0;

    LineData.position_mm = 0.0f;
    LineData.index_max = 0;

    // Khởi động ADC1 ở chế độ DMA (liên tục)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, NUM_SENSORS);
}

// ================================================================
// 2️⃣ CẬP NHẬT DỮ LIỆU TỪ DMA BUFFER (Remap theo PA4→PA0)
// ================================================================
void FOLLOW_ReadSensors(LineSensor_t *line)
{
    // Bảng ánh xạ: trái (PA4) → phải (PA0)
    static const uint8_t adc_map[NUM_SENSORS] = {4, 3, 2, 1, 0};

    for (int i = 0; i < NUM_SENSORS; i++)
        line->adc_raw[i] = adc_dma_buffer[adc_map[i]];
}

// ================================================================
// 3️⃣ CALLBACK KHI ADC DMA HOÀN TẤT 1 CHU KỲ
// ================================================================
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
        FOLLOW_ReadSensors(&LineData); // Cập nhật dữ liệu ổn định
}

// ================================================================
// 4️⃣ TÍNH VỊ TRÍ LINE (TRUNG BÌNH TRỌNG SỐ)
// ================================================================
float FOLLOW_ComputePosition(LineSensor_t *line)
{
    float numerator = 0.0f;        // Tổng có trọng số: Σ(x_i * I_i)
    float denominator = 0.0f;      // Tổng cường độ: Σ(I_i)

    // Duyệt trái -> phải (adc_raw[0] = trái nhất)
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        // Vị trí vật lý của cảm biến i so với tâm
        float x_i = ( CENTER_INDEX-i) * SENSOR_SPACING_MM;

              float I_i = (float)line->adc_raw[i];
        if (I_i < 0.0f) I_i = 0.0f; // tránh giá trị âm do nhiễu

        numerator += x_i * I_i;
        denominator += I_i;
    }

    // Kiểm tra tránh chia 0
    if (denominator == 0.0f)
        line->position_mm = 0.0f;
    else
        line->position_mm = numerator / denominator; // Σ(x_i * I_i) / Σ(I_i)

    // Tìm cảm biến mạnh nhất (đảo cực => tìm giá trị ADC nhỏ nhất)
    line->index_max = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    	if (line->adc_raw[i] > line->adc_raw[line->index_max])
    	        line->index_max = i;
    }

    return line->position_mm;
}


// ================================================================
// 5️⃣ TRẢ VỀ VỊ TRÍ HIỆN TẠI
// ================================================================
float FOLLOW_GetPositionMM(void)
{
    return LineData.position_mm;
}

// ================================================================
// 6️⃣ TEST UART: IN GIÁ TRỊ ADC + VỊ TRÍ LINE
// ================================================================
void FOLLOW_TestSensors(void)
{
    char buf[256];

    // Gọi FOLLOW_ReadSensors() để cập nhật dữ liệu ngay lúc test
    FOLLOW_ReadSensors(&LineData);

    sprintf(buf, "ADC RAW (L→R): ");
    print_uart(buf);

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        sprintf(buf, "%4d ", LineData.adc_raw[i]);
        print_uart(buf);
    }

    float pos = FOLLOW_ComputePosition(&LineData);

    sprintf(buf, " | MaxIdx:%d  Pos:%6.2f mm\r\n",
            LineData.index_max, pos);
    print_uart(buf);
}
