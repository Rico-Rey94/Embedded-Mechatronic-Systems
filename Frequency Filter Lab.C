/* USER CODE BEGIN Header */
/*
 * RC Filter Lab - NUCLEO-F439ZI
 * DAC PA4: sine output via DMA + TIM6 trigger
 * ADC1 DMA: sample Vout (PA3/A0) and Vin (PC0/A1)
 * UART prints CSV: f, Vin_pp, Vout_pp, gain, gain_dB
 */
/* USER CODE END Header */

#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE BEGIN PV */
#define VREF          3.3f
#define DAC_BITS      4095.0f
#define ADC_BITS      4095.0f
#define LUT_N         100U
#define ADC_BUF_LEN   4000U // Must be even
#define SETTLE_MS     200U
#define SINE_OFFSET_V 1.65f
#define SINE_AMP_V    1.00f

static uint16_t dac_lut[LUT_N];
// Interleaved: [Vout0, Vin0, Vout1, Vin1, ...]
static uint16_t adc_dma[ADC_BUF_LEN];
static volatile uint8_t adc_half_done = 0;
static volatile uint8_t adc_full_done = 0;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

static void build_sine_lut(void)
{
    for (uint32_t i = 0; i < LUT_N; i++) {
        float phase = 2.0f * 3.14159265f * ((float)i / (float)LUT_N);
        float v = SINE_OFFSET_V + SINE_AMP_V * sinf(phase);
        if (v < 0.0f) v = 0.0f;
        if (v > VREF) v = VREF;
        uint16_t code = (uint16_t)lroundf((v / VREF) * DAC_BITS);
        dac_lut[i] = code;
    }
}

static void set_tim6_sample_rate_hz(uint32_t fs_hz)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clkcfg;
    uint32_t flashLatency;
    HAL_RCC_GetClockConfig(&clkcfg, &flashLatency);
    uint32_t timclk = (clkcfg.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (2U * pclk1);
    uint32_t presc = 0;
    uint32_t arr = (timclk / fs_hz) - 1U;
    while (arr > 0xFFFFU) {
        presc++;
        arr = (timclk / (fs_hz * (presc + 1U))) - 1U;
        if (presc > 0xFFFFU) break;
    }
    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_PRESCALER(&htim6, presc);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    __HAL_TIM_ENABLE(&htim6);
}

static void compute_pp_from_adc_block(const uint16_t *buf, uint32_t len,
                                      float *vout_pp, float *vin_pp)
{
    uint16_t vout_min = 0xFFFF, vout_max = 0;
    uint16_t vin_min = 0xFFFF, vin_max = 0;
    for (uint32_t i = 0; i + 1 < len; i += 2) {
        uint16_t vout = buf[i];
        uint16_t vin = buf[i + 1];
        if (vout < vout_min) vout_min = vout;
        if (vout > vout_max) vout_max = vout;
        if (vin < vin_min) vin_min = vin;
        if (vin > vin_max) vin_max = vin;
    }
    float vout_v = ((float)(vout_max - vout_min) / ADC_BITS) * VREF;
    float vin_v  = ((float)(vin_max  - vin_min ) / ADC_BITS) * VREF;
    *vout_pp = vout_v;
    *vin_pp  = vin_v;
}

// ADC DMA callbacks
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) adc_half_done = 1;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) adc_full_done = 1;
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_TIM6_Init();
    MX_USART3_UART_Init(); // Change to MX_USART2_UART_Init if using USART2

    /* USER CODE BEGIN 2 */
    build_sine_lut();
    HAL_TIM_Base_Start(&htim6);
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_lut, LUT_N, DAC_ALIGN_12B_R) != HAL_OK)
        Error_Handler();
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma, ADC_BUF_LEN) != HAL_OK)
        Error_Handler();

    uart_print("f_hz, Vin_pp_V, Vout_pp_V, gain, gain_dB\r\n");
    const uint32_t f_list[] = {100, 300, 500, 800, 1000, 1500, 2000, 5000, 10000};
    const uint32_t nf = sizeof(f_list)/sizeof(f_list[0]);

    for (uint32_t k = 0; k < nf; k++) {
        uint32_t f_hz = f_list[k];
        uint32_t fs_hz = f_hz * LUT_N; // sample update rate for DAC
        set_tim6_sample_rate_hz(fs_hz);
        HAL_Delay(SETTLE_MS);
        adc_half_done = 0; adc_full_done = 0;
        while (!adc_full_done) { }
        const uint16_t *blk = &adc_dma[ADC_BUF_LEN/2];
        uint32_t blk_len = ADC_BUF_LEN/2;
        float vout_pp, vin_pp;
        compute_pp_from_adc_block(blk, blk_len, &vout_pp, &vin_pp);
        float gain = (vin_pp > 1e-6f) ? (vout_pp / vin_pp) : 0.0f;
        float gain_db = (gain > 1e-9f) ? (20.0f * log10f(gain)) : -999.0f;
        char line[120];
        snprintf(line, sizeof(line), "%lu, %.4f, %.4f, %.4f, %.2f\r\n",
                 (unsigned long)f_hz, vin_pp, vout_pp, gain, gain_db);
        uart_print(line);
        HAL_Delay(100);
    }
    uart_print("Sweep done.\r\n");
    /* USER CODE END 2 */

    while (1) {
        HAL_Delay(1000);
    }
}
