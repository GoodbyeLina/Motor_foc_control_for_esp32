#include "current_sense.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "current_sense";

// ADC参数
#define ADC_VREF       3.3f            // 参考电压 3.3V
#define ADC_RESOLUTION 4095.0f         // 12位最大值

// 电流检测参数
#define SHUNT_RESISTOR    0.01f        // 分流电阻 0.01Ω
#define AMP_GAIN          50.0f        // 运放增益 50倍
#define VOLTS_TO_AMPS     (1.0f / (SHUNT_RESISTOR * AMP_GAIN))  // ≈ 2.0

// 零电流偏移
static float offset_a = 0.0f;
static float offset_b = 0.0f;

/**
 * 读取指定ADC通道的电压值
 */
static float read_adc_voltage(int channel)
{
    int raw = adc1_get_raw(channel);
    return (float)raw * ADC_VREF / ADC_RESOLUTION;
}

void current_sense_init(void)
{
    // 1. 配置ADC宽度（12位）
    adc1_config_width(ADC_WIDTH_BIT_12);

    // 2. 配置A相通道 (GPIO 39 = ADC1_CHANNEL_3)
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

    // 3. 配置B相通道 (GPIO 36 = ADC1_CHANNEL_0)
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    // 4. 校准零偏移（读1000次取平均）
    const int cal_rounds = 1000;
    for (int i = 0; i < cal_rounds; i++) {
        offset_a += read_adc_voltage(ADC1_CHANNEL_3);
        offset_b += read_adc_voltage(ADC1_CHANNEL_0);
    }
    offset_a /= cal_rounds;
    offset_b /= cal_rounds;

    ESP_LOGI(TAG, "Initialized, offset_a=%.4fV offset_b=%.4fV", offset_a, offset_b);
}

float current_sense_read_a(void)
{
    float voltage = read_adc_voltage(ADC1_CHANNEL_3);
    return (voltage - offset_a) * VOLTS_TO_AMPS;
}

float current_sense_read_b(void)
{
    float voltage = read_adc_voltage(ADC1_CHANNEL_0);
    return (voltage - offset_b) * VOLTS_TO_AMPS;
}