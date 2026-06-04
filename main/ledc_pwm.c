#include "driver/ledc.h"
#include "ledc_pwm.h"
#include "esp_err.h"

void ledc_pwm_init(void)
{
    // 1. 配置定时器 (30kHz, 8位)
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz         = 30000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 2. 配置3个PWM通道 (每个通道独立配置，不使用循环)
    //    ledc_channel_config 会自动设置GPIO模式为输出
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0
    };

    // 通道0 -> GPIO 32 (A相)
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.gpio_num = 32;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    // 通道1 -> GPIO 33 (B相)
    ch_conf.channel = LEDC_CHANNEL_1;
    ch_conf.gpio_num = 33;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    // 通道2 -> GPIO 25 (C相)
    ch_conf.channel = LEDC_CHANNEL_2;
    ch_conf.gpio_num = 25;
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}


void ledc_pwm_set_duty(float dc_a, float dc_b, float dc_c){

    // 将占空比从0.0~1.0转换为0~255
    uint32_t duty_a = (uint32_t)(dc_a * 255);
    uint32_t duty_b = (uint32_t)(dc_b * 255);
    uint32_t duty_c = (uint32_t)(dc_c * 255);

    // 设置A相占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_a);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    // 设置B相占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // 设置C相占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

}




