#include "driver/ledc.h"
#include "ledc_pwm.h"
// 需要用到的结构体和函数：



void ledc_pwm_init(void){
    // 配置定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz         = 30000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // 用数组配置3个通道：GPIO 32(A), 33(B), 25(C)
    uint8_t gpios[3] = {32, 33, 25};
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0
    };
    for(int i = 0; i < 3; i++){
        ch_conf.channel = LEDC_CHANNEL_0 + i;  // 通道0,1,2
        ch_conf.gpio_num = gpios[i];
    ledc_channel_config(&ch_conf);
    }
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




