#ifndef LEDC_PWM_H
#define LEDC_PWM_H

/**
 * 初始化三路PWM输出
 * 频率: 30kHz, 精度: 8位
 * GPIO: 32 (A相), 33 (B相), 25 (C相)
 */
void ledc_pwm_init(void);

/**
 * 设置三相PWM占空比
 * @param dc_a  A相占空比 (0.0 ~ 1.0)
 * @param dc_b  B相占空比 (0.0 ~ 1.0)
 * @param dc_c  C相占空比 (0.0 ~ 1.0)
 */
void ledc_pwm_set_duty(float dc_a, float dc_b, float dc_c);

#endif
