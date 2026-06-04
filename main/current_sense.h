#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

/**
 * 初始化电流采样
 * - 配置ADC1通道 (GPIO 39 = CH3, GPIO 36 = CH0)
 * - 设为12位精度，11dB衰减
 * - 校准零电流偏移（读1000次取平均）
 */
void current_sense_init(void);

/**
 * 读取A相电流 (A)
 * 公式: (ADC电压 - 零偏移) × (1 / 分流电阻 / 增益)
 *       = (ADC电压 - offset) × (1 / 0.01 / 50)
 *       = (ADC电压 - offset) × 2.0
 */
float current_sense_read_a(void);

/**
 * 读取B相电流 (A)
 */
float current_sense_read_b(void);

#endif