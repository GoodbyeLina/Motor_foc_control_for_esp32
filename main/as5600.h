#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

/**
 * 初始化AS5600 (I2C)
 * SDA: GPIO 19, SCL: GPIO 18, 频率: 400kHz
 */
void as5600_init(void);

/**
 * 读取原始角度 (弧度)
 * @return 0 ~ 2π 之间的角度值
 */
float as5600_read_angle(void);

#endif

