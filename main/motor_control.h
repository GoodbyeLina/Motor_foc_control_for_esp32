#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "serial_cmd.h"  // for ctrl_mode_t

/**
 * 初始化电机控制 (PID实例)
 */
void motor_control_init(void);

/**
 * 传感器校准 (找零电角度)
 * 会输出 Uq=3V 保持 1 秒，然后记录偏移
 */
void motor_control_align(void);

/**
 * 单步控制循环 (应在 1kHz 循环中调用)
 * 内部根据当前控制模式执行开环/电流/速度闭环
 */
void motor_control_run(void);

// ========== 供串口命令调用的接口 ==========

void motor_control_set_target_velocity(float vel);
void motor_control_set_target_current(float cur);
void motor_control_set_mode(ctrl_mode_t mode);
void motor_control_stop(void);

float motor_control_get_velocity(void);
float motor_control_get_current(void);
float motor_control_get_voltage(void);

#endif
