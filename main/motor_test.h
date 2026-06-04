#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

/**
 * 测试状态查询
 */
uint8_t motor_test_is_running(void);

/**
 * 测试列表（通过串口命令触发）
 */
void motor_test_current_ramp(float start, float end, float step, float settle_s);
void motor_test_velocity_ramp(float start, float end, float step, float settle_s);
void motor_test_step_response(float target);
void motor_test_stop(void);

/**
 * 在 motor_control_run() 末尾调用（每控制周期 1 次）
 * 驱动测试状态机，自动推进阶梯
 */
void motor_test_tick(void);

/**
 * VOFA+ 数据输出抑制
 * 测试结束时暂停输出，让用户能看到汇总表
 */
uint8_t motor_test_vofa_suppressed(void);

#endif
