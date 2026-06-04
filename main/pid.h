#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float P;            // 比例增益
    float I;            // 积分增益
    float D;            // 微分增益
    float output_ramp;  // 输出变化率限制 (0=禁用)
    float limit;        // 输出限幅

    // 内部状态 (不用你初始化，由 pid_init 负责)
    float error_prev;
    float output_prev;
    float integral_prev;
    uint64_t timestamp_prev;  // 微秒级时间戳
} pid_controller_t;

/**
 * 初始化PID控制器
 */
void pid_init(pid_controller_t *pid, float P, float I, float D, float ramp, float limit);

/**
 * 计算PID输出
 * @param pid   PID控制器指针
 * @param error  目标值 - 当前值
 * @return       经过限幅后的PID输出值
 */
float pid_calculate(pid_controller_t *pid, float error);

#endif