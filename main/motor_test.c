#include "motor_test.h"
#include "motor_control.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==================== 测试状态机 ====================

typedef enum {
    TEST_IDLE = 0,
    TEST_CURRENT_RAMP,
    TEST_VELOCITY_RAMP,
    TEST_STEP_RESPONSE,
} test_type_t;

#define MAX_TEST_STEPS 30

typedef struct {
    float setpoint;     // 设定的值（c 或 v）
    float vel;          // 实测速度
    float iq;           // 实测 Iq
    float uq;           // 实测 Uq
} test_data_point_t;

static struct {
    test_type_t type;
    uint8_t     running;

    // 阶梯参数
    float  start_val;
    float  end_val;
    float  step;
    int    total_steps;
    int    current_step;

    // 稳态度量
    float  settle_time_s;      // 每档等待时间（秒）
    uint32_t settle_cycles;    // 已等待的控制周期数
    uint32_t cycles_needed;    // 需要等待的总周期数

    // 当前值
    float  setpoint;           // 当前设置的 v 或 c 值

    // 数据记录（测试结束统一打印）
    test_data_point_t data[MAX_TEST_STEPS];
    int data_count;
    uint8_t vofa_suppress;   // 暂停 VOFA 输出，让用户看结果
    uint32_t suppress_count; // 抑制计数器
} s_test;

#define SUPPRESS_DURATION 3000  // 抑制 3 秒（3000 个控制周期）

// ==================== 内部实现 ====================

static void test_begin(test_type_t type, float start, float end, float step, float settle_s)
{
    s_test.type          = type;
    s_test.running       = 1;
    s_test.start_val     = start;
    s_test.end_val       = end;
    s_test.step          = step;
    s_test.total_steps   = (int)((end - start) / step + 0.5f) + 1;
    s_test.current_step  = 0;
    s_test.settle_time_s = settle_s;
    s_test.settle_cycles = 0;
    s_test.cycles_needed = (uint32_t)(settle_s * 1000);  // 1kHz 控制频率
    s_test.setpoint      = start;
    s_test.data_count     = 0;
    s_test.vofa_suppress  = 0;  // 新测试开始时恢复 VOFA 输出
    s_test.suppress_count = 0;

    // 设置初始值
    if (type == TEST_CURRENT_RAMP) {
        motor_control_set_target_current(start);
        printf("[TST] 电流阶梯测试开始: %.2f → %.2f, 步长 %.2f, 每档 %.1f秒\r\n",
               start, end, step, settle_s);
    } else if (type == TEST_VELOCITY_RAMP) {
        motor_control_set_target_velocity(start);
        printf("[TST] 速度阶梯测试开始: %.0f → %.0f, 步长 %.0f, 每档 %.1f秒\r\n",
               start, end, step, settle_s);
    } else if (type == TEST_STEP_RESPONSE) {
        motor_control_set_target_velocity(start);
        printf("[TST] 阶跃测试: v=%.0f (无阶梯，请观察 VOFA+ 波形)\r\n", start);
        // 阶跃测试不是阶梯，直接开始，无结束条件
    }
}

static void test_record_step(void)
{
    if (s_test.data_count >= MAX_TEST_STEPS) return;

    int i = s_test.data_count;
    s_test.data[i].setpoint = s_test.setpoint;
    s_test.data[i].vel      = motor_control_get_velocity();
    s_test.data[i].iq       = motor_control_get_current();
    s_test.data[i].uq       = motor_control_get_voltage();
    s_test.data_count++;
}

static void test_print_summary(void)
{
    printf("\r\n");
    printf("========== 测试结果汇总 ==========\r\n");

    if (s_test.type == TEST_CURRENT_RAMP) {
        printf(" 目标电流  | 实测Iq  | Uq   | 速度   \r\n");
        printf("-----------+---------+------+--------\r\n");
        for (int i = 0; i < s_test.data_count; i++) {
            printf(" %8.3f  | %7.4f | %5.2f | %6.1f\r\n",
                   s_test.data[i].setpoint,
                   s_test.data[i].iq,
                   s_test.data[i].uq,
                   s_test.data[i].vel);
        }
    } else if (s_test.type == TEST_VELOCITY_RAMP) {
        printf(" 目标速度  | 实测速度 | 误差%% | Iq    | Uq   \r\n");
        printf("-----------+----------+-------+-------+------\r\n");
        for (int i = 0; i < s_test.data_count; i++) {
            float err = 0;
            if (s_test.data[i].setpoint != 0) {
                err = (s_test.data[i].vel - s_test.data[i].setpoint) / s_test.data[i].setpoint * 100;
            }
            printf(" %5.0f     | %7.1f  | %5.1f%% | %5.4f | %5.2f\r\n",
                   s_test.data[i].setpoint,
                   s_test.data[i].vel,
                   err,
                   s_test.data[i].iq,
                   s_test.data[i].uq);
        }
    }
    printf("==================================\r\n");
    printf("\r\n");
}

static void test_advance(void)
{
    if (s_test.type == TEST_CURRENT_RAMP) {
        // 推进到下一个电流值
        s_test.current_step++;
        float new_val = s_test.start_val + s_test.current_step * s_test.step;
        if (new_val > s_test.end_val + 0.001f || s_test.current_step >= s_test.total_steps) {
            printf("[TST] 电流阶梯测试完成\r\n");
            motor_test_stop();
            return;
        }
        s_test.setpoint = new_val;
        motor_control_set_target_current(new_val);

    } else if (s_test.type == TEST_VELOCITY_RAMP) {
        // 推进到下一个速度值
        s_test.current_step++;
        float new_val = s_test.start_val + s_test.current_step * s_test.step;
        if (new_val > s_test.end_val + 0.001f || s_test.current_step >= s_test.total_steps) {
            printf("[TST] 速度阶梯测试完成\r\n");
            motor_test_stop();
            return;
        }
        s_test.setpoint = new_val;
        motor_control_set_target_velocity(new_val);
    }

    // 重置计时器
    s_test.settle_cycles = 0;
}

// ==================== 公开接口 ====================

uint8_t motor_test_is_running(void)
{
    return s_test.running;
}

void motor_test_current_ramp(float start, float end, float step, float settle_s)
{
    // 如果有上一轮测试，自动停止
    if (s_test.running) {
        motor_test_stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // 确保切换到电流环
    motor_control_set_mode(1);
    test_begin(TEST_CURRENT_RAMP, start, end, step, settle_s);
}

void motor_test_velocity_ramp(float start, float end, float step, float settle_s)
{
    if (s_test.running) {
        motor_test_stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // 确保切换到速度环
    motor_control_set_mode(2);
    test_begin(TEST_VELOCITY_RAMP, start, end, step, settle_s);
}

void motor_test_step_response(float target)
{
    if (s_test.running) {
        motor_test_stop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // 先停止再启动阶跃
    motor_control_stop();
    vTaskDelay(pdMS_TO_TICKS(500));
    motor_control_set_mode(2);
    test_begin(TEST_STEP_RESPONSE, target, target, 0, 0);
}

void motor_test_stop(void)
{
    if (!s_test.running) return;

    // 暂停 VOFA 输出，防止汇总表被刷掉
    s_test.vofa_suppress = 1;
    s_test.suppress_count = 0;

    // 停止电机
    motor_control_stop();

    // 有数据才打印汇总表（阶跃测试没有阶梯数据，不打印）
    if (s_test.data_count > 0) {
        test_print_summary();
    }

    printf("[TST] 测试已停止（VOFA 输出暂停 3 秒，输入任意命令恢复）\r\n");
    s_test.running  = 0;
    s_test.type     = TEST_IDLE;
    s_test.current_step = 0;
    s_test.settle_cycles = 0;
    s_test.data_count = 0;
}

uint8_t motor_test_vofa_suppressed(void)
{
    return s_test.vofa_suppress;
}

/**
 * 在 motor_control_run() 末尾调用
 * 每 1ms 执行一次
 */
void motor_test_tick(void)
{
    // VOFA 抑制计时（3秒后自动恢复）
    if (s_test.vofa_suppress) {
        s_test.suppress_count++;
        if (s_test.suppress_count >= SUPPRESS_DURATION) {
            s_test.vofa_suppress = 0;
            printf("[TST] VOFA 输出已恢复\r\n");
        }
        return;  // 抑制期间不执行测试逻辑
    }

    if (!s_test.running) return;

    // 阶跃响应测试只需要设一次值，不自动推进
    if (s_test.type == TEST_STEP_RESPONSE) {
        // 测试首次进入时打印提示
        if (s_test.settle_cycles == 0) {
            printf("[TST] 阶跃测试进行中: v=%.0f (等稳定后输入 test_stop 结束)\r\n",
                   s_test.setpoint);
        }
        s_test.settle_cycles++;
        return;
    }

    // 阶梯测试：等待 settle 时间后推进
    s_test.settle_cycles++;
    if (s_test.settle_cycles >= s_test.cycles_needed) {
        // 已稳定，记录数据并推进
        test_record_step();
        test_advance();
    }
}
