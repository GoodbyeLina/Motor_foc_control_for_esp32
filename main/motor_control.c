#include "motor_control.h"
#include "motor_test.h"
#include "ledc_pwm.h"
#include "foc_core.h"
#include "as5600.h"
#include "current_sense.h"
#include "pid.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor_ctrl";

#define MOTOR_PP     7
#define VBUS        12.6f
#define SENSOR_DIR  1

// ========== PID 参数 ==========
#define PID_P    1.5f
#define PID_I    50.0f
#define PID_D    0.0f
#define PID_RAMP 100000.0f
#define PID_LIMIT (VBUS / sqrtf(3))  // SVPWM 最大调制比 ≈ 0.577*Vbus

#define VEL_P     0.15f
#define VEL_I     3.0f
#define VEL_D     0.0f
#define VEL_RAMP  500.0f
#define VEL_LIMIT 1.0f

// ========== 静态变量（模块内部状态） ==========
static pid_controller_t s_pid_current;
static pid_controller_t s_pid_velocity;
static float s_zero_angle = 0.0f;

static uint32_t s_loop_count = 0;
static float s_target_vel = 5.0f;
static float s_target_cur = 0.5f;
static float s_vel = 0.0f;
static float s_Iq  = 0.0f;
static float s_Uq  = 0.0f;
static float s_current_target = 0.0f;  // 电流环的输入目标（电流环=c值，速度环=速度PID输出）
static ctrl_mode_t s_ctrl_mode = CTRL_MODE_VELOCITY;

// ==================== 公开接口 ====================

void motor_control_init(void)
{
    pid_init(&s_pid_current, PID_P, PID_I, PID_D, PID_RAMP, PID_LIMIT);
    pid_init(&s_pid_velocity, VEL_P, VEL_I, VEL_D, VEL_RAMP, VEL_LIMIT);
    ESP_LOGI(TAG, "PID controllers initialized");
}

void motor_control_align(void)
{
    ESP_LOGI(TAG, "Aligning sensor...");

    // 输出 Uq=3V 对准 90° 方向
    foc_set_voltage(3.0f, 3 * PI / 2);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 读取编码器角度作为零电角度
    s_zero_angle = as5600_read_angle() * SENSOR_DIR * MOTOR_PP;

    // 停止输出
    foc_set_voltage(0, 0);
    ESP_LOGI(TAG, "Zero angle = %.2f rad", s_zero_angle);
}

void motor_control_run(void)
{
    // ① 读编码器 → 电角度
    float mech_angle = as5600_read_angle();
    float elec_angle = mech_angle * SENSOR_DIR * MOTOR_PP - s_zero_angle;
    elec_angle = fmodf(elec_angle, 2 * PI);
    if (elec_angle < 0) elec_angle += 2 * PI;

    // ② 算速度
    s_vel = foc_calc_velocity(mech_angle);

    // ③ 根据控制模式执行不同的控制
    float Uq = 0;
    float Ia = 0, Ib = 0;

    switch (s_ctrl_mode) {
        case CTRL_MODE_OPENLOOP:
            // 开环: 无需编码器和电流
            foc_openloop_velocity(s_target_vel);
            return;

        case CTRL_MODE_CURRENT:
            // 电流闭环: 单电流环
            Ia = current_sense_read_a();
            Ib = current_sense_read_b();
            s_Iq = foc_calc_iq(Ia, Ib, elec_angle);
            s_current_target = s_target_cur;  // 电流环的目标就是用户设的 c 值
            Uq = pid_calculate(&s_pid_current, s_current_target - s_Iq);
            break;

        case CTRL_MODE_VELOCITY:
            // 速度闭环: 速度环 + 电流环
            Ia = current_sense_read_a();
            Ib = current_sense_read_b();
            s_Iq = foc_calc_iq(Ia, Ib, elec_angle);
            s_current_target = pid_calculate(&s_pid_velocity, s_target_vel - s_vel);  // 速度PID输出 = 电流目标
            Uq = pid_calculate(&s_pid_current, s_current_target - s_Iq);
            break;
    }

    // ④ 输出
    s_Uq = Uq;
    foc_set_voltage(Uq, elec_angle);

    // ⑤ 状态打印 (VOFA+ FireWater 格式, 每10次输出)
    //    测试汇总表输出期间暂停，方便用户查看
    if (s_loop_count++ % 10 == 0 && !motor_test_vofa_suppressed()) {
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
               s_vel, s_target_vel, s_Iq, s_Uq, elec_angle, Ia, Ib, s_current_target);
               
    }

    // ⑥ 驱动测试状态机（无测试运行时立即返回）
    motor_test_tick();
}

// ========== 串口命令接口 ==========

void motor_control_set_target_velocity(float vel)
{
    s_target_vel = vel;
}

void motor_control_set_target_current(float cur)
{
    s_target_cur = cur;
}

void motor_control_set_mode(ctrl_mode_t mode)
{
    s_ctrl_mode = mode;
    // 切换模式时重置 PID，防止积分饱和
    pid_init(&s_pid_current, PID_P, PID_I, PID_D, PID_RAMP, PID_LIMIT);
    pid_init(&s_pid_velocity, VEL_P, VEL_I, VEL_D, VEL_RAMP, VEL_LIMIT);
}

void motor_control_stop(void)
{
    s_target_vel = 0;
    s_target_cur = 0;
    foc_set_voltage(0, 0);
    // 停止时重置 PID
    pid_init(&s_pid_current, PID_P, PID_I, PID_D, PID_RAMP, PID_LIMIT);
    pid_init(&s_pid_velocity, VEL_P, VEL_I, VEL_D, VEL_RAMP, VEL_LIMIT);
}

float motor_control_get_velocity(void) { return s_vel; }
float motor_control_get_current(void)  { return s_Iq; }
float motor_control_get_voltage(void)  { return s_Uq; }

// ========== 在线调参实现 ==========

void motor_control_set_current_pid(float p, float i, float d, float ramp, float limit)
{
    if (p > 0)    s_pid_current.P = p;
    if (i > 0)    s_pid_current.I = i;
    if (d > 0)    s_pid_current.D = d;
    if (ramp > 0) s_pid_current.output_ramp = ramp;
    if (limit > 0) s_pid_current.limit = limit;
    // 重置积分状态，防止突变
    pid_init(&s_pid_current, s_pid_current.P, s_pid_current.I, s_pid_current.D,
             s_pid_current.output_ramp, s_pid_current.limit);
    ESP_LOGI(TAG, "电流环 PID: P=%.3f I=%.3f D=%.3f ramp=%.0f limit=%.3f",
             s_pid_current.P, s_pid_current.I, s_pid_current.D,
             s_pid_current.output_ramp, s_pid_current.limit);
}

void motor_control_get_current_pid(float *p, float *i, float *d, float *ramp, float *limit)
{
    *p     = s_pid_current.P;
    *i     = s_pid_current.I;
    *d     = s_pid_current.D;
    *ramp  = s_pid_current.output_ramp;
    *limit = s_pid_current.limit;
}

void motor_control_set_velocity_pid(float p, float i, float d, float ramp, float limit)
{
    if (p > 0)    s_pid_velocity.P = p;
    if (i > 0)    s_pid_velocity.I = i;
    if (d > 0)    s_pid_velocity.D = d;
    if (ramp > 0) s_pid_velocity.output_ramp = ramp;
    if (limit > 0) s_pid_velocity.limit = limit;
    pid_init(&s_pid_velocity, s_pid_velocity.P, s_pid_velocity.I, s_pid_velocity.D,
             s_pid_velocity.output_ramp, s_pid_velocity.limit);
    ESP_LOGI(TAG, "速度环 PID: P=%.3f I=%.3f D=%.3f ramp=%.0f limit=%.3f",
             s_pid_velocity.P, s_pid_velocity.I, s_pid_velocity.D,
             s_pid_velocity.output_ramp, s_pid_velocity.limit);
}

void motor_control_get_velocity_pid(float *p, float *i, float *d, float *ramp, float *limit)
{
    *p     = s_pid_velocity.P;
    *i     = s_pid_velocity.I;
    *d     = s_pid_velocity.D;
    *ramp  = s_pid_velocity.output_ramp;
    *limit = s_pid_velocity.limit;
}
