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
#include "smo_observer.h"
#include "sensorless_startup.h"

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

// ========== SMO 观测器 ==========
static smo_observer_t s_smo;
static float s_Ualpha = 0.0f;      // 保存上次的 Uα (供 SMO 下次用)
static float s_Ubeta = 0.0f;       // 保存上次的 Uβ

// ========== 无感启动状态机 ==========
static sensorless_startup_t s_startup;
static float s_smo_omega_mech = 0.0f;  // SMO估算的机械角速度

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

        // SMO 参数初始化 (需根据电机参数调整)
    smo_init(&s_smo,
        1.5f,       // Rs: 定子电阻 (Ω) — 用万用表量
        0.0005f,    // Ls: 定子电感 (H) — 查规格书
        0.01f,      // Ke: 反电动势常数 — 可以先估一个
        0.8f,       // K_smo: 滑模增益 (需满足 Ts*K_smo/(Ls*δ)<1; 当前=0.001*0.8/(0.0005*2)=0.8<1 ✅)
        0.001f      // Ts: 控制周期 (s)
    );

    // 初始化启动状态机
    startup_init(&s_startup,
        0.5f,       // I_startup: 启动电流 (A)
        20.0f,      // omega_trans_start: 开始过渡电角速度 (rad/s)
        40.0f      // omega_trans_end: 完成过渡电角速度 (rad/s)
        
    );

    ESP_LOGI(TAG, "SMO + Startup initialized");

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
        
        case CTRL_MODE_SENSORLESS:{
            // 1.采样电流
            Ia = current_sense_read_a();
            Ib = current_sense_read_b();

            // 2.Clarke变换
            float Ialpha = Ia;
            float Ibeta = (Ia + 2 * Ib) / sqrtf(3);

                        // 3. SMO 一步更新
            smo_update(&s_smo, s_Ualpha, s_Ubeta, Ialpha, Ibeta);

            // ★ 调试打印：每100次（100ms）输出一次 SMO 状态 ★
            static int smo_debug_cnt = 0;
            if (++smo_debug_cnt % 100 == 0) {
                printf("[SMO] ω_smo=%.1f θ_smo=%.2f | ω_if=%.1f θ_if=%.2f | Eα=%.4f Eβ=%.4f | phase=%d\r\n",
                       s_smo.omega, s_smo.theta,
                       s_startup.omega_if, s_startup.theta_if,
                       s_smo.Ealpha, s_smo.Ebeta,
                       s_startup.phase);
            }

            // 4. 启动状态机运行
            //   用 SMO 的电角速度 smo_omega，但传入目标速度时注意单位
            //   target_vel 是机械角速度，smo.omega 是电角速度
            startup_run(&s_startup, s_smo.omega, s_smo.theta, s_target_vel * MOTOR_PP, 0.001f);

            // Step 5: 电角速度 → 机械角速度 (用于速度环)
            s_smo_omega_mech = s_smo.omega / MOTOR_PP;
            s_vel = s_smo_omega_mech; // 覆写速度值
            
            // Step 6: 计算 Iq (用启动状态机提供的角度)
            s_Iq = foc_calc_iq(Ia, Ib, s_startup.theta_used);
            
            // Step 7: 判断是否在 I-F 阶段
            if (s_startup.phase == STARTUP_IF ||
                    s_startup.phase == STARTUP_TRANSITION) {
                    // I-F / 过渡阶段：电流环跟踪启动电流
                    s_current_target = s_startup.Iq_ref;
                    Uq = pid_calculate(&s_pid_current, s_current_target - s_Iq);
                } else {
                    // SMO 闭环阶段：正常速度环 + 电流环
                    s_current_target = pid_calculate(&s_pid_velocity,
                                                    s_target_vel - s_smo_omega_mech);
                    Uq = pid_calculate(&s_pid_current, s_current_target - s_Iq);
                }
                
            // Step 8: 保存 Uα, Uβ (给下一次 SMO 用)
            //  从 Uq 和角度计算 Uα, Uβ
            float angle = s_startup.theta_used;
            s_Ualpha = -Uq * sinf(angle);
            s_Ubeta  =  Uq * cosf(angle);

                        // Step 9: SVPWM 输出 (用启动状态机的角度)
            s_Uq = Uq;
            foc_set_voltage(Uq, s_startup.theta_used);
            return;  // ← 跳过后面的公共输出代码(使用编码器角度)和VOFA+打印
            break;
        }

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
    // 在 case CTRL_MODE_SENSORLESS 时特殊处理
    if (mode == CTRL_MODE_SENSORLESS) {
        smo_reset(&s_smo);
        startup_reset(&s_startup);
        s_Ualpha = 0;
        s_Ubeta = 0;
    }
}

void motor_control_stop(void)
{
    s_target_vel = 0;
    s_target_cur = 0;
    foc_set_voltage(0, 0);
    // 停止时重置 PID
    pid_init(&s_pid_current, PID_P, PID_I, PID_D, PID_RAMP, PID_LIMIT);
    pid_init(&s_pid_velocity, VEL_P, VEL_I, VEL_D, VEL_RAMP, VEL_LIMIT);
    // 在停止函数末尾加：
    s_Ualpha = 0;
    s_Ubeta = 0;
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

// ========== 无感FOC调参实现 ==========

void motor_control_set_smo_params(float Rs, float Ls, float Ke, float K_smo)
{
    if (Rs > 0)     s_smo.Rs = Rs;
    if (Ls > 0)     s_smo.Ls = Ls;
    if (Ke > 0)     s_smo.Ke = Ke;
    if (K_smo > 0)  s_smo.K_smo = K_smo;
    smo_reset(&s_smo);
    ESP_LOGI(TAG, "SMO params: Rs=%.3f Ls=%.6f Ke=%.4f K_smo=%.1f",
             s_smo.Rs, s_smo.Ls, s_smo.Ke, s_smo.K_smo);
}

void motor_control_set_startup_params(float I_startup, float omega_start, float omega_end)
{
    if (I_startup > 0)   s_startup.I_startup = I_startup;
    if (omega_start > 0) s_startup.omega_trans_start = omega_start;
    if (omega_end > 0)   s_startup.omega_trans_end = omega_end;
    startup_reset(&s_startup);
    ESP_LOGI(TAG, "Startup params: I=%.2fA trans=%.0f->%.0f rad/s",
             s_startup.I_startup, s_startup.omega_trans_start, s_startup.omega_trans_end);
}
