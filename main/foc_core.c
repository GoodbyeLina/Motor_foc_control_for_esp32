#include "foc_core.h"
#include "esp_log.h"
#include <math.h>
#include "ledc_pwm.h"
#include "esp_timer.h"

static const char *TAG = "foc_core";

// 保存电源电压，给 setPwm 用
static float g_vbus = 0.0f;

void foc_core_init(float power_supply)
{
    g_vbus = power_supply;
    ESP_LOGI(TAG, "FOC initialized, Vbus = %.1fV", g_vbus);
}



















































void foc_set_voltage(float Uq, float angle_el)
{
    // ---- 1. 限制 Uq 不超过 Vbus/√3 (SVPWM最大调制比) ----
    // SVPWM 通过零序分量注入，母线电压利用率比 SPWM 高 ~15%
    // 最大相电压幅值 = Vbus/√3 ≈ 0.577*Vbus (SPWM仅 Vbus/2 = 0.5*Vbus)
    float Uq_max = g_vbus / sqrtf(3);
    if (Uq > Uq_max) {
        Uq = Uq_max;
    } else if (Uq < -Uq_max) {
        Uq = -Uq_max;
    }

    // 电角度归一化到 0~2π
    angle_el = fmodf(angle_el, 2 * PI);
    if (angle_el < 0) angle_el += 2 * PI;

    // ---- 2. 逆Park变换 (Ud=0) ----
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // ---- 3. SVPWM：7段式对称PWM（零序分量注入） ----
    //
    // 核心思想：
    //   SPWM 固定加 Vbus/2 偏置 → 母线利用率低
    //   SVPWM 动态计算零序分量 → 自动插入零矢量，PWM 中心对齐
    //
    // 公式推导:   raw_a = Ualpha / Vbus
    //             raw_b = (-Ualpha + √3·Ubeta) / (2·Vbus)
    //             raw_c = (-Ualpha - √3·Ubeta) / (2·Vbus)
    //             t_offset = 0.5 - 0.5·(min(raw) + max(raw))
    //             dc = raw + t_offset

    float vbus = g_vbus;

    // 3a. 三相原始占空比（相对 0 电平，范围约 [-0.5, 0.5]）
    float raw_a =  Ualpha / vbus;
    float raw_b = (-Ualpha + sqrtf(3) * Ubeta) / (2.0f * vbus);
    float raw_c = (-Ualpha - sqrtf(3) * Ubeta) / (2.0f * vbus);

    // 3b. 零序分量：使三相中心对称于 0.5
    float t_min = fminf(raw_a, fminf(raw_b, raw_c));
    float t_max = fmaxf(raw_a, fmaxf(raw_b, raw_c));
    float t_offset = 0.5f - 0.5f * (t_min + t_max);

    // 3c. 注入零序分量 → 最终占空比 [0, 1]
    float dc_a = raw_a + t_offset;
    float dc_b = raw_b + t_offset;
    float dc_c = raw_c + t_offset;

    // 3d. 安全钳位（理论上 SVPWM 保证在 [0,1] 内，
    //     但浮点精度可能微幅越界）
    if (dc_a < 0.0f) dc_a = 0.0f;
    else if (dc_a > 1.0f) dc_a = 1.0f;
    if (dc_b < 0.0f) dc_b = 0.0f;
    else if (dc_b > 1.0f) dc_b = 1.0f;
    if (dc_c < 0.0f) dc_c = 0.0f;
    else if (dc_c > 1.0f) dc_c = 1.0f;

    // ---- 4. 输出 PWM ----
    ledc_pwm_set_duty(dc_a, dc_b, dc_c);
}

float foc_calc_iq(float Ia, float Ib, float angle_el){

    // Clark 变换
    float Ialpha = Ia;
    float Ibeta = (Ia + 2*Ib) / sqrtf(3);

    // Park 变换
    float sin_el = sinf(angle_el);
    float cos_el = cosf(angle_el);
    float Iq = -Ialpha * sin_el + Ibeta * cos_el;
    return Iq;

}

// foc_core.c 实现
static float g_shaft_angle = 0.0f;
static uint64_t g_open_loop_timestamp = 0;

void foc_openloop_velocity(float target_velocity)
{
    uint64_t now_us = esp_timer_get_time();
    float Ts = (now_us - g_open_loop_timestamp) / 1000000.0f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // 机械角度累加
    g_shaft_angle += target_velocity * Ts;
    // 归一化
    if (g_shaft_angle > 2 * PI) g_shaft_angle -= 2 * PI;
    if (g_shaft_angle < 0) g_shaft_angle += 2 * PI;

    // 电角度 = 机械角 × 极对数
    float angle_el = g_shaft_angle * 7;

    // 目标速度为0时，不输出电压，让电机自由停止
    float Uq;
    if (fabsf(target_velocity) < 0.01f) {
        Uq = 0.0f;
    } else {
        Uq = g_vbus / 3.0f;
    }

    // 调用已有的 foc_set_voltage
    foc_set_voltage(Uq, angle_el);

    g_open_loop_timestamp = now_us;
}

// 速度计算相关静态变量
static float g_last_angle = 0.0f;
static float g_velocity_lpf = 0.0f;
static uint64_t g_last_vel_time = 0;
static bool g_vel_first = true;
float foc_calc_velocity(float angle)
{
    uint64_t now = esp_timer_get_time();

    // 第一次调用：只记录初始值，返回0
    if (g_vel_first) {
        g_last_angle = angle;
        g_last_vel_time = now;
        g_vel_first = false;
        return 0.0f;
    }

    // 计算时间差 (秒)
    float dt = (now - g_last_vel_time) / 1000000.0f;
    if (dt <= 0 || dt > 0.5f) dt = 1e-3f;

    // 计算角度差 (处理2π→0跳变)
    float d_angle = angle - g_last_angle;
    if (d_angle > PI)  d_angle -= 2 * PI;
    if (d_angle < -PI) d_angle += 2 * PI;

    // 原始速度
    float vel_raw = d_angle / dt;

        // 低通滤波: 0.95 * 上次 + 0.05 * 本次 (加强滤波，抑制编码器量化噪声)
    g_velocity_lpf = 0.95f * g_velocity_lpf + 0.05f * vel_raw;

    // 更新状态
    g_last_angle = angle;
    g_last_vel_time = now;

    return g_velocity_lpf;
}

