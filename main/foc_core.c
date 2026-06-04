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
    // ---- 1. 限制 Uq 不超过 Vbus/2 ----
    // 用 if 或 fminf/fmaxf
    // Uq 范围应该在 -Vbus/2 ~ +Vbus/2 之间
    if(Uq > g_vbus / 2){
        Uq = g_vbus / 2;
    } else if(Uq < -g_vbus / 2){
        Uq = -g_vbus / 2;
    }

    // 电角度归一化到 0~2π
    angle_el = fmodf(angle_el, 2 * PI);
    if (angle_el < 0) angle_el += 2 * PI;

    // ---- 2. 逆Park变换 (Ud=0) ----
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // ---- 3. 逆Clarke变换 → 三相电压 ----
    float Ua = Ualpha + g_vbus / 2;
    float Ub = (-Ualpha + sqrtf(3)* Ubeta) / 2 +g_vbus / 2;
    float Uc = (-Ualpha - sqrtf(3)* Ubeta) / 2 +g_vbus / 2;

    // ---- 4. 限幅到 0 ~ Vbus ----
    // 用 if 判断 + 裁剪
    if(Ua > g_vbus){
        Ua = g_vbus;
    
    } else if(Ua < 0){
        Ua = 0;
    }
    if(Ub > g_vbus){
        Ub = g_vbus;
    
    } else if(Ub < 0){
        Ub = 0;
    }
    if(Uc > g_vbus){
        Uc = g_vbus;
    
    } else if(Uc < 0){
        Uc = 0;
    }

    // ---- 5. 计算占空比并输出PWM ----
    float dc_a = Ua / g_vbus;
    float dc_b = Ub / g_vbus;
    float dc_c = Uc / g_vbus;
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

