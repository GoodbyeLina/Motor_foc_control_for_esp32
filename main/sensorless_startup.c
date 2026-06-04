#include "sensorless_startup.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "startup";

void startup_init(sensorless_startup_t *su, float I_startup,
                  float omega_start, float omega_end)
{
    // Step 1: 保存可配置参数
    su->I_startup = I_startup;
    su->omega_trans_start = omega_start;
    su->omega_trans_end   = omega_end;

    // Step 2: 调用 reset 清空所有内部状态
    startup_reset(su);

    // Step 3: 打印日志
    ESP_LOGI(TAG, "Startup init: I_startup=%.2fA, trans_start=%.0f, trans_end=%.0f (rad/s el)",
             I_startup, omega_start, omega_end);
}

void startup_run(sensorless_startup_t *su, float smo_omega, float smo_theta,
                 float target_vel, float dt)
{
    // ========================================================
    // 阶段①: I-F 开环强推
    //   作用: 零速时反电动势=0, SMO无法工作
    //         先由外部合成角度,用恒定电流把电机推到一定转速
    //   原理: theta_if = ∫ omega_if·dt  (自己积分合成角度)
    //         omega_if 从 0 逐步升到 omega_trans_start
    //         Iq_ref = I_startup (恒定电流)
    // ========================================================
    if (su->phase == STARTUP_IF) {
        // Step 1: 积分合成角度 (theta_if += omega_if * dt)
        su->theta_if += su->omega_if * dt;
        // 归一化到 [0, 2π], 防止长时间运行浮点溢出
        if (su->theta_if > 2.0f * (float)M_PI)
            su->theta_if -= 2.0f * (float)M_PI;
        else if (su->theta_if < 0.0f)
            su->theta_if += 2.0f * (float)M_PI;

        // Step 2: 频率逐步升高 (加速度 20 rad/s² 电角速度)
        su->omega_if += 20.0f * dt;
        // 限幅到过渡起始速度
        if (su->omega_if > su->omega_trans_start)
            su->omega_if = su->omega_trans_start;

        // Step 3: 输出给 FOC 引擎
        su->theta_used = su->theta_if;   // 使用开环合成角度
        su->omega_used = su->omega_if;   // 使用开环合成速度
        su->Iq_ref = su->I_startup;      // I-F 恒流模式

        // Step 4: 判断是否进入过渡阶段
        if (su->omega_if >= su->omega_trans_start) {
            su->phase = STARTUP_TRANSITION;
            ESP_LOGI(TAG, "I-F done, entering transition");
        }
        return;
    }

    // ========================================================
    // 阶段②: 混合过渡
    //   作用: 从开环平滑切换到 SMO 闭环
    //   原理: k 从 0→1 线性过渡
    //         θ_used = (1-k)·θ_if + k·θ_smo
    //         ω_used = (1-k)·ω_if + k·ω_smo
    // ========================================================
    if (su->phase == STARTUP_TRANSITION) {
        // Step 1: I-F 继续积分 (作为安全绳)
        su->theta_if += su->omega_if * dt;
        if (su->theta_if > 2.0f * (float)M_PI)
            su->theta_if -= 2.0f * (float)M_PI;
        else if (su->theta_if < 0.0f)
            su->theta_if += 2.0f * (float)M_PI;

        // Step 2: omega_if 继续升到 omega_trans_end
        if (su->omega_if < su->omega_trans_end) {
            su->omega_if += 20.0f * dt;
            if (su->omega_if > su->omega_trans_end)
                su->omega_if = su->omega_trans_end;
        }

        // Step 3: 计算混合系数 k (根据 SMO 速度)
        float k = (smo_omega - su->omega_trans_start) /
                  (su->omega_trans_end - su->omega_trans_start);
        if (k < 0.0f) k = 0.0f;
        if (k > 1.0f) k = 1.0f;

        // Step 4: 混合角度和速度
        su->theta_used = (1.0f - k) * su->theta_if + k * smo_theta;
        su->omega_used = (1.0f - k) * su->omega_if + k * smo_omega;
        su->Iq_ref = su->I_startup;  // 过渡期保持恒流

        // Step 5: 过渡完成 → 切闭环
        if (k >= 1.0f) {
            su->phase = STARTUP_CLOSED;
            ESP_LOGI(TAG, "Transition done, entering SMO closed-loop");
        }
        return;
    }

    // ========================================================
    // 阶段③: SMO 闭环
    //   作用: 完全使用 SMO 估算的角度和速度
    //         Iq_ref 设为 0 (由外部速度环 PID 覆写)
    // ========================================================
    if (su->phase == STARTUP_CLOSED) {
        // Step 1: 直接使用 SMO 的估算值
        su->theta_used = smo_theta;
        su->omega_used = smo_omega;

        // Step 2: Iq_ref 设为 0, 由外部速度环覆写
        su->Iq_ref = 0.0f;
        return;
    }
}


void startup_reset(sensorless_startup_t *su)
{
    // Step 1: 重置到 I-F 阶段
    su->phase = STARTUP_IF;

    // Step 2: 清空所有内部状态
    su->theta_if  = 0.0f;
    su->omega_if  = 0.0f;
    su->I_if      = su->I_startup;

    // Step 3: 清空输出
    su->theta_used = 0.0f;
    su->omega_used = 0.0f;
    su->Iq_ref     = 0.0f;

    // 注意: 保留参数 I_startup, omega_trans_start, omega_trans_end 不变
}

