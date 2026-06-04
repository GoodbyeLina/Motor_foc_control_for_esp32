// main/smo_observer.c
#include "smo_observer.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "smo_obs";

/**
 * 饱和函数: 在边界层 (-delta ~ delta) 内线性过渡
 * 代替 sign() 函数，有效抑制滑模抖振 (chattering)
 */
static inline float sat(float x, float delta)
{
    // TODO: 实现饱和函数
    // 如果 x > delta 返回 1.0
    // 如果 x < -delta 返回 -1.0
    // 否则返回 x / delta
    if(x > delta){
        return 1.0f;
    } else if(x < -delta){
        return -1.0f;
    } else {
        return x /delta;
    }
}

void smo_init(smo_observer_t *smo, float Rs, float Ls, float Ke,
              float K_smo, float Ts)
{
    // TODO: 
    // 1. 保存所有参数到 smo 结构体
    smo->Rs = Rs;
    smo->Ls = Ls;
    smo->Ke = Ke;
    smo->K_smo = K_smo;
    smo->Ts = Ts;

    // 2. 设置 F_lpf = 200.0f (默认)
    smo->F_lpf = 200.0f;
    // 3. 设置 F_pll_bw = 100.0f (默认)
    smo->F_pll_bw = 100.0f;
    // 4. 调用 smo_reset() 清空状态
    smo_reset(smo);
    // 5. 打印初始化日志
    ESP_LOGI(TAG, "SMO initialized: Rs=%.3fΩ, Ls=%.3fmH, Ke=%.3fV·s/rad, K_smo=%.1f, Ts=%.4fms",
             Rs, Ls * 1e3f, Ke, K_smo, Ts * 1e3f);
}

void smo_update(smo_observer_t *smo, float Ualpha, float Ubeta,
                float Ialpha, float Ibeta)
{
    float Ts = smo->Ts;
    float delta = 2.0f;   // 边界层厚度 (原0.05过小;需满足 Ts*K_smo/(Ls*δ) < 1 才能稳定)

    // ===== Step 1: 电流观测器 (滑模) =====
    // Ierr = Iest - I
    // z = K_smo * sat(Ierr, delta)
    // Iest += Ts * (-Rs/Ls * Iest + 1/Ls * U - 1/Ls * z)
    //
    // TODO: 实现上面公式，更新 Ialpha_est, Ibeta_est
    //       保存 zalpha, zbeta

    float Ialpha_err = smo->Ialpha_est - Ialpha;
    float Ibeta_err = smo->Ibeta_est - Ibeta;
    smo->zalpha = smo->K_smo * sat(Ialpha_err, delta);
    smo->zbeta = smo->K_smo * sat(Ibeta_err, delta);
    smo->Ialpha_est += Ts * (-smo->Rs / smo->Ls * smo->Ialpha_est + 1.0f / smo->Ls * Ualpha - 1.0f / smo->Ls * smo->zalpha);
    smo->Ibeta_est += Ts * (-smo->Rs / smo->Ls * smo->Ibeta_est + 1.0f / smo->Ls * Ubeta - 1.0f / smo->Ls * smo->zbeta);



    // ===== Step 2: 反电动势低通滤波 =====
    // lpf_gain = Ts * F_lpf  (限幅到 1.0)
    // E += lpf_gain * (z - E)
    //
    // TODO: 实现一阶 LPF
    float lpf_gain = Ts * smo->F_lpf;
    if(lpf_gain > 1.0f) {
        lpf_gain = 1.0f;
    } 
    smo->Ealpha += lpf_gain * (smo->zalpha - smo->Ealpha);
    smo->Ebeta += lpf_gain * (smo->zbeta - smo->Ebeta);

    // ===== Step 3: PLL 跟踪角度和速度 =====
    // theta_raw = atan2(-Ealpha, Ebeta)
    // err = theta_raw - theta
    // 归一化 err 到 [-PI, PI]
    //
    // Kp_pll = 2 * F_pll_bw
    // Ki_pll = F_pll_bw * F_pll_bw
    //
    // pll_out = Kp_pll * err + pll_integral
    // pll_integral += Ki_pll * Ts * err  (梯形积分更好)
    // 限幅 pll_integral 到 ±1000
    //
    // omega = pll_out
    // theta += omega * Ts
    // 归一化 theta 到 [0, 2PI]
    //
    // TODO: 实现 PLL
    float theta_raw = atan2f(-smo->Ealpha, smo->Ebeta);
    float err = theta_raw - smo->theta;
    // 归一化 err 到 [-PI, PI]
    // 改成 if 更高效
    if (err > M_PI)       err -= 2.0f * M_PI;
    else if (err < -M_PI) err += 2.0f * M_PI;

    float Kp_pll = 2.0f * smo->F_pll_bw;
    float Ki_pll = smo->F_pll_bw * smo->F_pll_bw;
    float pll_out = Kp_pll * err + smo->pll_integral;
    smo->pll_integral += Ki_pll * Ts * err;
    // 限幅 pll_integral 到 ±1000
    if(smo->pll_integral > 1000.0f){
        smo->pll_integral = 1000.0f;
    } else if(smo->pll_integral < -1000.0f){
        smo->pll_integral = -1000.0f;
    }
    smo->omega = pll_out;
        smo->theta += smo->omega * Ts;
    // 添加 LPF 相位补偿: 每个周期只补偿一次
    // phase_comp = arctan(ω/F_lpf) 为 LPF 引入的相位滞后
    float phase_comp = atan2f(smo->omega, smo->F_lpf);
        // 对缓变信号才加补偿, omega异常(NaN/过大)时跳过
    if (!isnan(smo->omega) && !isinf(smo->omega) && fabsf(smo->omega) < 500.0f) {
        smo->theta += phase_comp;
    }
    // 归一化 theta 到 [0, 2PI]
    if (smo->theta >= 2.0f * M_PI) smo->theta -= 2.0f * M_PI;
    else if (smo->theta < 0)        smo->theta += 2.0f * M_PI;
}

void smo_reset(smo_observer_t *smo)
{
    // TODO: 把所有内部状态清零
    // Ialpha_est, Ibeta_est, Ealpha, Ebeta
    // zalpha, zbeta, theta, omega
    // pll_integral, pll_err_prev, timestamp
    smo->Ialpha_est = 0.0f;
    smo->Ibeta_est = 0.0f;
    smo->Ealpha = 0.0f;
    smo->Ebeta = 0.0f;
    smo->zalpha = 0.0f;
    smo->zbeta = 0.0f;
    smo->theta = 0.0f;
    smo->omega = 0.0f;
    smo->pll_integral = 0.0f;
    smo->pll_err_prev = 0.0f;
    smo->timestamp = 0;

}