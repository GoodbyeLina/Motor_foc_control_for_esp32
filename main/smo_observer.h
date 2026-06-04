#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include <stdint.h>

/**
 * ==========================================================
 *  滑模观测器 (Sliding Mode Observer) - 无感FOC核心
 *  
 *  原理:
 *    通过检测电机端电压和相电流，用滑模观测器估算反电动势 (BEMF)，
 *    再通过 PLL 从反电动势中提取转子角度和速度。
 *  
 *  电机模型 (α-β坐标系):
 *    dIα/dt = (-Rs/Ls)·Iα + (1/Ls)·Uα - (1/Ls)·Eα
 *    dIβ/dt = (-Rs/Ls)·Iβ + (1/Ls)·Uβ - (1/Ls)·Eβ
 *    
 *    其中 Eα = -Ke·ω·sin(θ), Eβ = Ke·ω·cos(θ) 为反电动势
 *  
 *  观测器方程:
 *    dIα_est/dt = (-Rs/Ls)·Iα_est + (1/Ls)·Uα - (K/Ls)·sat(Iα_est - Iα)
 *    dIβ_est/dt = (-Rs/Ls)·Iβ_est + (1/Ls)·Uβ - (K/Ls)·sat(Iβ_est - Iβ)
 *    
 *    其中 sat(x) 为饱和函数（代替 sign 函数，减小抖振）
 *    
 *  反电动势提取 (低通滤波):
 *    Eα_est = LPF(zα), Eβ_est = LPF(zβ)
 *    其中 zα = K·sat(Iα_est - Iα), zβ = K·sat(Iβ_est - Iβ)
 *    
 *  角度估算:
 *    θ_est = atan2(-Eα_est, Eβ_est)
 *    
 *  使用 PLL 平滑跟踪:
 *    PLL 从 θ_est 中滤除噪声，输出平滑的 θ 和 ω
 * ==========================================================
 */

/** SMO 滑模观测器结构体 */
typedef struct {
    // ====== 电机参数 (需根据实际电机设置) ======
    float Rs;           // 定子电阻 (Ω)
    float Ls;           // 定子电感 (H)
    float Ke;           // 反电动势常数 (V·s/rad)

    // ====== SMO 参数 ======
    float K_smo;        // 滑模增益
    float F_lpf;        // 反电动势 LPF 截止频率 (rad/s)
    float F_pll_bw;     // PLL 带宽 (rad/s)

    // ====== SMO 内部状态 ======
    float Ialpha_est;   // 估计的 α 轴电流 (A)
    float Ibeta_est;    // 估计的 β 轴电流 (A)
    float Ealpha;       // 估计的 α 轴反电动势 (V)
    float Ebeta;        // 估计的 β 轴反电动势 (V)
    float zalpha;       // 滑模切换函数输出 α (V)
    float zbeta;        // 滑模切换函数输出 β (V)

    // ====== PLL 状态 ======
    float theta;        // PLL 估算的电角度 (rad)
    float omega;        // PLL 估算的电角速度 (rad/s)
    float pll_integral; // PLL 积分项
    float pll_err_prev; // PLL 上一次误差

    // ====== 时间戳 ======
    float Ts;           // 采样时间 (s)
    uint64_t timestamp; // 上次更新时间 (us)
} smo_observer_t;

/**
 * 初始化 SMO 观测器
 * @param smo   观测器指针
 * @param Rs    定子电阻 (Ω)
 * @param Ls    定子电感 (H)
 * @param Ke    反电动势常数 (V·s/rad)
 * @param K_smo 滑模增益 (建议 10~50，越大收敛越快但抖振越大)
 * @param Ts    控制周期 (s)
 */
void smo_init(smo_observer_t *smo, float Rs, float Ls, float Ke,
              float K_smo, float Ts);

/**
 * SMO 观测器单步更新
 * @param smo     观测器指针
 * @param Ualpha  α 轴电压 (V)
 * @param Ubeta   β 轴电压 (V)
 * @param Ialpha  α 轴电流 (A) - 实际测量值
 * @param Ibeta   β 轴电流 (A) - 实际测量值
 * 
 * @note 必须在每个控制周期调用 (通常 1kHz)
 *       内部会更新 smo->theta (电角度) 和 smo->omega (电角速度)
 */
void smo_update(smo_observer_t *smo, float Ualpha, float Ubeta,
                float Ialpha, float Ibeta);

/**
 * 重置 SMO 观测器 (用于模式切换时)
 */
void smo_reset(smo_observer_t *smo);

#endif // SMO_OBSERVER_H


