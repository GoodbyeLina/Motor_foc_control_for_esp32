// main/sensorless_startup.h
#ifndef SENSORLESS_STARTUP_H
#define SENSORLESS_STARTUP_H

/**
 * ==========================================================
 *  无感FOC三段式启动策略
 *  
 *  原理：
 *    零速时反电动势=0，SMO 无法工作。
 *    需要用 I-F（电流-频率）控制开环强推，等转速升上去后
 *    再平滑过渡到 SMO 闭环。
 *  
 *  三个阶段：
 *    ┌──────────────┬─────────────────────────────────────┐
 *    │ ① I-F强推     │ 开环合成角度，电流恒定，频率逐步升高  │
 *    │ ② 混合过渡    │ 角度 = (1-k)·θ_if + k·θ_smo       │
 *    │ ③ SMO闭环     │ 完全使用 SMO 估算的角度和速度        │
 *    └──────────────┴─────────────────────────────────────┘
 * ==========================================================
 */

// 启动阶段枚举
typedef enum {
    STARTUP_IF,         // ① I-F开环强推
    STARTUP_TRANSITION, // ② 混合过渡
    STARTUP_CLOSED      // ③ 纯SMO闭环
} startup_phase_t;

// 启动状态机结构体
typedef struct {
    // ── 当前状态 ──
    startup_phase_t phase;    // 当前阶段
    float theta_if;           // I-F合成角度 (电角度, rad)
    float omega_if;           // I-F角速度 (电角速度, rad/s)
    float I_if;               // I-F电流幅值 (A)

    // ── 可配置参数 ──
    float I_startup;          // 启动电流 (A)，建议 0.3~0.5
    float omega_trans_start;  // 开始过渡的电角速度 (rad/s)
    float omega_trans_end;    // 完成过渡的电角速度 (rad/s)

    // ── 输出 ──
    float theta_used;         // 最终用于 FOC 的电角度 (rad)
    float omega_used;         // 最终用于速度环的电角速度 (rad/s)
    float Iq_ref;             // q轴电流参考值 (A)
} sensorless_startup_t;

// ========== 接口函数 ==========

/**
 * 初始化启动状态机
 * @param su             状态机指针
 * @param I_startup      启动电流 (A)
 * @param omega_start    开始过渡的电角速度 (rad/s)
 * @param omega_end      完成过渡的电角速度 (rad/s)
 */
void startup_init(sensorless_startup_t *su, float I_startup,
                  float omega_start, float omega_end);

/**
 * 启动状态机单步运行 (每个控制周期调用一次)
 * @param su            状态机指针
 * @param smo_omega     SMO估算的电角速度 (rad/s)
 * @param smo_theta     SMO估算的电角度 (rad)
 * @param target_vel    目标机械角速度 (rad/s)
 * @param dt            控制周期 (s)
 * 
 * @note 调用后 su->theta_used / su->omega_used / su->Iq_ref 更新
 */
void startup_run(sensorless_startup_t *su, float smo_omega, float smo_theta,
                 float target_vel, float dt);

/**
 * 重置启动状态机 (回到 I-F 阶段)
 */
void startup_reset(sensorless_startup_t *su);

#endif // SENSORLESS_STARTUP_H