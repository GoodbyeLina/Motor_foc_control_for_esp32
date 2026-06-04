#ifndef FOC_CORE_H
#define FOC_CORE_H

#define PI 3.14159265359f


/**
 * 初始化FOC核心
 * @param power_supply 电源电压 (V)，例程中是 12.6V
 */
void foc_core_init(float power_supply);

/**
 * SVPWM 输出：设置 q 轴电压和电角度，输出三相 PWM
 * @param Uq        q轴电压 (V)，控制电机力矩/速度，范围 -Vbus/√3 ~ +Vbus/√3
 * @param angle_el  电角度 (rad)
 * @note 内部使用 7 段式 SVPWM（零序分量注入，Vbus/√3 ≈ 0.577·Vbus），
 *       相比 SPWM（Vbus/2 = 0.5·Vbus）母线电压利用率提高 ~15.4%
 */
void foc_set_voltage(float Uq, float angle_el);

/**
 * 电流变换: Clarke + Park (Ia,Ib → Iq)
 * @param Ia        A相电流 (A)
 * @param Ib        B相电流 (A)
 * @param angle_el  电角度 (rad)
 * @return          Iq 电流值 (A)
 * @note Clark: Iα=Ia, Iβ=(Ia+2·Ib)/√3
 *       Park:  Iq = -Iα·sinθ + Iβ·cosθ
 */
float foc_calc_iq(float Ia, float Ib, float angle_el);

// foc_core.h 加声明
void foc_openloop_velocity(float target_velocity);

/**
 * 从编码器角度计算机械速度 (带低通滤波)
 * @param angle  当前机械角度 (rad)
 * @return       机械角速度 (rad/s)
 */
float foc_calc_velocity(float angle);

#endif