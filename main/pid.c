#include "pid.h"
#include <esp_timer.h>   // 获取微秒时间: esp_timer_get_time()

// pid_init: 保存 P/I/D/ramp/limit，重置内部状态，记录当前时间戳
void pid_init(pid_controller_t *pid, float P, float I, float D, float ramp, float limit){
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = esp_timer_get_time();

}
// pid_calculate:
float pid_calculate(pid_controller_t *pid, float error){

    // 1. 计算时间差 dt = (当前时间 - 上次时间) / 1000000.0 (转为秒)
    uint64_t timestamp_now = esp_timer_get_time();
    float dt = (timestamp_now - pid->timestamp_prev) / 1000000.0f;
    if(dt <= 0.0f || dt > 0.5f) dt = 1e-3f;

    // 2. 比例项 = P * error
    float proportional = pid->P * error;

    // 3. 微分项 = D * (error - error_prev) / dt
    float derivative = (dt > 0) ? pid->D * (error - pid->error_prev) / dt : 0.0f;

    // 4. 先计算不含积分项的临时输出，用于判断饱和方向
    float output_no_i = proportional + derivative;

    // 5. 积分项累加: integral += I * dt * (error + error_prev) / 2 (梯形积分)
    float integral = pid->integral_prev + pid->I * dt * (error + pid->error_prev) / 2.0f;
    
    // 6. 初步的完整输出
    float output = output_no_i + integral;

    // 7. 限幅输出到 ±limit
    float output_limited = (output > pid->limit) ? pid->limit : (output < -pid->limit) ? -pid->limit : output;

    // 8. 斜坡限幅: 如果 output_ramp > 0，限制输出变化率
    if (pid->output_ramp > 0) {
        float max_delta = pid->output_ramp * dt;
        float delta = output_limited - pid->output_prev;
        if (delta > max_delta) output_limited = pid->output_prev + max_delta;
        else if (delta < -max_delta) output_limited = pid->output_prev - max_delta;
    }

    // 9. 抗积分饱和 (Anti-windup): 如果输出饱和了，反向调整积分项
    if (output != output_limited) {
        // 输出被限幅了，积分项 = 限幅后的输出 - 比例项 - 微分项
        integral = output_limited - output_no_i;
    }
    // 限幅积分项到 ±limit 防止溢出
    integral = (integral > pid->limit) ? pid->limit : (integral < -pid->limit) ? -pid->limit : integral;

    // 10. 更新 error_prev, output_prev, integral_prev, timestamp_prev
    pid->error_prev = error;
    pid->output_prev = output_limited;
    pid->integral_prev = integral;
    pid->timestamp_prev = timestamp_now;

    // 11. 返回经过限幅和斜坡处理后的输出
    return output_limited;
}

