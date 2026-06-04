#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include "ledc_pwm.h"
#include "foc_core.h"
#include "as5600.h"
#include "current_sense.h"
#include "pid.h"
#include "serial_cmd.h"

static const char *TAG = "foc_motor";

#define MOTOR_PP     7
#define VBUS        12.6f
#define SENSOR_DIR  -1

// PID参数
#define PID_P    5.0f
#define PID_I    200.0f
#define PID_D    0.0f
#define PID_RAMP 100000.0f
#define PID_LIMIT (VBUS / 2)

#define VEL_P     2.0f
#define VEL_I     10.0f
#define VEL_D     0.0f
#define VEL_RAMP  1000.0f
#define VEL_LIMIT 2.0f

static pid_controller_t g_pid_current;
static pid_controller_t g_pid_velocity;
static float g_zero_angle = 0.0f;

// 状态打印用的全局变量
static uint32_t g_loop_count = 0;
static float g_target_vel = 5.0f;
static float g_target_cur = 0.5f;
static float g_vel = 0, g_Iq = 0, g_Uq = 0;
static ctrl_mode_t g_ctrl_mode = CTRL_MODE_VELOCITY;

// ==================== 函数声明 ====================
static void foc_init_all(void);
static void foc_align_sensor(void);
static void foc_control_loop(void);

// ==================== 实现 ====================

static void foc_init_all(void)
{
    ledc_pwm_init();
    gpio_set_direction(12, GPIO_MODE_OUTPUT);
    gpio_set_level(12, 1);
    foc_core_init(VBUS);
    as5600_init();
    current_sense_init();
    pid_init(&g_pid_current, PID_P, PID_I, PID_D, PID_RAMP, PID_LIMIT);
    pid_init(&g_pid_velocity, VEL_P, VEL_I, VEL_D, VEL_RAMP, VEL_LIMIT);
    serial_cmd_init();
    ESP_LOGI(TAG, "All modules initialized");
}

static void foc_align_sensor(void)
{
    ESP_LOGI(TAG, "Aligning sensor...");
    foc_set_voltage(3.0f, 3 * PI / 2);
    vTaskDelay(pdMS_TO_TICKS(1000));
    g_zero_angle = as5600_read_angle() * SENSOR_DIR * MOTOR_PP;
    foc_set_voltage(0, 0);
    ESP_LOGI(TAG, "Zero angle = %.2f rad", g_zero_angle);
}

static void foc_control_loop(void)
{
    // ① 读编码器 → 电角度
    float mech_angle = as5600_read_angle();
    float elec_angle = mech_angle * SENSOR_DIR * MOTOR_PP - g_zero_angle;
    elec_angle = fmodf(elec_angle, 2 * PI);
    if (elec_angle < 0) elec_angle += 2 * PI;

    // ② 算速度
    g_vel = foc_calc_velocity(mech_angle);

    // ③ 根据控制模式执行不同的控制
    float Uq = 0;
    float Ia = 0, Ib = 0;

    switch (g_ctrl_mode) {
        case CTRL_MODE_OPENLOOP:
            // 开环: 用开环速度函数
            foc_openloop_velocity(g_target_vel);
            return;  // 开环已包含了输出，直接返回

        case CTRL_MODE_CURRENT:
            // 电流闭环: 只用电流PID
            Ia = current_sense_read_a();
            Ib = current_sense_read_b();
            g_Iq = foc_calc_iq(Ia, Ib, elec_angle);
            Uq = pid_calculate(&g_pid_current, g_target_cur - g_Iq);
            break;

        case CTRL_MODE_VELOCITY:
            // 速度闭环: 速度PID + 电流PID
            Ia = current_sense_read_a();
            Ib = current_sense_read_b();
            g_Iq = foc_calc_iq(Ia, Ib, elec_angle);
            float current_target = pid_calculate(&g_pid_velocity, g_target_vel - g_vel);
            Uq = pid_calculate(&g_pid_current, current_target - g_Iq);
            break;
    }

    // ④ 输出
    g_Uq = Uq;
    foc_set_voltage(Uq, elec_angle);

    // === ⑤ 状态打印 (VOFA+ FireWater 格式) ===
    if (g_loop_count++ % 10 == 0) {
        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
               g_vel,
               g_target_vel,
               g_Iq,
               g_Uq,
               elec_angle,
               Ia,
               Ib);
    }
}

void app_main(void)
{
    foc_init_all();
    foc_align_sensor();

    while (1) {
        // 处理串口命令
        serial_cmd_result_t cmd;
        if (serial_cmd_process(&cmd)) {
            if (cmd.stop) {
                g_target_vel = 0;
                g_target_cur = 0;
                g_ctrl_mode = cmd.control_mode;
                foc_set_voltage(0, 0);
                ESP_LOGI(TAG, "Motor stopped");
            } else {
                g_target_vel = cmd.target_velocity;
                g_target_cur = cmd.target_current;
                g_ctrl_mode = cmd.control_mode;
            }
        }

        foc_control_loop();
        vTaskDelay(pdMS_TO_TICKS(1));  // 至少让出CPU，最少1ms
    }
}