#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ledc_pwm.h"
#include "foc_core.h"
#include "as5600.h"
#include "current_sense.h"
#include "serial_cmd.h"
#include "motor_control.h"

static const char *TAG = "foc_motor";

#define VBUS  12.6f

/**
 * 初始化所有模块
 */
static void foc_init_all(void)
{
    // 1. 使能驱动板 (GPIO12)
    gpio_config_t en_io = {
        .pin_bit_mask = (1ULL << 12),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&en_io);
    gpio_set_level(12, 1);

    // 2. 初始化底层硬件
    ledc_pwm_init();
    foc_core_init(VBUS);
    as5600_init();
    current_sense_init();

    // 3. 初始化控制层
    motor_control_init();
    serial_cmd_init();

    ESP_LOGI(TAG, "All modules initialized");
}

void app_main(void)
{
    foc_init_all();
    motor_control_align();

    while (1) {
        // 处理串口命令
        serial_cmd_result_t cmd;
        if (serial_cmd_process(&cmd)) {
            if (cmd.stop) {
                motor_control_stop();
                motor_control_set_mode(cmd.control_mode);
                ESP_LOGI(TAG, "Motor stopped");
            } else {
                motor_control_set_target_velocity(cmd.target_velocity);
                motor_control_set_target_current(cmd.target_current);
                motor_control_set_mode(cmd.control_mode);
            }
        }

        motor_control_run();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}