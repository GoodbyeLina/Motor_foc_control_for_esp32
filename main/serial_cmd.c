#include "serial_cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "motor_test.h"

static const char *TAG = "serial_cmd";

#define UART_NUM        UART_NUM_0  // 使用默认串口 (与日志共用)
#define BUF_SIZE        256         // 接收缓冲区大小
#define RD_BUF_SIZE     64          // 单次读取大小

// 默认值
#define DEFAULT_VELOCITY  5.0f     // 默认目标速度 5 rad/s
#define DEFAULT_CURRENT   0.5f     // 默认目标电流 0.5A

// 初始命令结果
static serial_cmd_result_t g_default_result = {
    .parsed          = 0,
    .target_velocity = DEFAULT_VELOCITY,
    .target_current  = DEFAULT_CURRENT,
    .control_mode    = CTRL_MODE_VELOCITY,  // 默认速度闭环
    .stop            = 0,
};

void serial_cmd_init(void)
{
    // 配置UART
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Serial command system initialized");
    serial_print_help();
}

void serial_print_help(void)
{
    printf("\r\n");
    printf("========== DengFOC 串口命令系统 ==========\r\n");
    printf("  命令       | 说明                          \r\n");
    printf("------------+--------------------------------\r\n");
    printf("  help       | 显示本帮助信息                \r\n");
    printf("  v <num>    | 设置目标速度 (rad/s)          \r\n");
    printf("  c <num>    | 设置目标电流 (A)              \r\n");
    printf("  s          | 停止电机                      \r\n");
    printf("  mode <n>   | 切换模式: 0开环 1电流 2速度   \r\n");
    printf("  pid        | 查看电流环 PID 参数           \r\n");
    printf("  pid <p/i/d/ramp/limit> <val>  | 设置电流环 \r\n");
        printf("  vpid       | 查看速度环 PID 参数           \r\n");
    printf("  vpid <p/i/d/ramp/limit> <val> | 设置速度环 \r\n");
    printf("  test_cur   | 电流阶梯测试: c=0.05→0.50\r\n");
    printf("  test_vel   | 速度阶梯测试: v=10→70\r\n");
    printf("  test_step <v> | 速度阶跃测试\r\n");
    printf("  test_stop  | 停止当前测试\r\n");
    printf("  ---------- 例: test_cur 0.1 0.5 0.1 2\r\n");
    printf("              例: test_step 50\r\n");
    printf("============================================\r\n");
    printf("\r\n");
}

uint8_t serial_cmd_process(serial_cmd_result_t *result)
{
        // 先把结果设为默认（无新命令时保持原值）
    *result = g_default_result;

    // 每次处理命令前先清除 stop 标志，防止卡死
    g_default_result.stop = 0;

    // 从UART读取一行数据
    uint8_t data[RD_BUF_SIZE];
    int len = uart_read_bytes(UART_NUM, data, RD_BUF_SIZE - 1, 0);  // 非阻塞
    if (len <= 0) {
        return 0;  // 无新数据
    }

    // 以字符串结尾
    data[len] = '\0';

    // 去除换行符
    char *newline = strchr((char *)data, '\r');
    if (newline) *newline = '\0';
    newline = strchr((char *)data, '\n');
    if (newline) *newline = '\0';

    // 解析命令
    char cmd[32] = {0};
    float value = 0.0f;

    // 尝试解析 "v <num>" 或 "c <num>" 或 "mode <n>"
    int n = sscanf((char *)data, "%s %f", cmd, &value);

    // 回显输入
    printf(">>> %s", (char *)data);
    if (n >= 1) {
        printf("  (cmd=%s", cmd);
        if (n >= 2) printf(", val=%.3f", value);
        printf(")\r\n");
    }

        // 尝试解析三字段命令: "pid p 1.5" 或 "vpid i 50"
    char subcmd[32] = {0};
    float val2 = 0.0f;
    int n3 = sscanf((char *)data, "%s %s %f", cmd, subcmd, &val2);
    // 如果三字段解析失败，回退到两字段解析
    if (n3 < 2) {
        // 原始解析结果保留
    }

    // === 命令解析 ===
    if (strcmp(cmd, "help") == 0) {
        serial_print_help();
    }
    else if (strcmp(cmd, "v") == 0 && n >= 2) {
        g_default_result.target_velocity = value;
        g_default_result.parsed = 1;
        printf("  → 目标速度已设为 %.2f rad/s\r\n", value);
    }
    else if (strcmp(cmd, "c") == 0 && n >= 2) {
        g_default_result.target_current = value;
        g_default_result.parsed = 1;
        printf("  → 目标电流已设为 %.3f A\r\n", value);
    }
        else if (strcmp(cmd, "s") == 0) {
        // 手动停止时也终止测试
        if (motor_test_is_running()) {
            motor_test_stop();
        }
        g_default_result.stop = 1;
        g_default_result.parsed = 1;
        printf("  → 电机已停止\r\n");
    }
        else if (strcmp(cmd, "mode") == 0 && n >= 2) {
        // 手动切模式时终止测试
        if (motor_test_is_running()) {
            motor_test_stop();
        }
        int mode = (int)value;
        if (mode >= 0 && mode <= 2) {
            g_default_result.control_mode = (ctrl_mode_t)mode;
            g_default_result.parsed = 1;
            const char *mode_names[] = {"开环", "电流闭环", "速度闭环"};
            printf("  → 已切换到 %s 模式\r\n", mode_names[mode]);
        } else {
            printf("  → 错误: 模式必须为 0(开环) 1(电流) 2(速度)\r\n");
        }
    }
    // ====== 在线调参命令 ======
    else if (strcmp(cmd, "pid") == 0) {
        if (n3 == 1) {
            // pid → 显示参数
            float p, i, d, ramp, limit;
            motor_control_get_current_pid(&p, &i, &d, &ramp, &limit);
            printf("  电流环 PID: P=%.3f  I=%.3f  D=%.3f  ramp=%.0f  limit=%.3f\r\n",
                   p, i, d, ramp, limit);
        } else if (n3 == 3) {
            // pid p 1.5 → 设置
            if (strcmp(subcmd, "p") == 0) {
                motor_control_set_current_pid(val2, 0, 0, 0, 0);
            } else if (strcmp(subcmd, "i") == 0) {
                motor_control_set_current_pid(0, val2, 0, 0, 0);
            } else if (strcmp(subcmd, "d") == 0) {
                motor_control_set_current_pid(0, 0, val2, 0, 0);
            } else if (strcmp(subcmd, "ramp") == 0) {
                motor_control_set_current_pid(0, 0, 0, val2, 0);
            } else if (strcmp(subcmd, "limit") == 0) {
                motor_control_set_current_pid(0, 0, 0, 0, val2);
            } else {
                printf("  → 未知参数名 '%s'，可用: p i d ramp limit\r\n", subcmd);
            }
        }
        g_default_result.parsed = 1;
    }
    else if (strcmp(cmd, "vpid") == 0) {
        if (n3 == 1) {
            // vpid → 显示参数
            float p, i, d, ramp, limit;
            motor_control_get_velocity_pid(&p, &i, &d, &ramp, &limit);
            printf("  速度环 PID: P=%.3f  I=%.3f  D=%.3f  ramp=%.0f  limit=%.3f\r\n",
                   p, i, d, ramp, limit);
        } else if (n3 == 3) {
            if (strcmp(subcmd, "p") == 0) {
                motor_control_set_velocity_pid(val2, 0, 0, 0, 0);
            } else if (strcmp(subcmd, "i") == 0) {
                motor_control_set_velocity_pid(0, val2, 0, 0, 0);
            } else if (strcmp(subcmd, "d") == 0) {
                motor_control_set_velocity_pid(0, 0, val2, 0, 0);
            } else if (strcmp(subcmd, "ramp") == 0) {
                motor_control_set_velocity_pid(0, 0, 0, val2, 0);
            } else if (strcmp(subcmd, "limit") == 0) {
                motor_control_set_velocity_pid(0, 0, 0, 0, val2);
            } else {
                printf("  → 未知参数名 '%s'，可用: p i d ramp limit\r\n", subcmd);
            }
        }
        g_default_result.parsed = 1;
    }
        // ====== 测试命令 ======
    else if (strcmp(cmd, "test_cur") == 0) {
        float s = 0.05f, e = 0.50f, sp = 0.05f, st = 3.0f;
        int n4 = sscanf((char *)data, "%*s %f %f %f %f", &s, &e, &sp, &st);
        if (n4 >= 2) {
            // 用户提供了至少 start 和 end
            motor_test_current_ramp(s, e, sp, st);
        } else {
            // 默认参数: 0.05 → 0.50, step 0.05, 每档3秒
            motor_test_current_ramp(0.05f, 0.50f, 0.05f, 3.0f);
        }
        g_default_result.parsed = 0;  // 测试函数自己处理，不让主循环覆盖
    }
    else if (strcmp(cmd, "test_vel") == 0) {
        float s = 10.0f, e = 75.0f, sp = 10.0f, st = 5.0f;
        int n4 = sscanf((char *)data, "%*s %f %f %f %f", &s, &e, &sp, &st);
        if (n4 >= 2) {
            motor_test_velocity_ramp(s, e, sp, st);
        } else {
            motor_test_velocity_ramp(10.0f, 70.0f, 10.0f, 5.0f);
        }
        g_default_result.parsed = 0;
    }
    else if (strcmp(cmd, "test_step") == 0 && n3 >= 2) {
        float target;
        sscanf((char *)data, "%*s %f", &target);
        motor_test_step_response(target);
        g_default_result.parsed = 0;
    }
    else if (strcmp(cmd, "test_stop") == 0) {
        motor_test_stop();
        g_default_result.parsed = 0;
    }
    else if (strlen((char *)data) > 0) {
        printf("  → 未知命令，输入 help 查看帮助\r\n");
    }

    *result = g_default_result;
    return g_default_result.parsed;
}