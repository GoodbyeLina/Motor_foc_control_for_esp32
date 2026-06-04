#include "serial_cmd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    printf("  命令     | 说明                            \r\n");
    printf("----------+----------------------------------\r\n");
    printf("  help     | 显示本帮助信息                  \r\n");
    printf("  v <num>  | 设置目标速度 (rad/s)            \r\n");
    printf("           | 例: v 5     → 速度 5 rad/s      \r\n");
    printf("           | 例: v -3    → 反向 3 rad/s      \r\n");
    printf("  c <num>  | 设置目标电流 (A)                \r\n");
    printf("           | 例: c 0.5   → 电流 0.5A         \r\n");
    printf("  s        | 停止电机                        \r\n");
    printf("  mode <n> | 切换控制模式:                   \r\n");
    printf("           |   0=开环  1=电流  2=速度(默认)   \r\n");
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
            g_default_result.stop = 1;
            g_default_result.parsed = 1;
            printf("  → 电机已停止\r\n");
        }
        else if (strcmp(cmd, "mode") == 0 && n >= 2) {
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
    else if (strlen((char *)data) > 0) {
        printf("  → 未知命令，输入 help 查看帮助\r\n");
    }

    *result = g_default_result;
    return g_default_result.parsed;
}