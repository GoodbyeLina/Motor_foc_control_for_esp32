#include "as5600.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "as5600";

#define AS5600_ADDR         0x36  // AS5600 I2C设备地址
#define AS5600_ANGLE_REG    0x0C  // 角度寄存器地址

#define I2C_MASTER_NUM      I2C_NUM_0       // 使用I2C控制器0
#define I2C_MASTER_SDA_IO   19              // SDA引脚
#define I2C_MASTER_SCL_IO   18              // SCL引脚
#define I2C_MASTER_FREQ_HZ  400000          // 400kHz
#define I2C_MASTER_TX_BUF   0               // 不启用发送缓冲区
#define I2C_MASTER_RX_BUF   0               // 不启用接收缓冲区

// ========== 你的任务1：实现I2C初始化函数 ==========
void as5600_init(void)
{
    // TODO: 
    // 1. 配置 i2c_config_t 结构体
    i2c_config_t conf ={
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    // 2. 调用 i2c_param_config(I2C_MASTER_NUM, &conf)
    i2c_param_config(I2C_MASTER_NUM, &conf);

    // 3. 调用 i2c_driver_install(I2C_MASTER_NUM, ...)
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF, I2C_MASTER_TX_BUF, 0);

    // 4. 打印日志 "AS5600 initialized"
    ESP_LOGI(TAG, "AS5600 Initialized");
    
}


// ========== 你的任务2：实现角度读取函数 ==========
float as5600_read_angle(void)
{
    uint8_t data[2] = {0};
    uint16_t raw_angle = 0;

    // ---- 第1步：先写入寄存器地址 0x0C（告诉AS5600我要读哪个寄存器） ----
    i2c_cmd_handle_t cmd =i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AS5600_ANGLE_REG, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    // ---- 第2步：读取2个字节的角度数据 ----
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    // ---- 第3步：数据拼接 ----
    // AS5600 是12位数据，高位字节的低4位 + 低位字节的8位
    // 提示：AS5600 datasheet说，角度值 = (data[0] & 0x0F) << 8 | data[1]
    raw_angle = ((uint16_t)data[0] & 0x0F) << 8 | data[1];

    // ---- 第4步：转换为弧度 ----
    return (float)raw_angle / 4096.0f * 2 *M_PI; 

}
