#ifndef _MAX30102_CONFIG_H_
#define _MAX30102_CONFIG_H_

#include "max30102_reg_defs.h"

#define MAX30102_I2C_CONFIG_DEFAULT  (i2c_config_t){\
        .mode = I2C_MODE_MASTER,                    \
        .sda_io_num = I2C_MASTER_SDA_IO_NUM,        \
        .scl_io_num = I2C_MASTER_SCL_IO_NUM,        \
        .sda_pullup_en = GPIO_PULLUP_DISABLE,       \
        .scl_pullup_en = GPIO_PULLUP_DISABLE,       \
        .master.clk_speed = I2C_MASTER_FREQ,        \
        .clk_flags = 0                              \
    }


#define MAX30102_REGISTERS_CONFIG_DEFAULT  (max30102_registers_t){\
        .intr_enable_1_reg.a_full_en_bit = 1,                     \
        .intr_enable_1_reg.ppg_rdy_en_bit = 1,                    \
        .fifo_config_reg.smp_ave_bits = 0b010,                    \
        .fifo_config_reg.fifo_rollover_en_bit = 1,                \
        .mode_config_reg.mode_bits = 0b011,                       \
        .spo2_config_reg.spo2_adc_rge_bits = 0b01,                \
        .spo2_config_reg.spo2_sr_bits = 0b001,                    \
        .spo2_config_reg.led_pw_bits = 0b10,                      \
        .led1_pa_reg.led1_pa_bits = 0x24,                         \
        .led2_pa_reg.led2_pa_bits = 0x24,                         \
        .prox_mode_led_reg.pilot_pa_bits = 0x7F,                  \
        .prox_intr_thresh_reg.prox_intr_thresh_bits = 0x0F,       \
        .die_temp_config_reg.temp_en_bit = 1                      \
    }

#define MAX30102_DEVICE_CONFIG_DEFAULT    (max30102_device_t){\
        .max30102_i2c_master_port = I2C_MASTER_PORT,                 \
        .max30102_i2c_cfg = MAX30102_I2C_CONFIG_DEFAULT,             \
        .registers = MAX30102_REGISTERS_CONFIG_DEFAULT,              \
        .dev_status = UNINITIALIZED                                  \
    }

#endif /*_MAX30102_CONFIG_H_*/