#ifndef I2C_API_H_
#define I2C_API_H_

#include <driver/i2c.h>

#include "esp_err.h"


#define I2C_MASTER_PORT               (I2C_NUM_0)

#define I2C_MASTER_SDA_IO_NUM         (GPIO_NUM_32)
#define I2C_MASTER_SCL_IO_NUM         (GPIO_NUM_33)

#define I2C_FREQ_100_KHZ              (100000U)
#define I2C_FREQ_400_KHZ              (400000U)
#define I2C_MASTER_FREQ               I2C_FREQ_400_KHZ

#define ACK_CHECK_DIS                 (0U)
#define ACK_CHECK_EN                  (1U)

#define ACK_SIGNAL                    (0U)
#define NACK_SIGNAL                   (1U)

#define MAX30102_I2C_ADDRESS          (0x57)

esp_err_t i2c_master_init(i2c_port_t i2c_master_port, i2c_config_t *i2c_master_cfg);
esp_err_t i2c_device_register_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data_buf, size_t data_len);
esp_err_t i2c_device_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_buf, size_t data_len);


#endif /*I2C_API_H_*/