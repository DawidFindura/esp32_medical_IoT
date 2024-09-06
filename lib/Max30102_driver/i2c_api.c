#include "i2c_api.h"
#include "esp_log.h"

static const char *TAG = "I2C API TEST";

esp_err_t i2c_master_init(i2c_port_t i2c_master_port, i2c_config_t *i2c_master_cfg)
{
    //Configure I2C bus with the given configuration
    esp_err_t error = ESP_OK;
    error = i2c_param_config(i2c_master_port, i2c_master_cfg);

    //Check valid parameter configuration
    if(error != ESP_OK){
        ESP_LOGE(TAG, "I2C parameter configuration failed!");
        return error;
    }
    
    //Install driver and enable I2C master peripheral
    error = i2c_driver_install(i2c_master_port, i2c_master_cfg->mode, 0, 0, 0);

    if(error != ESP_OK)
        ESP_LOGE(TAG, "I2C driver install failed!");

    return error;
}

esp_err_t i2c_device_register_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data_buf, size_t data_len)
{
    esp_err_t err = ESP_OK;
    uint8_t write_dev_addr = (dev_addr << 1) | I2C_MASTER_WRITE;

    //Create handle to commands queue
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    //Generate START condition
    err = i2c_master_start(cmd_handle);

    //Send write address of the device
    err = i2c_master_write_byte(cmd_handle, write_dev_addr, ACK_CHECK_EN);

     //Send device register address to wrtie
    err = i2c_master_write_byte(cmd_handle, reg_addr, ACK_CHECK_EN);

    //Send data to write  
    err = i2c_master_write(cmd_handle, data_buf, data_len, ACK_CHECK_EN);

    //Generate STOP condition
    i2c_master_stop(cmd_handle);

    //Execute queued commands in cmd_handle link
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_handle, pdMS_TO_TICKS(0));

    //Free resources
    i2c_cmd_link_delete(cmd_handle);

    return err;
}


esp_err_t i2c_device_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data_buf, size_t data_len)
{
    esp_err_t err = ESP_OK;
    uint8_t write_dev_addr = (dev_addr << 1) | I2C_MASTER_WRITE;
    uint8_t read_dev_addr = (dev_addr << 1) | I2C_MASTER_READ;
    
    //Create handle to commands queue
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    //Generate START condition
    err = i2c_master_start(cmd_handle);

    //Send device address + write mode
    err = i2c_master_write_byte(cmd_handle, write_dev_addr, ACK_CHECK_EN);

     //Send device register address to write
    err = i2c_master_write_byte(cmd_handle, reg_addr, ACK_CHECK_EN);

    //REPEATED_START;
    err = i2c_master_start(cmd_handle);

    //Send device address + read mode
    err = i2c_master_write_byte(cmd_handle, read_dev_addr, ACK_CHECK_EN);

    if(data_len > 1){
        i2c_master_read(cmd_handle, data_buf, (data_len - 1), ACK_SIGNAL);
    }

    i2c_master_read_byte(cmd_handle, (data_buf + data_len - 1), NACK_SIGNAL);

    //STOP;
    err = i2c_master_stop(cmd_handle);

    //Execute queued commands in cmd_handle link
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd_handle, pdMS_TO_TICKS(100));

    //Free resources
    i2c_cmd_link_delete(cmd_handle);

    return err;

}
