#ifndef _MAX30102_DRIVER_H_
#define _MAX30102_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "max30102_reg_defs.h"
#include "i2c_api.h"

#define MAX30102_NUM_OF_REGISTERS         (22U)
#define MAX30102_LEDS_BUFFER_SIZE         (50U)

typedef enum {
    UNINITIALIZED = 0,
    INITIALIZED = 1
} max30102_status_t;

typedef struct 
{

    i2c_port_t max30102_i2c_master_port;
    i2c_config_t max30102_i2c_cfg;
    max30102_registers_t registers;
    max30102_status_t dev_status;
    uint32_t red_led_buffer[ MAX30102_LEDS_BUFFER_SIZE ];
    uint32_t ir_led_buffer[ MAX30102_LEDS_BUFFER_SIZE ];
    float die_temp_buff;

} max30102_device_t;


esp_err_t max30102_default_config_init(max30102_device_t *max30102_device );
esp_err_t max30102_device_init(max30102_device_t *max30102_device_handle);
esp_err_t max30102_device_turn_on(max30102_device_t *dev);
esp_err_t max30102_device_turn_off(max30102_device_t *dev);
esp_err_t max30102_device_reset(max30102_device_t *dev);


esp_err_t max30102_set_register(max30102_generic_register_t *reg);
esp_err_t max30102_set_registers_burst(max30102_registers_t *reg);
esp_err_t max30102_clear_register(max30102_generic_register_t *reg);

esp_err_t max30102_get_register(max30102_generic_register_t *reg);
esp_err_t max30102_get_intr_status_reg_1(max30102_device_t *dev, int8_t *reg_val);

esp_err_t max30102_start_data_acquisition(max30102_device_t *dev);
esp_err_t max30102_fifo_data_read(max30102_device_t *dev);
esp_err_t max30102_get_die_temp( max30102_device_t * dev );


#ifdef __cplusplus
}
#endif

#endif /*_MAX30102_DRIVER_H_*/