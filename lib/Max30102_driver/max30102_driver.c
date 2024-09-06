#include <esp_log.h>
#include <esp_err.h>

#include "i2c_api.h"
#include "max30102_driver.h"

#define DEBUG

/* array of max30102 register addresses */

static const uint8_t max30102_register_addresses[MAX30102_NUM_OF_REGISTERS] = {
    MAX30102_REG_ADDR_INTERRUPT_STATUS_1,
    MAX30102_REG_ADDR_INTERRUPT_STATUS_2,
    MAX30102_REG_ADDR_INTERRUPT_ENABLE_1,
    MAX30102_REG_ADDR_INTERRUPT_ENABLE_2,
    MAX30102_REG_ADDR_FIFO_WR_PTR,
    MAX30102_REG_ADDR_OVF_COUNTER,
    MAX30102_REG_ADDR_FIFO_RD_PTR,
    MAX30102_REG_ADDR_FIFO_DATA,
    MAX30102_REG_ADDR_FIFO_CONFIG,                    
    MAX30102_REG_ADDR_MODE_CONFIG,                  
    MAX30102_REG_ADDR_SPO2_CONFIG,                      
    MAX30102_REG_ADDR_LED1_PA,                         
    MAX30102_REG_ADDR_LED2_PA,                         
    MAX30102_REG_ADDR_PROX_MODE_LED_PA,         
    MAX30102_REG_ADDR_MULTI_LED_MODE_CNTRL_1,   
    MAX30102_REG_ADDR_MULTI_LED_MODE_CNTRL_2,
    MAX30102_REG_ADDR_DIE_TEMP_INT,
    MAX30102_REG_ADDR_DIE_TEMP_FRAC,
    MAX30102_REG_ADDR_DIE_TEMP_CONFIG,
    MAX30102_REG_ADDR_PROX_INT_THRESH,
    MAX30102_REG_ADDR_REVISION_ID,
    MAX30102_REG_ADDR_PART_ID   
};


static const char *TAG = "max30102_device_test";


/* Private functions definitions */

static esp_err_t max30102_get_fifo_wr_ptr(max30102_device_t *dev, int8_t *reg_val)
{   
    esp_err_t error = ESP_OK;

    error = max30102_get_register((max30102_generic_register_t *) &(dev->registers.fifo_wr_ptr_reg));
    
    if(error == ESP_OK) {
        *reg_val = dev->registers.fifo_wr_ptr_reg.fifo_wr_ptr_bits;
    }

    return error;  
}

static esp_err_t max30102_get_fifo_rd_ptr(max30102_device_t *dev, int8_t *reg_val)
{   
    esp_err_t error = ESP_OK;

    error = max30102_get_register((max30102_generic_register_t *) &(dev->registers.fifo_rd_ptr_reg));
    
    if(error == ESP_OK) {
        *reg_val = dev->registers.fifo_rd_ptr_reg.fifo_rd_ptr_bits;
    }

    return error;  
}

static esp_err_t max30102_get_fifo_sample(max30102_device_t *dev, uint32_t *red_data, uint32_t *ir_data)
{
    esp_err_t error = ESP_OK;
    uint8_t data_temp[6];
    *red_data = 0;
    *ir_data = 0;
   
    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = dev->registers.fifo_data_reg.reg_addr;

    error = i2c_device_register_read(dev_addr, reg_addr, &data_temp[0], 6);
     
    if(error == ESP_OK) {
        *red_data += data_temp[0] << 16;
        *red_data += data_temp[1] << 8;
        *red_data += data_temp[2];

        *ir_data += data_temp[3] << 16;
        *ir_data += data_temp[4] << 8;
        *ir_data += data_temp[5];
    }
    

    return error;
}

static inline void max30102_reg_addr_init(max30102_generic_register_t *start_reg)
{  
    for(int i = 0; i < MAX30102_NUM_OF_REGISTERS; i++){
        start_reg[i].reg_addr = max30102_register_addresses[i];
        ESP_LOGI(TAG, "\r\n%x \t %x", start_reg[i].reg_addr, max30102_register_addresses[i]);
    }
}


/* Public functions definitions */

esp_err_t max30102_default_config_init(max30102_device_t * max30102_device )
{   
    max30102_device->max30102_i2c_master_port = I2C_MASTER_PORT;

    max30102_device->max30102_i2c_cfg.mode = I2C_MODE_MASTER;
    max30102_device->max30102_i2c_cfg.sda_io_num = I2C_MASTER_SDA_IO_NUM;
    max30102_device->max30102_i2c_cfg.scl_io_num = I2C_MASTER_SCL_IO_NUM;   
    max30102_device->max30102_i2c_cfg.sda_pullup_en = GPIO_PULLUP_DISABLE;     
    max30102_device->max30102_i2c_cfg.scl_pullup_en = GPIO_PULLUP_DISABLE;     
    max30102_device->max30102_i2c_cfg.master.clk_speed = I2C_MASTER_FREQ;
    max30102_device->max30102_i2c_cfg.clk_flags = 0;

    max30102_device->registers.intr_enable_1_reg.a_full_en_bit = 1;                   
    max30102_device->registers.intr_enable_1_reg.ppg_rdy_en_bit = 1;  

    max30102_device->registers.intr_enable_2_reg.die_temp_rdy_en_bit = 1;

    max30102_device->registers.fifo_config_reg.smp_ave_bits = 0b010;       // 4 samples             
    max30102_device->registers.fifo_config_reg.fifo_rollover_en_bit = 1;
    max30102_device->registers.fifo_config_reg.fifo_a_full_bits = 0xF;    // free space for 15 samples

    max30102_device->registers.mode_config_reg.mode_bits = 0b011;        // SpO2 mode (RED LED and IR LED )

    max30102_device->registers.spo2_config_reg.spo2_adc_rge_bits = 0b01;                
    max30102_device->registers.spo2_config_reg.spo2_sr_bits = 0b000;                    
    max30102_device->registers.spo2_config_reg.led_pw_bits = 0b10;                      
    max30102_device->registers.led1_pa_reg.led1_pa_bits = 0x24;                         
    max30102_device->registers.led2_pa_reg.led2_pa_bits = 0x24;                         
    max30102_device->registers.prox_mode_led_reg.pilot_pa_bits = 0x7F;                  
    max30102_device->registers.prox_intr_thresh_reg.prox_intr_thresh_bits = 0x0F;           
    
    max30102_device->dev_status = UNINITIALIZED;

    return ESP_OK;    
}

esp_err_t max30102_device_init(max30102_device_t *max30102_device)
{   
    esp_err_t error = ESP_OK;

    /************I2C PERIPHERAL INITIALIZATION************/
    //Configuration parameters of I2C master peripheral
    i2c_config_t max30102_i2c_cfg = max30102_device->max30102_i2c_cfg;

    //ESP32 I2C master port to communicate with MAX30102 device
    i2c_port_t max30102_i2c_master_port = max30102_device->max30102_i2c_master_port;

    //ESP32 I2C peripheral initialization
    i2c_master_init(max30102_i2c_master_port, &max30102_i2c_cfg);
    
    if(error != ESP_OK){
        ESP_LOGE(TAG, "I2C master initialization failed");
        return error;
    }

    /**********MAX30102 REGISTERS INITIALIZATION**********/
    //Address initialization of MAX30102 registers
    max30102_generic_register_t *max30102_first_reg_ptr = (max30102_generic_register_t *) &(max30102_device->registers);
    max30102_reg_addr_init(max30102_first_reg_ptr);

    //Reset MAX30102 device to power-on state
    max30102_device_reset(max30102_device);

    //Setting configuration values of MAX30102 registers 
    error = max30102_set_registers_burst(&max30102_device->registers);

    if(error != ESP_OK){
        ESP_LOGE(TAG, "MAX30102 device register initialization failed");
        return error;
    }

    //Turn off MAX30102 to save power
    max30102_device_turn_off(max30102_device);

    max30102_device->dev_status = INITIALIZED;

    return error;
}

esp_err_t max30102_device_turn_on(max30102_device_t *dev)
{
    esp_err_t error = ESP_OK;

    dev->registers.mode_config_reg.shdn_bit = 0;
    error = max30102_set_register((max30102_generic_register_t *) &(dev->registers.mode_config_reg));

    return error;
}

esp_err_t max30102_device_turn_off(max30102_device_t *dev)
{
    esp_err_t error = ESP_OK;

    dev->registers.mode_config_reg.shdn_bit = 1;
    error = max30102_set_register((max30102_generic_register_t *) &(dev->registers.mode_config_reg));

    return error;
}

esp_err_t max30102_device_reset(max30102_device_t *dev)
{
    esp_err_t error = ESP_OK;

    dev->registers.mode_config_reg.reset_bit = 1;
    error = max30102_set_register((max30102_generic_register_t *) &(dev->registers.mode_config_reg));
    dev->registers.mode_config_reg.reset_bit = 0;

    return error;
}



esp_err_t max30102_set_register(max30102_generic_register_t *reg)
{   
    esp_err_t error = ESP_OK;
    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = reg->reg_addr;
    const uint8_t *reg_value = &(reg->reg_val); 
    error = i2c_device_register_write(dev_addr, reg_addr, reg_value, 1);
    return error;
}

esp_err_t max30102_clear_register(max30102_generic_register_t *reg)
{
    esp_err_t error = ESP_OK;
    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = reg->reg_addr;
    reg->reg_val = 0; 
    error = i2c_device_register_write(dev_addr, reg_addr, &reg->reg_val, 1);
    return error;
}

esp_err_t max30102_get_register(max30102_generic_register_t *reg)
{
    esp_err_t error = ESP_OK;
    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = reg->reg_addr;
    uint8_t *reg_value = &(reg->reg_val);
    error = i2c_device_register_read(dev_addr, reg_addr, reg_value, 1);
    return error; 
}

esp_err_t max30102_set_registers_burst(max30102_registers_t *reg)
{
    esp_err_t error = ESP_OK;
    max30102_generic_register_t *first_reg_to_write = (max30102_generic_register_t *) reg;

    for (int i = 2; i < MAX30102_NUM_OF_REGISTERS - 2; i++)
    {
        if(i == 7 || i ==16 || i == 17) {
            continue;
        }
        
        error = max30102_set_register(&first_reg_to_write[i]);
        
        if(error != ESP_OK) {
            break;
        }
    }

    return error;
}


esp_err_t max30102_get_intr_status_reg_1(max30102_device_t *dev, int8_t *reg_val)
{   
    esp_err_t error = ESP_OK;

    max30102_generic_register_t *reg = (max30102_generic_register_t *) &(dev->registers.intr_status_1_reg);
    error = max30102_get_register(reg);

    if(error == ESP_OK) {
        *reg_val = reg->reg_val;
    }
    
    return error;  
}


esp_err_t max30102_start_data_acquisition(max30102_device_t *dev)
{
    esp_err_t error = ESP_OK;

    error = max30102_clear_register((max30102_generic_register_t *) &(dev->registers.fifo_rd_ptr_reg));
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "clearing fifo rd ptr reg failed");
        return error;
    }

    error = max30102_clear_register((max30102_generic_register_t *) &(dev->registers.fifo_wr_ptr_reg));
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "clearing fifo wr ptr reg failed");
        return error;
    }

    error = max30102_clear_register((max30102_generic_register_t *) &(dev->registers.overflow_counter_reg));
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "clearing overflow counter  reg failed");
        return error;
    }

    error = max30102_device_turn_on(dev);
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "turning device on failed");
        return error;
    }

    return error;
}

esp_err_t max30102_fifo_data_read(max30102_device_t *dev)
{
    esp_err_t error = ESP_OK;

    int8_t fifo_wr_ptr = 0;
    int8_t fifo_rd_ptr = 0;
    
    max30102_get_fifo_wr_ptr(dev, &fifo_wr_ptr);
    max30102_get_fifo_rd_ptr(dev, &fifo_rd_ptr);
    
    uint8_t num_samples_to_read = (fifo_wr_ptr > fifo_rd_ptr) ? (fifo_wr_ptr - fifo_rd_ptr) : (fifo_rd_ptr - fifo_wr_ptr);
    #ifdef DEBUG
        printf(" Num samples to read %d", num_samples_to_read);
    #endif 
    
    for (int i = 0; i < num_samples_to_read; i++)
    {
        error = max30102_get_fifo_sample(dev, &dev->red_led_buffer[i], &dev->ir_led_buffer[i]);
        if( ESP_OK != error )
        {
            return error;
        }
        #ifdef DEBUG
            printf("\r\nred led: %ld\r\nir led: %ld\r\n",dev->red_led_buffer[i], dev->ir_led_buffer[i]);
        #endif 
    }

    return error;
}

esp_err_t max30102_get_die_temp( max30102_device_t * dev )
{
    uint8_t temp[2];
    float temp_frac = 0.0;
    int temp_int = 0;

    dev->registers.die_temp_config_reg.temp_en_bit = 1; 
    esp_err_t ret = max30102_set_register( (max30102_generic_register_t *) &(dev->registers.die_temp_config_reg) );
    if( ret != ESP_OK )
    {
        return ret;
    }

    ret = max30102_get_register( (max30102_generic_register_t*) &(dev->registers.intr_status_2_reg) );
    if( ret != ESP_OK )
    {
        return ret;
    }

    if(dev->registers.intr_status_2_reg.die_temp_rdy_bit) 
    {
        ret = i2c_device_register_read( MAX30102_I2C_ADDRESS, dev->registers.die_temp_int_reg.reg_addr, temp, 2 );
        if( ESP_OK == ret )
        {
            temp_int = (int8_t)temp[0];
            temp_frac = ((float) temp[1]) * 0.0625;
            dev->die_temp_buff = temp_frac + ((float)temp_int);
        }
        
    #ifdef DEBUG
        printf( "Die temp: %f\n", dev->die_temp_buff );
    #endif // DEBUG
    
    }

    return ret;
}
