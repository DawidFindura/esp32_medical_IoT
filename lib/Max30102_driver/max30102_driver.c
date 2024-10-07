#include <esp_log.h>
#include <esp_err.h>
#include <string.h>

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

    error = max30102_get_register((max30102_generic_register_t *) &(dev->registers.fifo_wr_ptr_reg), 1);
    
    if(error == ESP_OK) {
        *reg_val = dev->registers.fifo_wr_ptr_reg.fifo_wr_ptr_bits;
    }

    return error;  
}

static esp_err_t max30102_get_fifo_rd_ptr(max30102_device_t *dev, int8_t *reg_val)
{   
    esp_err_t error = ESP_OK;

    error = max30102_get_register((max30102_generic_register_t *) &(dev->registers.fifo_rd_ptr_reg), 1);
    
    if(error == ESP_OK) {
        *reg_val = dev->registers.fifo_rd_ptr_reg.fifo_rd_ptr_bits;
    }

    return error;  
}

static esp_err_t adc_shift_from_pwm( max30102_device_t * dev, uint8_t * adc_shift ) 
{
    esp_err_t esp_err = ESP_FAIL;
    
    uint8_t shift = 0;
    max30102_ledpwm_t pwm = dev->led_pwm;

    switch(pwm) {
        case MAX31_LED_PWM_69:
        /** adc resolution = 15 bits **/
            shift = 3;
            esp_err = ESP_OK;
            break;
        case MAX31_LED_PWM_118:
        /** adc resolution 16 bits **/
            shift = 2;
            esp_err = ESP_OK;
            break;
        case MAX31_LED_PWM_215:
        /** adc res 17 bits **/
            shift = 1;
            esp_err = ESP_OK;
            break;
        case MAX31_LED_PWM_411:
        /** adc res 18 bits **/
            shift = 0;
            esp_err = ESP_OK;
            break;
        default:
            ESP_LOGE( TAG, "Error - invalid ledpwm set");
            esp_err = ESP_FAIL;
            break;
    }

    if( NULL != adc_shift )
    {
        *adc_shift = shift;
    }
    else
    {
        esp_err = ESP_ERR_INVALID_ARG;
    }

    return esp_err;
}

static esp_err_t conversion_factor_from_adcrange( max30102_device_t * dev, float * conversion_factor )
{
    esp_err_t esp_err = ESP_FAIL;

    float factor = 0;
    max30102_adcrange_t adc_range = dev->adc_range;

    switch (adc_range)
    {
        case MAX31_ADC_RNG_2048:
            factor = 7.81;
            esp_err = ESP_OK;
            break;
        case MAX31_ADC_RNG_4096:
            factor = 15.63;
            esp_err = ESP_OK;
            break;
        case MAX31_ADC_RNG_8192:
            factor = 31.25;
            esp_err = ESP_OK;
            break;
        case MAX31_ADC_RNG_16384:
            factor = 62.5;
            esp_err = ESP_OK;
            break;
        default:
            ESP_LOGE(TAG, "Error - invalid adcrange set");
            esp_err = ESP_FAIL;
            break;
    }

    if( NULL != conversion_factor )
    {
        *conversion_factor = factor;
    }
    else
    {
        esp_err = ESP_ERR_INVALID_ARG;
    }
    
    return esp_err;
}

static esp_err_t max30102_get_fifo_sample(max30102_device_t *dev, uint32_t *red_data, uint32_t *ir_data)
{
    esp_err_t error = ESP_OK;
    
    uint8_t data_temp[6];
    *red_data = 0;
    *ir_data = 0;

    float conversion_factor = 0.0f;
    uint8_t adc_shift = 0;

    error = conversion_factor_from_adcrange( dev, &conversion_factor );
    if( ESP_OK != error )
    {
        return error;
    }

    error = adc_shift_from_pwm( dev, &adc_shift );
    if( ESP_OK != error )
    {
        return error;
    }

    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = dev->registers.fifo_data_reg.reg_addr;

    error = i2c_device_register_read(dev_addr, reg_addr, &data_temp[0], 6);
     
    if(error == ESP_OK) {
        *red_data = ( data_temp[0] << 16 ) | ( data_temp[1] << 8 ) | ( data_temp[2] );
        *red_data = *red_data >> adc_shift;

        *ir_data = ( data_temp[3] << 16 ) | ( data_temp[4] << 8 ) | ( data_temp[5] );
        *ir_data = *ir_data >> adc_shift;
    }
    

    return error;
}

static inline void max30102_reg_addr_init(max30102_generic_register_t *start_reg)
{  
    for(int i = 0; i < MAX30102_NUM_OF_REGISTERS; i++){
        start_reg[i].reg_addr = max30102_register_addresses[i];
    }
}



/* Public functions definitions */

esp_err_t max30102_default_config_init(max30102_device_t * max30102_device )
{   
    max30102_device->max30102_i2c_master_port = I2C_MASTER_PORT;

    max30102_device->max30102_i2c_cfg.mode              = I2C_MODE_MASTER;
    max30102_device->max30102_i2c_cfg.sda_io_num        = I2C_MASTER_SDA_IO_NUM;
    max30102_device->max30102_i2c_cfg.scl_io_num        = I2C_MASTER_SCL_IO_NUM;   
    max30102_device->max30102_i2c_cfg.sda_pullup_en     = GPIO_PULLUP_DISABLE;     
    max30102_device->max30102_i2c_cfg.scl_pullup_en     = GPIO_PULLUP_DISABLE;     
    max30102_device->max30102_i2c_cfg.master.clk_speed  = I2C_MASTER_FREQ;
    max30102_device->max30102_i2c_cfg.clk_flags         = 0;    // default clock configuration for demanded i2c communication frequency


    max30102_device->registers.intr_enable_1_reg.a_full_en_bit            = 1;      // enable fifo almost full interrupt              
    max30102_device->registers.intr_enable_1_reg.ppg_rdy_en_bit           = 0;      // disable interrupt on every new data sample

    max30102_device->registers.intr_enable_2_reg.die_temp_rdy_en_bit      = 1;      // enable temperature sample ready interrupt 

    max30102_device->registers.fifo_config_reg.smp_ave_bits               = MAX31_SAMPLE_AVG_32;  // 16 samples averaging
    max30102_device->sample_avg = MAX31_SAMPLE_AVG_32;

    max30102_device->registers.fifo_config_reg.fifo_rollover_en_bit       = 1;
    max30102_device->registers.fifo_config_reg.fifo_a_full_bits           = 0xF;    // trigger interrupt on free space for 15 samples left

    max30102_device->registers.mode_config_reg.mode_bits                  = 0b011;  // SpO2 mode (RED LED and IR LED )
    max30102_device->mode = MAX31_MODE_SPO2_RED_IR;

    max30102_device->registers.spo2_config_reg.spo2_adc_rge_bits          = MAX31_ADC_RNG_4096;   // 4096 ADC range
    max30102_device->adc_range = MAX31_ADC_RNG_4096;

    max30102_device->registers.spo2_config_reg.spo2_sr_bits               = MAX31_SAMPLERATE_800;  // 00 samples per second
    max30102_device->samplerate = MAX31_SAMPLERATE_800;

    max30102_device->registers.spo2_config_reg.led_pw_bits                = MAX31_LED_PWM_411;   // led pulse width 411 us (18 bit ADC resolution)
    max30102_device->led_pwm  = MAX31_LED_PWM_411;

    max30102_device->registers.led1_pa_reg.led1_pa_bits                   = 0x3F;                         
    max30102_device->registers.led2_pa_reg.led2_pa_bits                   = 0x3F;

    //max30102_device->registers.prox_mode_led_reg.pilot_pa_bits            = 0x7F;                  
    //max30102_device->registers.prox_intr_thresh_reg.prox_intr_thresh_bits = 0xFF;           
    
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

esp_err_t max30102_get_register(max30102_generic_register_t *reg, uint8_t data_len)
{
    esp_err_t error = ESP_OK;
    uint8_t dev_addr = MAX30102_I2C_ADDRESS;
    uint8_t reg_addr = reg->reg_addr;
    uint8_t *reg_value = &(reg->reg_val);
    error = i2c_device_register_read(dev_addr, reg_addr, reg_value, data_len);
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
    error = max30102_get_register(reg, 1);

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

esp_err_t max30102_fifo_data_read(max30102_device_t *dev, uint8_t * numOfSamples )
{
    esp_err_t error = ESP_OK;

    int8_t fifo_wr_ptr = 0;
    int8_t fifo_rd_ptr = 0;
    
    memset( &dev->red_led_buffer, 0, sizeof(dev->red_led_buffer) );
    memset( &dev->ir_led_buffer, 0, sizeof(dev->ir_led_buffer) );

    max30102_get_fifo_wr_ptr(dev, &fifo_wr_ptr);
    max30102_get_fifo_rd_ptr(dev, &fifo_rd_ptr);
    
    uint8_t num_samples_to_read = (fifo_wr_ptr > fifo_rd_ptr) ? (fifo_wr_ptr - fifo_rd_ptr) : (fifo_rd_ptr - fifo_wr_ptr);
    #ifdef DEBUG
        //printf(" Num samples to read %d", num_samples_to_read);
    #endif
    
    for (int i = 0; i < num_samples_to_read; i++)
    {
        error = max30102_get_fifo_sample(dev, &dev->red_led_buffer[i], &dev->ir_led_buffer[i]);
        if( ESP_OK != error )
        {
            return error;
        }
    }

    if( NULL != numOfSamples )
    {
        *numOfSamples = num_samples_to_read;
    }
    else
    {
        error = ESP_ERR_INVALID_ARG;
    }

    return error;
}

esp_err_t max30102_get_die_temp( max30102_device_t * dev )
{
    uint8_t temp[2];
    float temp_frac = 0.0;
    int temp_int = 0;

    esp_err_t ret = max30102_get_register( (max30102_generic_register_t*) &(dev->registers.intr_status_2_reg), 1 );
    if( ret != ESP_OK )
    {
        return ret;
    }

   
    ret = i2c_device_register_read( MAX30102_I2C_ADDRESS, dev->registers.die_temp_int_reg.reg_addr, temp, 2 );
    if( ESP_OK == ret )
    {
        temp_int = (int8_t)temp[0];
        temp_frac = ((float) temp[1]) * 0.0625;
        dev->die_temp_buff = (int32_t)(temp_frac + ((float)temp_int));
    }
        
    #ifdef DEBUG
        //printf( "Die temp: %f\n", dev->die_temp_buff );
    #endif // DEBUG
    
    return ret;
}
