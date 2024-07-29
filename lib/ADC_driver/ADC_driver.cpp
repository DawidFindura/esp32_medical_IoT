#include <esp_log.h>
#include <driver/gpio.h>

#include "ADC_driver.hpp"

#define CONVERSION_FRAME_SIZE   256 // bytes
#define MAX_STORE_BUF_SIZE      1024 // bytes

#define NUMBER_OF_ADC_CHANNELS  1
#define ADC_GPIO_PIN            GPIO_NUM_34
#define ADC_SAMPLE_FREQ_HZ      20000

static const char * TAG = " ADC_driver";

using namespace Driver;


// public function definitions

ADC_driver::ADC_driver() : m_adc_device()
{

}

ADC_driver::~ADC_driver()
{

}

execStatus ADC_driver::init()
{
    execStatus eStatus = execStatus::FAILURE;

    eStatus = createCaliScheme();
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Successfully created calibration scheme for ADC%d module", ( m_adc_device.adc_unit + 1 ) );
    }

    eStatus = adc_continuous_mode_drv_init();
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Successfull initialization of continuous mode ADC driver");
    }

    return eStatus;
}

execStatus ADC_driver::deinit()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    esp_err = adc_continuous_deinit( m_adc_device.adc_continuous_mode_drv_handle );
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
    }
    return eStatus;
}

execStatus ADC_driver::start()
{
    execStatus eStatus = execStatus::FAILURE;
    return eStatus;
}

execStatus ADC_driver::stop()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    esp_err = adc_continuous_stop( m_adc_device.adc_continuous_mode_drv_handle );
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
    }
    
    return eStatus;
}

execStatus ADC_driver::setBitwidth( adc_bitwidth_t a_adc_bitwidth )
{
    execStatus eStatus = execStatus::FAILURE;

    m_adc_device.adc_bitwidth = a_adc_bitwidth;
    if( m_adc_device.adc_bitwidth == a_adc_bitwidth )
    {
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}

execStatus ADC_driver::setAttenuation( adc_atten_t a_adc_atten )
{
    execStatus eStatus = execStatus::FAILURE;

    m_adc_device.adc_atten = a_adc_atten;
    if( m_adc_device.adc_atten == a_adc_atten )
    {
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}

execStatus ADC_driver::getBitwidth( adc_bitwidth_t & a_adc_bitwidth )
{
    execStatus eStatus = execStatus::SUCCESS;
    a_adc_bitwidth = m_adc_device.adc_bitwidth;
    return eStatus;
}

execStatus ADC_driver::getAttenuation( adc_atten_t & a_adc_atten )
{
    execStatus eStatus = execStatus::SUCCESS;
    a_adc_atten = m_adc_device.adc_atten;
    return eStatus;
}

execStatus ADC_driver::getCaliSchemeHandle( adc_cali_handle_t & a_adc_cali_scheme_handle )
{
    execStatus eStatus = execStatus::SUCCESS;
    a_adc_cali_scheme_handle = m_adc_device.adc_cali_scheme_handle;
    return eStatus;
}

// private function definitions

execStatus ADC_driver::createCaliScheme()
{
    esp_err_t esp_err = ESP_FAIL;
    execStatus eStatus = execStatus::FAILURE;

    adc_cali_scheme_ver_t cali_scheme;
    esp_err = adc_cali_check_scheme( &cali_scheme );
    if( ESP_OK != esp_err )
    {
        ESP_LOGE( TAG, "FAILED TO CHECK CALIBRATION SCHEME" );
        return eStatus;
    }

    if( cali_scheme & BIT(0) )
    {
        // line fitting scheme version supported
        ESP_LOGI( TAG, "Line fitting scheme version supported" );

        adc_cali_line_fitting_efuse_val_t efuseVal;
        adc_cali_scheme_line_fitting_check_efuse( &efuseVal );
        uint32_t VrefDefault = 0;

        switch( efuseVal )
        {
            case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_VREF:
            {
                ESP_LOGI( TAG, "Characterization based on reference voltage stored in eFuse" );
                VrefDefault = 0;
                break;
            }
            case ADC_CALI_LINE_FITTING_EFUSE_VAL_EFUSE_TP:
            {
                ESP_LOGI( TAG, "Characterization based on Two Point values stored in eFuse" );
                VrefDefault = 0;
                break;
            }
            case ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF:
            {
                ESP_LOGI( TAG, "Characterization based on default reference voltage" );
                VrefDefault = 1100; // mV
                break;
            }
        }

        adc_cali_line_fitting_config_t cali_config = 
        {
            .unit_id = m_adc_device.adc_unit,
            .atten = m_adc_device.adc_atten,
            .bitwidth = m_adc_device.adc_bitwidth,
            .default_vref = VrefDefault
        };
        
        esp_err = adc_cali_create_scheme_line_fitting( &cali_config, &m_adc_device.adc_cali_scheme_handle );
        if( ESP_OK == esp_err )
        {
            eStatus = execStatus::SUCCESS;
        }
    }

    return eStatus;
}

 execStatus ADC_driver::adc_continuous_mode_drv_init()
 {
    esp_err_t esp_err = ESP_FAIL;
    execStatus eStatus = execStatus::FAILURE;

    adc_continuous_handle_t adc_continuous_mode_drv_handle = NULL;

    adc_continuous_handle_cfg_t adc_continuous_init_cfg =
    {
        .max_store_buf_size = MAX_STORE_BUF_SIZE,
        .conv_frame_size = CONVERSION_FRAME_SIZE
    };

    esp_err = adc_continuous_new_handle( &adc_continuous_init_cfg, &adc_continuous_mode_drv_handle );
    if( ESP_OK != esp_err )
    {
        ESP_LOGE( TAG, "FAILED TO CREATE NEW ADC CONTINUOUS MODE DRIVER HANDLE" );
        return eStatus;
    }

    adc_channel_t adc_channel; 
    adc_unit_t adc_unit_id;
    esp_err = adc_continuous_io_to_channel( ADC_GPIO_PIN, &adc_unit_id, &adc_channel );
    if( ESP_OK == esp_err && adc_unit_id == m_adc_device.adc_unit )
    {
        m_adc_device.adc_channel = adc_channel;
    }

    adc_digi_pattern_config_t adc_digi_pattern_config =
    {
        .atten = m_adc_device.adc_atten,
        .channel = m_adc_device.adc_channel,
        .unit = m_adc_device.adc_unit,
        .bit_width = m_adc_device.adc_bitwidth
    };


    adc_continuous_config_t adc_continuous_mode_drv_cfg =
    {
        .pattern_num = NUMBER_OF_ADC_CHANNELS,
        .adc_pattern = &adc_digi_pattern_config,
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // ADC digital controller (DMA mode) work mode. Only use ADC1 for conversion
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1 //ADC digital controller (DMA mode) output data format.
    };
    
    if( NULL != adc_continuous_mode_drv_handle )
    {
        esp_err = adc_continuous_config( adc_continuous_mode_drv_handle, &adc_continuous_mode_drv_cfg );
        if( ESP_OK == esp_err )
        {
            eStatus = execStatus::SUCCESS;
            m_adc_device.adc_continuous_mode_drv_handle = adc_continuous_mode_drv_handle;
            ESP_LOGI( TAG, " Configuration of continuous mode ADC driver finished successfully" );
        }   
    }
    
    return eStatus;
 }

