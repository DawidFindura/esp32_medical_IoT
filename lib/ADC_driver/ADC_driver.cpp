#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "ADC_driver.hpp"

#define CONVERSION_FRAME_SIZE               256 // bytes
#define MAX_STORE_BUF_SIZE                  1024 // bytes

#define NUMBER_OF_ADC_CHANNELS              1
#define ADC_GPIO_PIN                        GPIO_NUM_34
#define ADC_SAMPLE_FREQ_HZ                  20000 // 20 kHz

#define ADC_DATA_READER_TASK_STACK_SIZE     5000 // bytes
#define ADC_DATA_READER_TASK_PRIORITY       5

#define ADC_DATA_PROC_TASK_STACK_SIZE       10000 // bytes
#define ADC_DATA_PROC_TASK_PRIORITY         ADC_DATA_READER_TASK_PRIORITY
#define ADC_DATA_PROC_BUFFER_SIZE           ()

#define ADC_DATA_RING_BUFFER_ITEM_SIZE      sizeof(int)
#define ADC_DATA_RING_BUFFER_MAX_ITEMS      3200


static const char * TAG = " ADC_driver";

using namespace Driver;


// public function definitions

ADC_driver::ADC_driver() :   
    m_adc_device(),
    m_adc_driver_state( eDriverState::UNINITIALIZED ), 
    m_adc_data_reader_task_handle( NULL ),
    m_adc_data_proc_task_handle( NULL ), 
    m_adc_data_ring_buff_handle( NULL ),
    m_adc_data_reader_semphr( NULL ),
    m_adc_data_proc_semphr( NULL ),
    m_data_logger()
{

}

ADC_driver::~ADC_driver()
{
    (void)this-> deinit();
}

execStatus ADC_driver::init()
{
    execStatus eStatus = execStatus::FAILURE;

    if( eDriverState::UNINITIALIZED != m_adc_driver_state && eDriverState::DEINITIALIZED != m_adc_driver_state )
    {
        ESP_LOGI( TAG, "Driver already initialized!!!" );
        return eStatus;
    }

    /* Creation of ring buffer for ADC result data */
    m_adc_data_ring_buff_handle =  xRingbufferCreateNoSplit( ADC_DATA_RING_BUFFER_ITEM_SIZE, ADC_DATA_RING_BUFFER_MAX_ITEMS );
    if( NULL == m_adc_data_ring_buff_handle )
    {
        ESP_LOGW( TAG, "Failed to create ring buffer" );
        return eStatus = execStatus::FAILURE;
    }

    /* Creation of task synchronizing semaphore */
    m_adc_data_reader_semphr = xSemaphoreCreateBinary();
    if( NULL == m_adc_data_reader_semphr )
    {
        ESP_LOGW( TAG, "Failed to create binary semaphore for data reader task" );
        return eStatus = execStatus::FAILURE;   
    }

    /* Creation of task synchronizing semaphore */
    m_adc_data_proc_semphr = xSemaphoreCreateBinary();
    if( NULL == m_adc_data_proc_semphr )
    {
        ESP_LOGW( TAG, "Failed to create binary semaphore for data processor task" );
        return eStatus = execStatus::FAILURE;   
    }

    /* ADC data reader task creation*/
    BaseType_t ret = xTaskCreate
                    (
                        adc_data_reader_task,
                        "ADC data reader task",
                        ADC_DATA_READER_TASK_STACK_SIZE,
                        (void *)this,
                        ADC_DATA_READER_TASK_PRIORITY,
                        &m_adc_data_reader_task_handle
                    );

    if( pdTRUE != ret )
    {
        ESP_LOGI( TAG, "Error while creating ADC data reader task. Error num: %d", (int)ret );
        if( NULL != m_adc_data_reader_task_handle )
        {
            vTaskDelete( m_adc_data_reader_task_handle );
            m_adc_data_reader_task_handle = NULL;   
        }

        return eStatus = execStatus::FAILURE;
    }   

    /* ADC data processor task creation*/
    ret = xTaskCreate
        (
            adc_data_processor_task,
            "ADC data processor task",
            ADC_DATA_PROC_TASK_STACK_SIZE,
            (void *)this,
            ADC_DATA_PROC_TASK_PRIORITY,
            &m_adc_data_proc_task_handle
        );

    if( pdTRUE != ret )
    {
        ESP_LOGI( TAG, "Error while creating ADC data processor task. Error num: %d", (int)ret );
        if( NULL != m_adc_data_proc_task_handle )
        {
            vTaskDelete( m_adc_data_proc_task_handle );
            m_adc_data_proc_task_handle = NULL;   
        }

        return eStatus = execStatus::FAILURE;
    }   

    eStatus = createCaliScheme();
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Successfully created calibration scheme for ADC%d module", ( m_adc_device.adc_unit + 1 ) );
    }

    eStatus = adc_continuous_mode_drv_init();
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Successfull initialization of continuous mode ADC driver");
        m_adc_driver_state = eDriverState::INITIALIZED;
    }

    return eStatus;
}

execStatus ADC_driver::deinit()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::STARTED == m_adc_driver_state )
    {
        eStatus = this->stop();
        if( execStatus::SUCCESS != eStatus )
        {
            ESP_LOGI( TAG, " Failed to stop driver!!!" );
            return eStatus;
        }
    }

    if( eDriverState::INITIALIZED != m_adc_driver_state && eDriverState::STOPPED != m_adc_driver_state )
    {
        ESP_LOGI( TAG, "Invalid state transition from %d to %d", (int)m_adc_driver_state, (int)eDriverState::DEINITIALIZED );
        return eStatus = execStatus::FAILURE;
    }

    esp_err = adc_continuous_deinit( m_adc_device.adc_continuous_mode_drv_handle );
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
        m_adc_driver_state = eDriverState::DEINITIALIZED;
    }

    return eStatus;
}

execStatus ADC_driver::start()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::INITIALIZED != m_adc_driver_state && eDriverState::STOPPED != m_adc_driver_state )
    {
        ESP_LOGI( TAG, "Driver is not initialized yet!!!" );
        return eStatus = execStatus::FAILURE;
    }

    if( NULL == m_adc_device.adc_continuous_mode_drv_handle )
    {
        ESP_LOGI( TAG, " ERROR: ADC driver handle is NULL" );
        return eStatus = execStatus::FAILURE;
    }

    esp_err = adc_continuous_start( m_adc_device.adc_continuous_mode_drv_handle );
    if( ESP_OK == esp_err )
    {
        m_adc_driver_state = eDriverState::STARTED;

        if( NULL != m_adc_data_reader_semphr )
        {  
            if( pdTRUE != xSemaphoreGive( m_adc_data_reader_semphr ) )
            {
                ESP_LOGW( TAG, "Failed to release semaphore and start the adc data reader task!!!" );
                return eStatus = execStatus::FAILURE;
            }
        }

        if( NULL != m_adc_data_proc_semphr )
        {  
            if( pdTRUE != xSemaphoreGive( m_adc_data_proc_semphr ) )
            {
                ESP_LOGW( TAG, "Failed to release semaphore and start the adc data processor task!!!" );
                return eStatus = execStatus::FAILURE;
            }
        }
        
        eStatus = execStatus::SUCCESS;
    }
   
    return eStatus;
}

execStatus ADC_driver::stop()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::STARTED != m_adc_driver_state )
    {
        ESP_LOGI( TAG, "Driver is not in STARTED internal state" );
        return eStatus;
    }

    esp_err = adc_continuous_stop( m_adc_device.adc_continuous_mode_drv_handle );
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
        m_adc_driver_state = eDriverState::STOPPED;
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

execStatus ADC_driver::setMultisamplingMode( multisampling_mode_t in_multisampling_mode )
{
    execStatus eStatus = execStatus::FAILURE;

    m_adc_device.multisampling_mode = in_multisampling_mode;
    if( m_adc_device.multisampling_mode == in_multisampling_mode )
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
    ESP_LOGI( TAG, "adc_unit_num = %d; adc_channel_num = %d ", (int)adc_unit_id, (int)adc_channel );
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
        adc_continuous_evt_cbs_t callback_function_mode = 
        {
            .on_conv_done = conv_done_callback,
            .on_pool_ovf = NULL
        };

        esp_err = adc_continuous_register_event_callbacks( adc_continuous_mode_drv_handle, &callback_function_mode, this );
        
        if( ESP_OK == esp_err )
        {
            esp_err = adc_continuous_config( adc_continuous_mode_drv_handle, &adc_continuous_mode_drv_cfg );
        }
        
        if( ESP_OK == esp_err )
        {
            eStatus = execStatus::SUCCESS;
            ESP_LOGI( TAG, " adc handle = %d", (int) adc_continuous_mode_drv_handle);
            m_adc_device.adc_continuous_mode_drv_handle = adc_continuous_mode_drv_handle;
            ESP_LOGI( TAG, " Configuration of continuous mode ADC driver finished successfully" );
        }   
    }
    
    return eStatus;
 }


/* Static functions definitions */

 bool ADC_driver::conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data )
 {
    BaseType_t mustYield = pdFALSE;
    ADC_driver * adc_driver = static_cast<ADC_driver *>( user_data );

    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR( adc_driver->m_adc_data_reader_task_handle, &mustYield );

    return (mustYield == pdTRUE);
}


void ADC_driver::adc_data_reader_task( void * pvUserData )
{
    uint8_t adc_raw_result[ CONVERSION_FRAME_SIZE ] = {0}; 
    uint32_t read_result_len = 0;
    const uint32_t timeout_ms = 100;
    const TickType_t ticks_to_wait_for_room = pdMS_TO_TICKS( 10 ); // 10 ms

    esp_err_t esp_err = ESP_FAIL;
    adc_continuous_handle_t adc_handle = NULL;
    RingbufHandle_t ring_buff_handle = NULL; 

    ADC_driver * adc_driver = static_cast<ADC_driver *>( pvUserData );

    /* label for goto statement */
    task_beginning:

        /* Wait in blocked state until the task is explicitly started */
        while( eDriverState::STARTED != adc_driver->m_adc_driver_state )
        {
            ESP_LOGI( TAG, "Waiting in blocked state until samphore is given" );
            (void)xSemaphoreTake( adc_driver->m_adc_data_reader_semphr, portMAX_DELAY );
            ESP_LOGI( TAG, "Task is explicitly started" );
        }
    
        adc_handle = adc_driver->m_adc_device.adc_continuous_mode_drv_handle;
        ring_buff_handle = adc_driver->m_adc_data_ring_buff_handle;

        while( eDriverState::STARTED == adc_driver->m_adc_driver_state )
        {
            /* Wait in blocked state until notification from adc conversion done callback is sent */
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        
            if( NULL != adc_handle )
            {
                esp_err = adc_continuous_read( adc_handle, (uint8_t *)&adc_raw_result, CONVERSION_FRAME_SIZE, &read_result_len, timeout_ms );
            }
        
            if( ESP_OK == esp_err )
            {
                /* Iterate over all raw data in the buffer with step equal to result bytes constant defined in soc_caps.h file */
                for( int dataIndex = 0; dataIndex < read_result_len; dataIndex += SOC_ADC_DIGI_RESULT_BYTES )
                {
                    adc_digi_output_data_t * adc_output_data = reinterpret_cast<adc_digi_output_data_t *>( &adc_raw_result[ dataIndex ]);
                    uint32_t adc_chan_num = adc_output_data->type1.channel;
                    uint32_t adc_raw_data = adc_output_data->type1.data;
                    
                    /* check validity of the result adc data */
                    if( adc_chan_num < SOC_ADC_CHANNEL_NUM( ADC_UNIT_1 ) )
                    {
                        int voltage = 0;
                        esp_err = adc_cali_raw_to_voltage( adc_driver->m_adc_device.adc_cali_scheme_handle, adc_raw_data, &voltage );
                        if( ESP_OK == esp_err )
                        {
                            //ESP_LOGI(TAG, "Channel: %d, Raw Value: %d Voltage: %d", (int)adc_chan_num, (int)adc_raw_data, (int)voltage );
                            UBaseType_t ret = xRingbufferSend( ring_buff_handle, &voltage, sizeof( voltage ), ticks_to_wait_for_room );
                            if( ret != pdTRUE )
                            {
                                //ESP_LOGE( TAG, " Failed to send data to ring buffer" );
                            }
                        }
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "Invalid data [%d_ %d]", (int)adc_chan_num, (int)adc_raw_data);
                    }
                }
            }
            /* minimum delay for Idle Task to do clean job and reset watchdog timer */
            vTaskDelay(1);
        }

        /* if task is stopped and go out of the while loop the control goes to the beginning of the task code */
        goto task_beginning;
}

void ADC_driver::adc_data_processor_task( void * pvUserData )
{
    RingbufHandle_t ring_buff_handle = NULL;
    size_t item_size = 0;
    int * received_item_ptr = NULL;

    ADC_driver * adc_driver = static_cast<ADC_driver *>( pvUserData );
    
     /* label for goto statement */
    task_beginning:

        /* Wait in blocked state until the task is explicitly started */
        while( eDriverState::STARTED != adc_driver->m_adc_driver_state )
        {
            ESP_LOGI( TAG, "Waiting in blocked state until samphore is given" );
            (void)xSemaphoreTake( adc_driver->m_adc_data_proc_semphr, portMAX_DELAY );
            ESP_LOGI( TAG, "Task is explicitly started" );
        }
    
        ring_buff_handle = adc_driver->m_adc_data_ring_buff_handle;
        int multisampling_window_size = static_cast<int>( adc_driver->m_adc_device.multisampling_mode );

        while( eDriverState::STARTED == adc_driver->m_adc_driver_state )
        {  
            int result = 0;
            int multisampling_index = 0;

            do
            {
                /* wait infinitely long time for item to be available in the ring buffer */
                received_item_ptr = (int * )xRingbufferReceive( ring_buff_handle, &item_size, portMAX_DELAY );
                result += *received_item_ptr;
                vRingbufferReturnItem( ring_buff_handle, (void *)received_item_ptr );
                multisampling_index++;

            } while ( multisampling_index < multisampling_window_size );
            
            result /= multisampling_index;
            
            printf( "%d", result );
            //adc_driver->m_data_logger.write_out( a );
            
            /* minimum delay for Idle Task to do clean job and reset watchdog timer */
            vTaskDelay(1);
        }

        /* if task is stopped and go out of the while loop the control goes to the beginning of the task code */
        goto task_beginning;
}