#include <esp_log.h>
#include <math.h>

#include "max30102Driver.hpp"
#include "algorithm.h"

#define RING_BUFFER_ITEM_SIZE           (sizeof( ring_buff_item_t ))
#define RING_BUFFER_MAX_ITEMS           (32U) 

#define DATA_READER_TASK_STACK_SIZE     (3200U) // bytes
#define DATA_READER_TASK_PRIORITY       (5U)

#define DATA_PROC_TASK_STACK_SIZE       (10000U) // bytes
#define DATA_PROC_TASK_PRIORITY         (DATA_READER_TASK_PRIORITY)

#define GPIO_INTERRUPT_PIN              (GPIO_NUM_25)   
#define ESP_INTR_FLAG_DEFAULT           (0)

using namespace Driver;

static const char * TAG = "MAX30102_Driver";

#define DEBUG

Max30102Driver::Max30102Driver( Interface::IDriverManager & driverManager ) : 
    
    m_driverManager( driverManager ), 
    m_driverState( eDriverState::UNINITIALIZED ),
    m_max30102Device{},
    m_dataReaderTaskHandle( NULL ),
    m_dataProcessorTaskHandle( NULL ),
    m_ring_buff_handle( NULL ),
    m_readerTaskStartSmphrHandle( NULL ),
    m_processorTaskStartSmphrHandle( NULL ),
    m_readerTaskFinishedSmphrHandle( NULL ),
    m_processorTaskFinishedSmphrHandle( NULL )

{
    execStatus eStatus = m_driverManager.registerDriver( this, m_driverID );
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE( TAG, "Could not register driver: %d", (int)m_driverID );
    }
} 

Max30102Driver::~Max30102Driver()
{
    execStatus eStatus = m_driverManager.unregisterDriver( this, m_driverID );
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE( TAG, "Could not unregister driver: %d", (int)m_driverID );
    }

    this->deinit();
}

execStatus Max30102Driver::init()
{
    execStatus eStatus = execStatus::FAILURE;

    if( eDriverState::UNINITIALIZED != m_driverState && eDriverState::DEINITIALIZED != m_driverState )
    {
        ESP_LOGI( TAG, "Driver already initialized!!!" );
        return eStatus;
    }

    //set default configuration of max30102 device registers
    esp_err_t esp_err = max30102_default_config_init( &m_max30102Device );
    if( ESP_OK == esp_err )
    {
        //initialization of max30102 device
        esp_err = max30102_device_init( &m_max30102Device );
    }
    
    /* Creation of ring buffer for max30102 result data */
    m_ring_buff_handle = xRingbufferCreateNoSplit( RING_BUFFER_ITEM_SIZE, RING_BUFFER_MAX_ITEMS );
    if( NULL == m_ring_buff_handle )
    {
        ESP_LOGW( TAG, "Failed to create ring buffer" );
        return eStatus = execStatus::FAILURE;
    }

    /* Creation of task synchronizing semaphore */
    m_readerTaskStartSmphrHandle = xSemaphoreCreateBinary();
    if( NULL == m_readerTaskStartSmphrHandle )
    {
        ESP_LOGW( TAG, "Failed to create binary start semaphore for data reader task" );
        return eStatus = execStatus::FAILURE;   
    }

    /* Creation of task synchronizing semaphore */
    m_processorTaskStartSmphrHandle = xSemaphoreCreateBinary();
    if( NULL == m_processorTaskStartSmphrHandle )
    {
        ESP_LOGW( TAG, "Failed to create binary start semaphore for data processor task" );
        return eStatus = execStatus::FAILURE;   
    }

     /* Creation of task synchronizing semaphore */
    m_readerTaskFinishedSmphrHandle = xSemaphoreCreateBinary();
    if( NULL == m_readerTaskFinishedSmphrHandle )
    {
        ESP_LOGW( TAG, "Failed to create binary finished semaphore for data reader task" );
        return eStatus = execStatus::FAILURE;   
    }

    /* Creation of task synchronizing semaphore */
    m_processorTaskFinishedSmphrHandle = xSemaphoreCreateBinary();
    if( NULL == m_processorTaskFinishedSmphrHandle )
    {
        ESP_LOGW( TAG, "Failed to create binary finished semaphore for data processor task" );
        return eStatus = execStatus::FAILURE;   
    }

    /* max30102 data reader task creation*/
    BaseType_t ret = xTaskCreate
                    (
                        dataReaderTask,
                        "MAX30102 data reader task",
                        DATA_READER_TASK_STACK_SIZE,
                        (void *)this,
                        DATA_READER_TASK_PRIORITY,
                        &m_dataReaderTaskHandle
                    );

    if( pdTRUE != ret )
    {
        ESP_LOGI( TAG, "Error while creating max30102 data reader task. Error num: %d", (int)ret );
        if( NULL != m_dataReaderTaskHandle )
        {
            vTaskDelete( m_dataReaderTaskHandle );
            m_dataReaderTaskHandle = NULL;   
        }

        return eStatus = execStatus::FAILURE;
    }   

    /* MAX30102 data processor task creation*/
    ret = xTaskCreate
        (
            dataProcessorTask,
            "max30102 data processor task",
            DATA_PROC_TASK_STACK_SIZE,
            (void *)this,
            DATA_PROC_TASK_PRIORITY,
            &m_dataProcessorTaskHandle
        );

    if( pdTRUE != ret )
    {
        ESP_LOGI( TAG, "Error while creating ADC data processor task. Error num: %d", (int)ret );
        if( NULL != m_dataProcessorTaskHandle )
        {
            vTaskDelete( m_dataProcessorTaskHandle );
            m_dataProcessorTaskHandle = NULL;   
        }

        return eStatus = execStatus::FAILURE;
    }   

    eStatus = this->gpioIntrInit();
    if( execStatus::SUCCESS != eStatus )
    {
        return eStatus;
    }

    // if( ESP_OK == esp_err )
    // {
    //     eStatus = execStatus::SUCCESS;
    //     m_driverState = eDriverState::INITIALIZED;
    // }
    eStatus = execStatus::SUCCESS;
    m_driverState = eDriverState::INITIALIZED;

    return eStatus;
}

execStatus Max30102Driver::deinit()
{
    execStatus eStatus = execStatus::FAILURE;

    if( eDriverState::STARTED == m_driverState )
    {
        eStatus = this->stop();
        if( execStatus::SUCCESS != eStatus )
        {
            ESP_LOGI( TAG, " Failed to stop driver!!!" );
            return eStatus;
        }
    }

    if( eDriverState::INITIALIZED != m_driverState && eDriverState::STOPPED != m_driverState )
    {
        ESP_LOGI( TAG, "Invalid state transition from %d to %d", (int)m_driverState, (int)eDriverState::DEINITIALIZED );
        return eStatus = execStatus::FAILURE;
    }

    vTaskDelete( m_dataReaderTaskHandle );
    m_dataReaderTaskHandle = NULL;

    vTaskDelete( m_dataProcessorTaskHandle );
    m_dataProcessorTaskHandle = NULL;

    vRingbufferDelete( m_ring_buff_handle );
    m_ring_buff_handle = NULL;

    vSemaphoreDelete( m_readerTaskStartSmphrHandle );
    m_readerTaskStartSmphrHandle = NULL;

    vSemaphoreDelete( m_processorTaskStartSmphrHandle );
    m_processorTaskStartSmphrHandle = NULL;

    vSemaphoreDelete( m_readerTaskFinishedSmphrHandle );
    m_readerTaskFinishedSmphrHandle = NULL;

    vSemaphoreDelete( m_processorTaskFinishedSmphrHandle );
    m_processorTaskFinishedSmphrHandle = NULL;
    
    gpio_uninstall_isr_service();

    /* clear configuration of max 30102 device */
    m_max30102Device = {};

    /* reset registers to power on state */
    esp_err_t esp_err =  max30102_device_reset( &m_max30102Device );

    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
        m_driverState = eDriverState::DEINITIALIZED;
    }

    return eStatus;
}

execStatus Max30102Driver::start()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::INITIALIZED != m_driverState && eDriverState::STOPPED != m_driverState )
    {
        ESP_LOGI( TAG, "Driver is not initialized yet!!!" );
        return eStatus = execStatus::FAILURE;
    }

    if( UNINITIALIZED == m_max30102Device.dev_status )
    {
        ESP_LOGI( TAG, " ERROR: max30102 device is not initialized" );
        return eStatus = execStatus::FAILURE;
    }


    m_driverState = eDriverState::STARTED;

    if( NULL != m_readerTaskStartSmphrHandle )
    {  
        if( pdTRUE != xSemaphoreGive( m_readerTaskStartSmphrHandle ) )
        {
            ESP_LOGW( TAG, "Failed to release semaphore and start the max30102 data reader task!!!" );
            return eStatus = execStatus::FAILURE;
        }
    }

    if( NULL != m_processorTaskStartSmphrHandle )
    {  
        if( pdTRUE != xSemaphoreGive( m_processorTaskStartSmphrHandle ) )
        {
            ESP_LOGW( TAG, "Failed to release semaphore and start the max30102 data processor task!!!" );
            return eStatus = execStatus::FAILURE;
        }
    }
        
    esp_err = max30102_start_data_acquisition( &m_max30102Device );
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
    }

    ESP_LOGI( TAG, "driver started with status code: %d", (int)eStatus );

    return eStatus;
}


execStatus Max30102Driver::stop()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::STARTED != m_driverState )
    {
        ESP_LOGI( TAG, "Driver is not in STARTED internal state" );
        return eStatus;
    }

    m_driverState = eDriverState::STOPPED;
    ESP_LOGI( TAG, "Change state to STOPPED");

    if( pdTRUE == xSemaphoreTake( m_readerTaskFinishedSmphrHandle, portMAX_DELAY ) )
    {
        ESP_LOGI( TAG, "Data reader task finished" );
    }

    esp_err = max30102_device_turn_off( &m_max30102Device );
    if( ESP_OK == esp_err )
    {   
        if( pdTRUE == xSemaphoreTake( m_processorTaskFinishedSmphrHandle, portMAX_DELAY ) )
        {
            ESP_LOGI( TAG, "Data processor task finished" );
            eStatus = execStatus::SUCCESS;
        } 
    }
    
    return eStatus;
}

execStatus Max30102Driver::forwardMessage( const pduMessage_t & pduMessage )
{
    execStatus eStatus = execStatus::FAILURE;

    return eStatus;
}

execStatus Max30102Driver::gpioIntrInit()
{
    execStatus eStatus = execStatus::FAILURE;

    // GPIO configuration structure
    gpio_config_t io_conf = {};
    
    // Set as input mode for the selected pin
    io_conf.intr_type = GPIO_INTR_NEGEDGE;                  // Interrupt on falling edge
    io_conf.mode = GPIO_MODE_INPUT;                         // Input mode
    io_conf.pin_bit_mask = ( 1ULL << GPIO_INTERRUPT_PIN );  // Pin mask for the GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;           // Disable pull-down
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                // Enable pull-up

    // Apply the configuration
    esp_err_t esp_err = gpio_config(&io_conf);

    if( ESP_OK == esp_err )
    {
        // Install the GPIO ISR handler
        esp_err = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    }
    
    if( ESP_OK == esp_err )
    {
        // Attach the ISR handler for the specific pin
        esp_err = gpio_isr_handler_add( GPIO_INTERRUPT_PIN, max30102_intr_handler, this );
    }
    
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}

void Max30102Driver::dataReaderTask( void * pvUserData )
{
    /* time to wait for free space in ring buffer before timeout */
    TickType_t ticks_to_wait_for_room = pdMS_TO_TICKS( 100 ); // 100 ms 
    
    ring_buff_item_t ring_buff_item;
    //static uint32_t numOfSentSamples = 0;

    Max30102Driver * maxDriver = (Max30102Driver *) pvUserData;

    /* label for goto statement */
    task_beginning:

        /* Wait in blocked state until the task is explicitly started */
        while( eDriverState::STARTED != maxDriver->m_driverState )
        {
            ESP_LOGI( TAG, "Data reader task: Waiting in blocked state until samphore is given" );
            if( pdTRUE == xSemaphoreTake( maxDriver->m_readerTaskStartSmphrHandle, portMAX_DELAY ) )
            {
                ESP_LOGI( TAG, "Data reader task: Task is explicitly started" );
            }
        }

        /* clear max30102 internal data registers and turn on the device */
        esp_err_t esp_err = max30102_start_data_acquisition( &maxDriver->m_max30102Device );
        if( ESP_OK != esp_err )
        {
            ESP_LOGE( TAG, "ERROR: could not start data acquisition" );
        }

        //numOfSentSamples = 0;

        /* Enable die temperature acquisition */
        maxDriver->m_max30102Device.registers.die_temp_config_reg.temp_en_bit = 1; 
        esp_err = max30102_set_register( (max30102_generic_register_t *) &(maxDriver->m_max30102Device.registers.die_temp_config_reg) );
        if( esp_err != ESP_OK )
        {
            ESP_LOGW( TAG, "Failed to set temp config register ");
        }

        while( eDriverState::STARTED == maxDriver->m_driverState )
        {   
            // esp_err = max30102_device_turn_on( &maxDriver->m_max30102Device );
            // if( ESP_OK != esp_err )
            // {
            //     ESP_LOGE( TAG, "Failed to turn on the device" );
            // }

            /* Wait in blocked state until notification from max30102 ISR is sent */
            ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

            uint8_t interruptSrc = 0;
            execStatus eStatus = maxDriver->getInterruptSrc( maxDriver->m_max30102Device, interruptSrc );
            if( execStatus::SUCCESS != eStatus )
            {
                ESP_LOGW( TAG, "Failed to get interrupt source ");
                interruptSrc = 0;
            }

            if( interruptSrc & MAX30102_INTR_TYPE_ALMFULL )
            {   
                #ifdef DEBUG
                    //ESP_LOGI( TAG, "Almost full interrupt");
                #endif 

                uint8_t numOfSamples = 0;
                esp_err = max30102_fifo_data_read( &maxDriver->m_max30102Device, &numOfSamples );
                if( ESP_OK == esp_err )
                {
                    for( int sampleIndex = 0; sampleIndex < numOfSamples; sampleIndex++ )
                    {
                        ring_buff_item.red_led = maxDriver->m_max30102Device.red_led_buffer[sampleIndex];
                        ring_buff_item.ir_led = maxDriver->m_max30102Device.ir_led_buffer[sampleIndex];

                        UBaseType_t ret = xRingbufferSend( maxDriver->m_ring_buff_handle, &ring_buff_item, sizeof( ring_buff_item_t ), ticks_to_wait_for_room );
                        if( ret == pdTRUE )
                        {
                            //numOfSentSamples += numOfSamples;
                        }
                        else
                        {
                            ESP_LOGE( TAG, " Failed to send data to ring buffer" );
                        }
                    }
                }
                else
                {
                    ESP_LOGE( TAG, "ERROR: could not read fifo data" );
                }
                
            }

            if( interruptSrc & MAX30102_INTR_TYPE_DIETEMP_RDY )
            {   
                esp_err = max30102_get_die_temp( &maxDriver->m_max30102Device );
                if( ESP_OK != esp_err )
                {
                    ESP_LOGE( TAG, "ERROR: could not get die temperature " );
                }

                #ifdef DEBUG
                    //ESP_LOGI( TAG, "Die temp rdy interrupt: %.2f", maxDriver->m_max30102Device.die_temp_buff );
                #endif
                
            }
            
            // if( numOfSentSamples >= BUFFER_SIZE *2)
            // {
            //     max30102_device_turn_off( &maxDriver->m_max30102Device );
            //     numOfSentSamples = 0;

            //     vTaskDelay( pdMS_TO_TICKS(1000) ); // one second delay
            // }
        }

        /* task signals main task that it finsihed */
        xSemaphoreGive( maxDriver->m_readerTaskFinishedSmphrHandle );
        
        /* if task is stopped and go out of the while loop the control goes to the beginning of the task code */
        goto task_beginning;
}

void Max30102Driver::dataProcessorTask( void * pvUserData )
{   
    ring_buff_item_t * received_item_ptr = NULL;

    Max30102Driver * maxDriver = static_cast<Max30102Driver *>( pvUserData );
    
    pduMessage_t pduToSend;
    pduToSend.header.driverID = (uint8_t)maxDriver->m_driverID;
    pduToSend.header.deviceID = 0;
    pduToSend.header.externalDev = 0;

    uint32_t ir_buffer[BUFFER_SIZE];
    uint32_t red_buffer[BUFFER_SIZE];

    int32_t heartRate = 0;
    int32_t lastValidHeartRate = 0;

    float SpO2 = 0.0f;
    int32_t lastValidSpo2 = 0;
    
    int8_t hrValid;
    int8_t spValid;
    
    float ratio = 0.0f;
    float correl = 0.0f;

     /* label for goto statement */
    task_beginning:

        /* Wait in blocked state until the task is explicitly started */
        while( eDriverState::STARTED != maxDriver->m_driverState )
        {
            ESP_LOGI( TAG, "Data proc task: Waiting in blocked state until samphore is given" );
            if( pdTRUE == xSemaphoreTake( maxDriver->m_processorTaskStartSmphrHandle, portMAX_DELAY ) )
            {
                ESP_LOGI( TAG, "Data proc task: Task is explicitly started" );
            }
        }
    
        RingbufHandle_t ring_buff_handle = maxDriver->m_ring_buff_handle;
        bool itemsWaiting = false;
        bool firstTime = true;
        uint32_t read_offset = 0;
        uint32_t write_offset = 0;

        /* task can go into stopped state only after it empties the ring buffer */
        while( eDriverState::STARTED == maxDriver->m_driverState || itemsWaiting )
        {  
            UBaseType_t numOfItemsWaiting = 0;
            size_t item_size = 0;

            if( true == firstTime )
            {   
                vRingbufferGetInfo( ring_buff_handle, NULL, NULL, NULL, NULL, &numOfItemsWaiting );
                if( numOfItemsWaiting >= BUFFER_SIZE )
                {
                    itemsWaiting = true;
                }
                else
                {
                    itemsWaiting = false;
                }

                /* first time initialization of led's buffers*/
                for( int sampleIndex = 0; (sampleIndex < BUFFER_SIZE) && (eDriverState::STARTED == maxDriver->m_driverState || itemsWaiting); sampleIndex++)
                {
                    /* wait infinitely long time for item to be available in the ring buffer */
                    received_item_ptr = (ring_buff_item_t * )xRingbufferReceive( ring_buff_handle, &item_size, portMAX_DELAY );
                    ir_buffer[sampleIndex] = received_item_ptr->red_led;
                    red_buffer[sampleIndex] = received_item_ptr->ir_led;
                    vRingbufferReturnItem( ring_buff_handle, (void *)received_item_ptr );
                }

                firstTime = false;  
            }
            

            vRingbufferGetInfo( ring_buff_handle, NULL, NULL, NULL, NULL, &numOfItemsWaiting );
            if( numOfItemsWaiting >= SAMPLES_IN_ONE_SECOND )
            {
                itemsWaiting = true;
            }
            else
            {
                itemsWaiting = false;
            }

            for( int sampleIndex = 0; (sampleIndex < SAMPLES_IN_ONE_SECOND) && (eDriverState::STARTED == maxDriver->m_driverState || itemsWaiting); sampleIndex++ )
            {
                /* wait infinitely long time for item to be available in the ring buffer */
                received_item_ptr = (ring_buff_item_t * )xRingbufferReceive( ring_buff_handle, &item_size, portMAX_DELAY );

                /* we need to swap red_led and ir_led buffers because of chinese copy of our module max30102*/
                ir_buffer[write_offset] = received_item_ptr->red_led;
                red_buffer[write_offset] = received_item_ptr->ir_led;
                vRingbufferReturnItem( ring_buff_handle, (void *)received_item_ptr );

                write_offset++;
                if( write_offset >= BUFFER_SIZE )
                {
                    write_offset = 0;
                }

                read_offset = write_offset;  
            }

            // Process the data to calculate heart rate and SpO2
            rf_heart_rate_and_oxygen_saturation( ir_buffer, BUFFER_SIZE, red_buffer, read_offset, &SpO2, &spValid, &heartRate, &hrValid, &ratio, &correl);
            
            /* Enable another die temperature acquisition */
            maxDriver->m_max30102Device.registers.die_temp_config_reg.temp_en_bit = 1; 
            (void)max30102_set_register( (max30102_generic_register_t *) &(maxDriver->m_max30102Device.registers.die_temp_config_reg) );
            
            if( spValid && hrValid )
            {
                lastValidHeartRate = static_cast<int32_t>(heartRate);
                lastValidSpo2 = static_cast<int32_t>(SpO2);
            }
    
            memcpy( &pduToSend.signals[0], &lastValidHeartRate, sizeof(lastValidHeartRate) );
            memcpy( &pduToSend.signals[4], &lastValidSpo2, sizeof(lastValidSpo2) );
            memcpy( &pduToSend.signals[8], &maxDriver->m_max30102Device.die_temp_buff, sizeof(int32_t) );

            execStatus eStatus = maxDriver->m_driverManager.sendMessage( pduToSend );
            if( execStatus::SUCCESS != eStatus )
            {
                ESP_LOGE( TAG, "Failed to send message to diver manager" );
            }

            #ifdef DEBUG
                ESP_LOGI(TAG, "Heart Rate: %ld bpm, SpO2: %.2f, ratio: %.2f, correl: %.2f, temp: %ld", heartRate, SpO2, ratio, correl, maxDriver->m_max30102Device.die_temp_buff);
            #endif

            /* minimum delay for Idle Task to do clean job and reset watchdog timer */
            vTaskDelay( pdMS_TO_TICKS( 1 ));
        }

        /* task signals that it finished  */
        xSemaphoreGive( maxDriver->m_processorTaskFinishedSmphrHandle );

        /* if task is stopped and go out of the while loop the control goes to the beginning of the task code */
        goto task_beginning;
}

void Max30102Driver::max30102_intr_handler( void * pvArgs )
{
    BaseType_t mustYield = pdFALSE;
    Max30102Driver * maxDriver = static_cast<Max30102Driver *>( pvArgs );

    if( nullptr != maxDriver )
    {   
        vTaskNotifyGiveFromISR( maxDriver->m_dataReaderTaskHandle, &mustYield );
    }

    if( pdTRUE == mustYield )
    {
        portYIELD_FROM_ISR();
    }
    
}


execStatus Max30102Driver::getInterruptSrc( max30102_device_t & dev, uint8_t & intr_src ) 
{
    execStatus eStatus = execStatus::FAILURE;
    intr_src = 0;

    max30102_generic_register_t * intr_status_reg_1 = (max30102_generic_register_t *) &(dev.registers.intr_status_1_reg);
    max30102_generic_register_t * intr_status_reg_2 = (max30102_generic_register_t *) &(dev.registers.intr_status_2_reg);

    esp_err_t esp_err = max30102_get_register( intr_status_reg_1, 1 );
    if( ESP_OK == esp_err )
    {
        esp_err = max30102_get_register( intr_status_reg_2, 1 );
    }

    if( ESP_OK == esp_err )
    {
        intr_src = dev.registers.intr_status_1_reg.reg_val | dev.registers.intr_status_2_reg.reg_val;
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}
