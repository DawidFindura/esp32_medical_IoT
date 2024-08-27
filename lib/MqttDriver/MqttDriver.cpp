#include <esp_log.h>

#include "MqttDriver.hpp"


static const char * TAG = "MQTT test";

/*Default MQTT credentials used if user does not provide the other ones*/
/**
 * @brief default MQTT broker URI
 * 
 * @var static const char * pcDefaultMqttUri 
 */
//static const char * pcDefaultMqttUri = "mqtt://incredible-dispatcher.cloudmqtt.com";

static const char * pcDefaultMqttUri = "mqtt://192.168.100.5";

static const uint32_t u32DefaultMqttPort = 1883;

/**
 * @brief default MQTT user name used during connection to MQTT broker
 * 
 * @var static const char * pcDefaultMqttUserName
 */
static const char * pcDefaultMqttUserName = "dawid_esp32";

/**
 * @brief default MQTT password used during connection to MQTT broker
 * 
 */
//static const char * pcDefaultMqttPassword = "2xXToHvIYFB_";

/**
 * @brief default MQTT topic used by MQTT client to subscribe and publish data 
 * 
 */
static const char * pcDefaultTopic = "esp_32/test";


using namespace Driver;


/**
 * @brief Construct a new Mqtt Driver - MqttDriver object
 * 
 * @details Initializes non-static members of MqttDriver class to default values.
 * Especially sets URI of the MQTT broker, username and password of MQTT client in m_sClientConfig member 
 * structure to default values defined in MqttDriver.hpp file
 */
MqttDriver::MqttDriver() : 
    m_sClientConfig(),
    m_reconnectNum( 0 ),
    m_psMqttEventGroup( NULL ),
    m_mqttClientHandle( NULL ),
    m_eState( eDriverState::UNINITIALIZED ),
    m_pcTopic( pcDefaultTopic ),
    bStopTask( pdFALSE ),
    m_psReceiverQueueHandle( NULL ),
    m_psPublisherQueueHandle( NULL ),
    m_sReceiverQueue(),
    m_sPublisherQueue(),
    au8PublisherQueueBuffer(),
    au8ReceiverQueueBuffer(),
    m_psPublisherTaskHandle( NULL ),
    m_psReceiverTaskHandle( NULL ),
    m_sReceiverTaskBuffer(),
    m_sPublisherTaskBuffer(),
    m_au8ReceiverTaskStack(),
    m_au8PublisherTaskStack()

{
    m_sClientConfig.broker.address.uri = pcDefaultMqttUri;
    m_sClientConfig.broker.address.port = u32DefaultMqttPort;
    m_sClientConfig.credentials.username = pcDefaultMqttUserName;
    //m_sClientConfig.credentials.authentication.password = pcDefaultMqttPassword;
}

/**
 * @brief Destroy the MqttDriver object.
 * 
 * @details Performs deinitialization of the MqttDriver object and unregister from Driver Manager instance.
 */
MqttDriver::~MqttDriver()
{
    this->deinit();
}

/**
 * @brief Initializes MqttDriver object with necessary configuration.
 * 
 * @fn execStatus MqttDriver::init()
 * @return execStatus The method execution status: SUCCESS or FAILURE
 */
execStatus MqttDriver::init()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    /**
    * @brief Initialization of m_sClientConfig structure to work as MQTT client. 
    * 
    * @details If URI of MQTT broker is not provided it will set URI, username and password to default ones defined
    * in MqttDriver.hpp file
    */
    if( NULL == m_sClientConfig.broker.address.uri )
    {
        m_sClientConfig.broker.address.uri = pcDefaultMqttUri;
        m_sClientConfig.broker.address.port = u32DefaultMqttPort;
        m_sClientConfig.credentials.username = pcDefaultMqttUserName;
        //m_sClientConfig.credentials.authentication.password = pcDefaultMqttPassword;
    }

    if( eDriverState::UNINITIALIZED == m_eState || eDriverState::DEINITIALIZED == m_eState )
    {
        m_mqttClientHandle = esp_mqtt_client_init( &m_sClientConfig );
        if( NULL != m_mqttClientHandle )
        {
            esp_mqtt_event_id_t eMqttEventAnyID = static_cast<esp_mqtt_event_id_t>( ESP_EVENT_ANY_ID );

            /* as last argument pass pointer to this object instance */
            esp_err = esp_mqtt_client_register_event( m_mqttClientHandle, eMqttEventAnyID, mqttEventHandler, (void *)this );
        }   
    }

    if( ESP_OK == esp_err )
    {
        m_psMqttEventGroup = xEventGroupCreate();
        if( NULL == m_psMqttEventGroup  )
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err )
    {
        m_psReceiverQueueHandle = xQueueCreateStatic( m_receiverQueueBufferSize, m_u8PduItemSize, au8ReceiverQueueBuffer, &m_sReceiverQueue );
        if( NULL == m_psReceiverQueueHandle )
        {
            ESP_LOGI(TAG,"m_psReceiverQueueHandle is null");
            esp_err = ESP_FAIL;
        }    
    }

    if( ESP_OK == esp_err )
    {
        m_psPublisherQueueHandle = xQueueCreateStatic( m_publisherQueueBufferSize, m_u8PduItemSize, au8PublisherQueueBuffer, &m_sPublisherQueue );
        if( m_psPublisherQueueHandle == NULL )
        {
            ESP_LOGI(TAG,"m_psPublisherQueueHandle is null");
            esp_err = ESP_FAIL;
        }  
    }

    if( ESP_OK == esp_err )
    {
        m_psReceiverTaskHandle = xTaskCreateStatic
                                ( 
                                    dataReceiverTask, 
                                    "DataReceiverTask",
                                    m_receiverTaskStackSize,
                                    (void *)this,
                                    m_receiverTaskPriority,
                                    m_au8ReceiverTaskStack,
                                    &m_sReceiverTaskBuffer
                                );
        
        if( m_psReceiverTaskHandle == NULL )
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err )
    {
        m_psPublisherTaskHandle = xTaskCreateStatic
                                ( 
                                    dataPublisherTask, 
                                    "DataPublisherTask",
                                    m_publisherTaskStackSize,
                                    (void *)this,
                                    m_publisherTaskPriority,
                                    m_au8PublisherTaskStack,
                                    &m_sPublisherTaskBuffer
                                );
        
        if( NULL == m_psPublisherTaskHandle )
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err )
    {
        m_eState = eDriverState::INITIALIZED;
        ret = execStatus::SUCCESS;
    }

    return ret;
}

execStatus MqttDriver::deinit()
{
    //sntp_stop();

    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;
    int32_t i32MsgID = -1;

    m_eState =  eDriverState::STOPPED;
    i32MsgID = esp_mqtt_client_unsubscribe( m_mqttClientHandle, m_pcTopic );
    if( i32MsgID != -1)
    {
        esp_err = ESP_OK; 
    }

    if( ESP_OK == esp_err )
    {
        esp_err = esp_mqtt_client_disconnect( m_mqttClientHandle );
    }
    
    if( ESP_OK == esp_err )
    {
        esp_err = esp_mqtt_client_stop( m_mqttClientHandle );
    }

    if( ESP_OK == esp_err )
    {
        esp_err = esp_mqtt_client_destroy( m_mqttClientHandle );
        m_mqttClientHandle = NULL;
    }

     if( ESP_OK == esp_err )
    {
        if( uxQueueMessagesWaiting( m_psReceiverQueueHandle ) == 0 )
        {
            vTaskSuspend( m_psReceiverTaskHandle );
            esp_err = ESP_OK;
        }
        else
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err )
    {
        if( uxQueueMessagesWaiting( m_psPublisherQueueHandle ) == 0 )
        {
            vTaskSuspend( m_psPublisherTaskHandle );
            esp_err = ESP_OK;
        }
        else
        {
            esp_err = ESP_FAIL;
        }
    }    

    if( ESP_OK == esp_err )
    {
        ESP_LOGI( TAG, "Mqtt tasks stopped" );
        bStopTask = pdTRUE;
    }

    if( ESP_OK == esp_err )
    {
        memset( &m_sClientConfig, 0, sizeof( m_sClientConfig ) );
        m_sClientConfig.broker.address.uri = NULL;
        m_sClientConfig.credentials.username = NULL;
        m_sClientConfig.credentials.authentication.password = NULL;

        vEventGroupDelete( m_psMqttEventGroup );
        MqttDriver::m_psMqttEventGroup = NULL;

        /*Static tasks deinit*/
        vTaskDelete( m_psPublisherTaskHandle );
        vTaskDelete( m_psReceiverTaskHandle );
        
        m_psPublisherTaskHandle = NULL;
        m_psReceiverTaskHandle = NULL;
        m_sReceiverTaskBuffer = {};
        m_sPublisherTaskBuffer = {};
        
        memset( m_au8ReceiverTaskStack, 0, sizeof( m_au8ReceiverTaskStack ) );
        memset( m_au8PublisherTaskStack, 0, sizeof( m_au8PublisherTaskStack ) );

        /*Static queues deinit*/
        m_sReceiverQueue = {};
        m_sPublisherQueue = {};
        m_psReceiverQueueHandle = NULL;
        m_psPublisherQueueHandle = NULL;
        
        memset( au8PublisherQueueBuffer, 0, sizeof( au8PublisherQueueBuffer ) );
        memset( au8ReceiverQueueBuffer, 0, sizeof( au8ReceiverQueueBuffer ) );

        m_eState = eDriverState::DEINITIALIZED;
        ret = execStatus::SUCCESS;
    }
    
    return ret;  
}

execStatus MqttDriver::start()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::INITIALIZED == m_eState || eDriverState::STOPPED == m_eState )
    {
        if( pdTRUE == bStopTask )
        {
            vTaskResume( m_psPublisherTaskHandle );
            vTaskResume( m_psReceiverTaskHandle );
            bStopTask = pdFALSE;
        }
        m_eState = eDriverState::STARTED;
        esp_err = ESP_OK;
    }
    else
    {
        ESP_LOGI(TAG, "MQTT client is not initialized");
        esp_err = ESP_FAIL;
    }

    if( ESP_OK == esp_err )
    {
        esp_err = esp_mqtt_client_start( m_mqttClientHandle );
    }

    if( ( ESP_OK == esp_err ) && ( NULL != m_psMqttEventGroup ) )
    {
        /* waits for specific bits to be set in event handler and returns the bits before the call returned */
        EventBits_t bits = xEventGroupWaitBits
                        (
                            m_psMqttEventGroup, 
                            MqttClientConnectedBit | MqttClientFailedBit,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY
                        );

        /* We can test now which event actually happened. */
        if( 0 != ( bits & MqttClientConnectedBit ) ) 
        {
            ESP_LOGI( TAG, "connected to MQTT broker:%s username:%s", m_sClientConfig.broker.address.uri, m_sClientConfig.credentials.username );
        } 
        else if( 0 != ( bits & MqttClientFailedBit ) ) 
        {
            ESP_LOGI( TAG, "Failed to connect to MQTT broker:%s, username:%s", m_sClientConfig.broker.address.uri, m_sClientConfig.credentials.username );
        }   
        else 
        {
            ESP_LOGE( TAG, "UNEXPECTED EVENT" );
        }                
       
        esp_err = ESP_OK;
    }
    else
    {
        esp_err = ESP_FAIL;
    }
   
    if( ESP_OK == esp_err )
    {
        ret = execStatus::SUCCESS;
    }

    return ret;
}

execStatus MqttDriver::stop()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    if( eDriverState::STARTED == m_eState )
    {
        m_eState = eDriverState::STOPPED;
        esp_err = esp_mqtt_client_stop( m_mqttClientHandle );
    }
    else
    {
        ESP_LOGI(TAG, "MQTT is not started yet");
        esp_err = ESP_FAIL;
    }
  
    if( ESP_OK == esp_err )
    {
        if( uxQueueMessagesWaiting( m_psReceiverQueueHandle ) == 0 )
        {
            vTaskSuspend( m_psReceiverTaskHandle );
            esp_err = ESP_OK;
        }
        else
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err )
    {
        if( uxQueueMessagesWaiting( m_psPublisherQueueHandle ) == 0 )
        {
            vTaskSuspend( m_psPublisherTaskHandle );
            esp_err = ESP_OK;
        }
        else
        {
            esp_err = ESP_FAIL;
        }
    }    

    if( ESP_OK == esp_err )
    {
        ESP_LOGI( TAG, "Mqtt tasks stopped" );
        bStopTask = pdTRUE;
        ret = execStatus::SUCCESS;
    }
    
    return ret;
}

execStatus MqttDriver::sendDataToDriver( const char * data )
{
    execStatus ret = execStatus::FAILURE;
    BaseType_t bStatus = pdFAIL;

    const char * data_to_send = data;
    if( NULL != m_psPublisherQueueHandle )
    {
        bStatus = xQueueSendToBack( m_psPublisherQueueHandle, data_to_send, 100 );
    }

    if( pdPASS == bStatus )
    {
        ret = execStatus::SUCCESS;
    }
    
    return ret;
    
}

void MqttDriver::mqttEventHandler( void * a_pvArgs, esp_event_base_t a_pcBase, int32_t a_i32EventID, void * a_pvEventData )
{
    /* cast of void pointer passed to event handler to appropriate type */
    MqttDriver * mqtt_driver = static_cast<MqttDriver *>( a_pvArgs );

    esp_mqtt_event_handle_t psMqttEventData = static_cast<esp_mqtt_event_handle_t>( a_pvEventData );
    esp_mqtt_client_handle_t psMqttClient = psMqttEventData->client;
    esp_mqtt_event_id_t eMqttID = static_cast<esp_mqtt_event_id_t>( a_i32EventID );

    //arrays needed for MQTT_EVENT_DATA case
    char acTopic[ mqtt_driver->m_maxMqttTopicLen + 1 ] = {};
    char acReceivedData[ m_u8PduItemSize ] = {};

    /* check what kind of event id was the cause of event handler call */
    switch( eMqttID )
    {
        case MQTT_EVENT_CONNECTED:
        { 
            ESP_LOGI( TAG, "MQTT event connected" );

            if( NULL == psMqttClient )
            {
                ESP_LOGE( TAG, "psMqttClient is NULL" );
                break;
            }

            esp_mqtt_client_subscribe( psMqttClient, mqtt_driver->m_pcTopic, 0 );
            mqtt_driver->m_reconnectNum = 0;
            xEventGroupSetBits( mqtt_driver->m_psMqttEventGroup, MqttDriver::MqttClientConnectedBit );
            break;
        }
        
        case MQTT_EVENT_DISCONNECTED:
        {
            ESP_LOGI( TAG, "MQTT client disconnected");
            if( ( eDriverState::STARTED == mqtt_driver->m_eState ) && ( mqtt_driver->m_reconnectNum <  mqtt_driver->m_u8MaxReconnectNum ) )
            {
                esp_mqtt_client_reconnect( psMqttClient );
                ESP_LOGI(TAG, "trying to reconnect:%d", mqtt_driver->m_reconnectNum);
                mqtt_driver->m_reconnectNum++;
            }
            else
            {
                xEventGroupSetBits( mqtt_driver->m_psMqttEventGroup, MqttDriver::MqttClientFailedBit );
            }

            break;
        }
        
        case MQTT_EVENT_DATA:
        {
            memset( acTopic, 0, sizeof( acTopic ) );
            if( psMqttEventData->topic_len <= mqtt_driver->m_maxMqttTopicLen )
            {
                memcpy( acTopic, psMqttEventData->topic, psMqttEventData->topic_len );
            }

            ESP_LOGI( TAG, "MQTT_EVENT_DATA topic: %s", acTopic );
            if( strcmp( acTopic, mqtt_driver->m_pcTopic ) == 0 )
            {
                memset( acReceivedData, 0, sizeof( acReceivedData ) );
                if( psMqttEventData->data_len <= m_u8PduItemSize )
                {
                    memcpy( acReceivedData, psMqttEventData->data, psMqttEventData->data_len );
                    ESP_LOGI( TAG, "Data received: %s and data_length: %d ", acReceivedData, psMqttEventData->data_len );
                    xQueueSendToBack( mqtt_driver->m_psReceiverQueueHandle, acReceivedData, 100 );
                }
            }
            
            break;
        }

        default:
        {
            break;
        }
        
    }
}

execStatus MqttDriver::setMqttBroker( const char * a_pcBrokerUri, const char * a_pcUsername, const char * a_pcPassword )
{
    execStatus result = execStatus::FAILURE;
    
    bool wasStarted = false;
    if( eDriverState::STARTED == m_eState )
    {
        wasStarted = true;
    }

    if( eDriverState::UNINITIALIZED != m_eState && eDriverState::DEINITIALIZED != m_eState )
    {
        result = this->deinit();
    }
    else
    {
        result = execStatus::SUCCESS;
    }
    
    if( execStatus::SUCCESS == result )
    {
        m_sClientConfig.broker.address.uri = a_pcBrokerUri;
        m_sClientConfig.credentials.username = a_pcUsername;
        m_sClientConfig.credentials.authentication.password = a_pcPassword;
        result = this->init();
    }
    if( execStatus::SUCCESS == result )
    {
        if( true == wasStarted )
        {
            result = this->start();
        }
    }
    
    return result;
}

void MqttDriver::dataReceiverTask( void * a_pvArgs )
{
    /*cast void pointer passed as argument of Receiver task to pointer to MqttDriver object */
    MqttDriver * mqtt_driver = static_cast<MqttDriver *>( a_pvArgs );

    uint8_t au8ReceivedData[ m_u8PduItemSize ] = {};
    BaseType_t bStatus = pdFAIL;

    /*indefinitely wait for message in the queue*/
    const TickType_t ticksToWait = portMAX_DELAY;

    for( ;; )
    {
        /*wait for message passed into the queue. If queue is empty change state of the current task to blocked */
        bStatus = xQueueReceive( mqtt_driver->m_psReceiverQueueHandle, au8ReceivedData, ticksToWait );

        if( pdPASS == bStatus )
        {
            ESP_LOGI("Reader TASK", "Received the message. Message length %d:", sizeof( au8ReceivedData ) );
            
            /* TO DO */
            //memcpy( &sPduMessage, au8ReceivedData, sizeof( au8ReceivedData ) );
            //driverManager->sendData( sPduMessage );
        }
    }    
}

void MqttDriver::dataPublisherTask( void * a_pvArgs )
{
    BaseType_t bStatus = pdFAIL;
    
    /* TO DO */
    struct sReceivedPdu
    {
        char data[m_u8PduItemSize];
    } sReceivedPdu;

    /* cast void pointer passed as argument of Receiver task to pointer to MqttDriver object */
    MqttDriver * mqtt_driver = static_cast<MqttDriver *>( a_pvArgs );

    /*indefinitely long wait for message in the queue*/
    const TickType_t ticksToWait = portMAX_DELAY;

    char acDataBuf[ m_u8PduItemSize ] = {};

    int32_t i32MsgID = -1;
    
    for ( ;; )
    {
        /*wait for message passed into the queue. If its empty the change state to blocked task*/
        bStatus = xQueueReceive( mqtt_driver->m_psPublisherQueueHandle, &sReceivedPdu, ticksToWait );

        if( pdPASS == bStatus )
        {
            memcpy( acDataBuf, &sReceivedPdu, sizeof( sReceivedPdu ) );
            char * pcDataToSend = static_cast<char *>( &acDataBuf[0] );
            
            i32MsgID = esp_mqtt_client_publish( mqtt_driver->m_mqttClientHandle, mqtt_driver->m_pcTopic, pcDataToSend, m_u8PduItemSize, 0, 0 );
            
            /* chceck the message id after publishing; equals 0 on successfull publish */
            if( ( -1 == i32MsgID ) || ( -2 == i32MsgID ) )
            {
                switch( i32MsgID )
                {
                    case -1:
                    {
                        ESP_LOGE( TAG, "ERROR: Failed to publish data to MQTT broker" );
                        break;   
                    }

                    case -2:
                    {
                        ESP_LOGW( TAG, "The outbox of MQTT client is full ");
                        break;
                    }
                }
            } 
        }
    }
}