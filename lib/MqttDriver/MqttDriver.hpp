#ifndef MQTTDRIVER_HPP
#define MQTTDRIVER_HPP

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>

#include "Interfaces/IDriver.hpp"
#include "DriverManager.hpp"
#include "Enums/eDriverState.hpp"
#include "Enums/eExecStatus.hpp"
#include "Enums/eDriverID.hpp"
#include "common.hpp"


namespace Driver
{
     /**
     * @brief MqttDriver class that inherits IDriver, IWriteLog interface
     * 
     * @class MqttDriver
     * 
     */
    class MqttDriver : public Interface::IDriver
    {
    private:
        
        Interface::IDriverManager & m_driverManager;

        const eCommDriverID m_commDriverID = eCommDriverID::MQTT_DRIVER;

        /**
         * @brief MQTT client configuration structure
         * 
         */
        esp_mqtt_client_config_t m_sClientConfig;

        /**
         * @brief Private enum definition for class MqttDriver. Defines MqttClientFailedBit and MqttClientConnectedBit used in FreeRTOS 
         * Event Group during xEventGroupWaitBits finction call.
         * 
         * @enum eMqttBits
         */
        enum eMqttBits
        {
            MqttClientFailedBit     = ( 1 << 0 ),
            MqttClientConnectedBit  = ( 1 << 1 ),
        };

        /**
         * @brief private member of class MqttDriver. Counts the number of reconnection tries of MQTT client to MQTT broker.
         *
         */
        uint8_t m_reconnectNum;

        /** 
         * @brief maximum number of mqtt client reconnection tries if it is disconnected from the broker
         * 
         */
        const uint8_t m_u8MaxReconnectNum = 5U;

        /**
         * @brief  size of item being passed into to the queues
         * 
         */
        static const uint8_t m_u8PduItemSize = sizeof( pduMessage_t );

        /**
         * @brief MQTT message receiver task priority
         */
        const uint8_t m_receiverTaskPriority = 5;

        /**
         * @brief MQTT message publisher task priority
         */
        const uint8_t m_publisherTaskPriority = 5;

        /**
         * @brief size of the Queue Buffer used in Publisher task
         * 
         */
        static const uint8_t m_publisherQueueBufferSize = 30U;

        /**
         * @brief  size of the Queue Buffer used in Receiver task
         *
         */
        static const uint8_t m_receiverQueueBufferSize = 30U;

        /**
         * @brief stack size in bytes of Publisher task
         * 
         */
        static const uint16_t m_publisherTaskStackSize = 2000U;

        /**
         * @brief stack size in bytes of Receiver task
         * 
         */
        static const uint16_t m_receiverTaskStackSize = 2000U;

        /**
         * @brief maximum length of topic used in connection with the MQTT broker.
         * 
         */
        const uint8_t m_maxMqttTopicLen = 25U;
    
        /**
         * @brief FreeRTOS event group handle
         * 
         */
        EventGroupHandle_t m_psMqttEventGroup;

        /**
         * @brief handle to mqtt client structure returned during Mqtt client initialization
         * 
         */
        esp_mqtt_client_handle_t m_mqttClientHandle;

        /**
         * @brief enum variable decribing current state of Mqtt driver
         * 
         */
        eDriverState m_eState;

        /**
         * @brief topic used to subscribe and publish MQTT message
         * 
         */
        const char * m_pcTopic;

        /**
         * @brief boolean variable which is used to stop the running task
         * 
         */
        bool bStopTask;

        /**
         * @brief handle to the receiver queue being used in receiver task
         * 
         */
        QueueHandle_t m_psReceiverQueueHandle;

        /**
         * @brief handle to the publisher queue being used in publisher task
         * 
         */
        QueueHandle_t m_psPublisherQueueHandle;

        /**
         * @brief holds the data structure of receiver queue
         * 
         */
        StaticQueue_t m_sReceiverQueue;

        /**
         * @brief holds the data structure of publisher queue
         * 
         */
        StaticQueue_t m_sPublisherQueue;

        /**
         * @brief buffer for queue used in the publisher task
         * 
         */
        uint8_t au8PublisherQueueBuffer[ m_publisherQueueBufferSize * m_u8PduItemSize ];

        /**
         * @brief buffer for queue used in the receiver task
         * 
         */
        uint8_t au8ReceiverQueueBuffer[ m_receiverQueueBufferSize * m_u8PduItemSize ];

        /**
         * @brief task handle of the Publisher task
         * 
         */
        TaskHandle_t m_psPublisherTaskHandle;

        /**
         * @brief task handle of the Receiver task
         * 
         */
        TaskHandle_t m_psReceiverTaskHandle;

        /**
         * @brief  Structure that will hold the TCB's of the Receiver task being created.
         * 
         * @var static StaticTask_t m_sReceiverTaskBuffer
         */
        StaticTask_t m_sReceiverTaskBuffer;

         /**
         * @brief  Structure that will hold the TCB's of the Publisher task being created.
         * 
         * @var static StaticTask_t m_sPublisherTaskBuffer
         */
        StaticTask_t m_sPublisherTaskBuffer;

        /**
         * @brief Buffer that the Receiver task being created will use as its stack
         * 
         */
        StackType_t m_au8ReceiverTaskStack[ m_receiverTaskStackSize ];

         /**
         * @brief Buffer that the Publisher task being created will use as its stack
         * 
         */
        StackType_t m_au8PublisherTaskStack[ m_publisherTaskStackSize ];


        /* static methods declarations */

        /**
         * @brief function being called in default event loop task after dispatching Mqtt related event
         * 
         * @param a_pvArgs [IN] void pointer to optional arguments passed to event handler after dispatching of Mqtt event
         * @param a_pcBase [IN] base name of event being the cause of event handler call 
         * @param a_i32EventID [IN] id of event being the cause of event handler call
         * @param a_pvEventData [IN] void pointer to  data passed by the task which created the event 
         */
        static void mqttEventHandler( void * a_pvArgs, esp_event_base_t a_pcBase, int32_t a_i32EventID, void * a_pvEventData );

        /**
         * @brief task which is run after data receving from MQTT broker 
         * 
         * @param a_pvArgs [IN] optional arguments passed to dataReceiverTask
         */
        static void dataReceiverTask( void * a_pvArgs );

        /**
         * @brief task which responsible for publishing data to MQTT broker 
         * 
         * @param a_pvArgs [IN] optional arguments passed to dataReceiverTask
         */
        static void dataPublisherTask( void * a_pvArgs );

    public:
        
        MqttDriver( Interface::IDriverManager & driverManager );
        /**
         * @brief Destructor of MqttDriver class. Destroys the MqttDriver object.
         * 
         * @fn ~MqttDriver
         */

        virtual ~MqttDriver();
        /**
         * @brief Initializes MqttDriver object with necessary configuration to operate as MQTT client.
         * 
         * @fn init()
         * @return execStatus The method execution status: SUCCESS or FAILURE
         */
        execStatus init();

        /**
         * @brief Deinitializes previuosly initialized MqttDriver object.
         * 
         * @details Restores values of the configuration structures and interfaces to the state before initialization
         * with init() method, frees control blocks of MqttDriver object created during initialization process, 
         * disconnects MQTT client from the broker and stops MQTT client and event handler tasks. 
         * @fn deinit
         * @return execStatus The method execution status: SUCCESS or FAILURE
         */
        execStatus deinit();

        /**
         * @brief Runs previuosly initialized MqttDriver.
         * 
         * @details Connects to MQTT mosquitto broker, starts MQTT client task and MQTT event handler task.
         * @fn execStatus start()
         * @return execStatus The method execution status: SUCCESS or FAILURE
         */
        execStatus start();

        /**
         * @brief Stops previously started MqttDriver.
         * 
         * @details Disconnects MQTT client from the broker and stops MQTT client tasks but retains initialized state
         * so it can be started with start() method at any time. 
         * @fn execStatus stop()
         * @return execStatus The method execution status: SUCCESS or FAILURE
         */
        execStatus stop();

        /**
         * @brief Publish data message PDU contained in pduMessage_t structure to MQTT broker. 
         * 
         * @return execStatus  The method execution status: SUCCESS or FAILURE
         */
        execStatus forwardMessage( const pduMessage_t & pduMessage );

        /**
         * @brief Sets the MQTT broker URI and credentials for authentication purposes. 
         * 
         * @fn execStatus setMqttBroker( const char * a_pcBrokerUri, const char * a_pcUsername, const char * a_pcPassword )
         * @param a_pcBrokerUri [IN] URI of the MQTT broker
         * @param a_pcUsername  [IN] username credential for authentication purposes
         * @param a_pcPassword  [IN] password credential for authentication purposes
         * @return execStatus     The method execution status: SUCCESS or FAILURE
         */
        execStatus setMqttBroker( const char * a_pcBrokerUri, const char * a_pcUsername, const char * a_pcPassword );
    };
}

#endif // MQTTDRIVER_HPP