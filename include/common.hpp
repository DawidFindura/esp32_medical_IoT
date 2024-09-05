#ifndef COMMON_HPP
#define COMMON_HPP

#include <stdint.h>
#include <cstring>

#define PDU_MESSAGE_SIZE  (17U) 

#define DRIVER_ID_MASK    (0x0F);
#define DEVICE_ID_MASK    (0xF0);

#define MQTT_AVAILABILITY_PUBLISH_TOPIC     "DEV_AVAILABLE"
#define MQTT_STATE_CHANGE_TOPIC             "STATE_CHANGE"

#define MAX_NUM_OF_COMMANDS    (5U)

enum class eDataType
{
    NONE = -1,
    CONFIG = 0,
    ECG,
    SATURATION,
    HUMIDITY,
    TEMPERATURE
};

enum class eDeviceState
{
    RUN,
    STOP,
    SLEEP,
    RESET,
    FILTER_ON,
    FILTER_OFF
};

static const char * command_list[] = 
{
    "START",
    "STOP",
    "CLOSE",
    "RESET",
    "FILTER_ON",
    "FILTER_OFF",
};

typedef struct header
{
    
    uint8_t externalDev : 1;
    uint8_t deviceID    : 3;
    uint8_t driverID    : 4;

} pdu_header_t;

typedef struct pduMessage
{
    pduMessage() { memset( wholeMessage, 0, sizeof( pduMessage ) ); }

    union
    {
        uint8_t wholeMessage[PDU_MESSAGE_SIZE];
        
        struct 
        {
            pdu_header_t header;
            uint8_t signals[PDU_MESSAGE_SIZE - 1];
        };
        

    };
    
} pduMessage_t;


#endif // COMMON_HPP
