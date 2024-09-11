#ifndef COMMON_HPP
#define COMMON_HPP

#include <stdint.h>
#include <cstring>

#define PDU_MESSAGE_SIZE  (17U) 

#define DRIVER_ID_MASK    (0x0F);
#define DEVICE_ID_MASK    (0xF0);

#define MQTT_AVAILABILITY_PUBLISH_TOPIC     "DEV_AVAILABLE"
#define MQTT_STATE_CHANGE_TOPIC             "STATE_CHANGE"

#define MAX_NUM_OF_COMMANDS    (6U)

#define INVALID_SIGNAL         (0)       

enum class eDataType
{
    NONE = -1,

    CONFIG = 0,
    ECG,
    SATURATION,
    TEMPERATURE,
    HEART_RATE,
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

extern const char * command_list[MAX_NUM_OF_COMMANDS];


typedef struct header
{
    
    uint8_t externalDev : 1;
    uint8_t deviceID    : 3;
    uint8_t driverID    : 4;

} pdu_header_t;

typedef struct pduMessage
{
    pduMessage() { memset( wholeMessage, INVALID_SIGNAL, sizeof( pduMessage ) ); }

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


enum class eECGSignals
{
    ECG_SAMPLE_1_BYTE_1 = 0,
    ECG_SAMPLE_1_BYTE_2,
    ECG_SAMPLE_1_BYTE_3,
    ECG_SAMPLE_1_BYTE_4,

    ECG_SAMPLE_2_BYTE_1,
    ECG_SAMPLE_2_BYTE_2,
    ECG_SAMPLE_2_BYTE_3,
    ECG_SAMPLE_2_BYTE_4,

    ECG_SAMPLE_3_BYTE_1,
    ECG_SAMPLE_3_BYTE_2,
    ECG_SAMPLE_3_BYTE_3,
    ECG_SAMPLE_3_BYTE_4,

    ECG_SAMPLE_4_BYTE_1,
    ECG_SAMPLE_4_BYTE_2,
    ECG_SAMPLE_4_BYTE_3,
    ECG_SAMPLE_4_BYTE_4
};


enum class eMAX30102Signals
{
    HR_SAMPLE_1_BYTE_1 = 0,
    HR_SAMPLE_1_BYTE_2,
    HR_SAMPLE_1_BYTE_3,
    HR_SAMPLE_1_BYTE_4,

    SPO2_SAMPLE_1_BYTE_1,
    SPO2_SAMPLE_1_BYTE_2,
    SPO2_SAMPLE_1_BYTE_3,
    SPO2_SAMPLE_1_BYTE_4,

    TEMP_SAMPLE_1_BYTE_1,
    TEMP_SAMPLE_1_BYTE_2,
    TEMP_SAMPLE_1_BYTE_3,
    TEMP_SAMPLE_1_BYTE_4,

    RESERVED_BYTE_1,
    RESERVED_BYTE_2,
    RESERVED_BYTE_3,
    RESERVED_BYTE_4
};

#endif // COMMON_HPP
