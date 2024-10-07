#include "common.hpp"

const char * command_list[MAX_NUM_OF_COMMANDS]
{
    "START",
    "STOP",
    "CLOSE",
    "RESET",
    "FILTER_ON",
    "FILTER_OFF",
};

/* topic names to publish via MQTT. Must follow indexing deifined in eDriverID.hpp file */
const char * MQTT_topic_names[] = {"AD8232_DRIVER", "MAX30102_DRIVER", "RTC_DRIVER", "DHT_DRIVER"};