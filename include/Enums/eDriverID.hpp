#ifndef EDRIVER_ID_HPP
#define EDRIVER_ID_HPP

/* enum ID's for data producing drivers */
enum class eDriverID
{
    NONE = -1,

    AD8232_DRIVER = 0,
    MAX30102_DRIVER,
    RTC_DRIVER,
    DHT_DRIVER
};

/* enum ID's for communication drivers */
enum class eCommDriverID
{
    NONE = -1,
    
    MQTT_DRIVER,
    WIFI_DRIVER
};

#endif // EDRIVER_ID_HPP