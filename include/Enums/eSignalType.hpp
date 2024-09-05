#ifndef ESIGNAL_TYPE_HPP
#define ESIGNAL_TYPE_HPP

enum class eSignalType
{
    NONE = -1,
    CONFIG = 0,
    ECG,
    SATURATION,
    HUMIDITY,
    TEMPERATURE
};

#endif // ESIGNAL_TYPE_HPP