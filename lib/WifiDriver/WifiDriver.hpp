#ifndef WIFI_DRIVER
#define WIFI_DRIVER

#include "Interfaces/IDriver.hpp"
#include "Enums/execStatus.hpp"

namespace Driver
{
    class WifiDriver : public Interface::IDriver
    {
    public:
        WifiDriver();
        ~WifiDriver();

        execStatus init();
        execStatus start();
        execStatus stop();

    };
};

#endif // WIFI_DRIVER