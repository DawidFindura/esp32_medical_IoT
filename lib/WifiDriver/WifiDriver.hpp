#ifndef WIFI_DRIVER_HPP
#define WIFI_DRIVER_HPP

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
        execStatus deinit();
        execStatus start();
        execStatus stop();

    };
};

#endif // WIFI_DRIVER_HPP