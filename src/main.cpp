#include <WifiDriver.hpp>

extern "C" void app_main() 
{
    Driver::WifiDriver wifiDriver;
    wifiDriver.init();
}