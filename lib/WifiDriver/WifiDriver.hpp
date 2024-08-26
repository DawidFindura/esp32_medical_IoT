#ifndef WIFIDRIVER_HPP
#define WIFIDRIVER_HPP

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_event.h>
#include <esp_wifi.h>

#include "Interfaces/IDriver.hpp"
#include "Enums/eExecStatus.hpp"
#include "Enums/eDriverState.hpp"


/**
 * @brief namespace grouping all divers in the project.
 * 
 * @namespace Driver
 */
namespace Driver
{
    /**
     * @brief WifiDriver class that implements Wifi driver module. Inherits from IDriver interface.
     * @class WifiDriver
     */
    class WifiDriver : public Interface::IDriver
    {
    private:

        /**
         * @brief Wifi password for Wifi network that driver is trying to connect to
         * 
         * @var const char * m_pcWifiPassword
         */
        const char * m_pcWifiPassword;

        /**
         * @brief name of Wifi network that driver is trying to connect to
         * 
         */
        const char * m_pcWifiSSID;

        /**
         * @brief static variable used to count retrying connection to Wifi network 
         * 
         * @var static uint8_t u8RetryNum
         */
        static uint8_t u8RetryNum;
      
        /**
         * @brief Private member of WifiDriver class. WiFi stack configuration parameters passed to esp_wifi_init call.
         * 
         * @var wifi_init_config_t m_sWifiInitConfig
         */
        wifi_init_config_t m_sWifiInitConfig;

        /**
         * @brief Private member of WifiDriver class.Configuration data for ESP32 AP or STA. The usage of this union (for ap or sta configuration) is determined by the accompanying interface argument passed to esp_wifi_set_config() or esp_wifi_get_config().
         * 
         * @var wifi_config_t m_uWifiConfig
         */
        wifi_config_t m_uWifiConfig;

        /**
         * @brief Private member of WifiDriver class. 
         * Pointer to esp_netif_t structure which is initialized during esp_netif_create_default_wifi_sta() call.
         * Contains default parameters for Wifi Station network interface.
         * 
         * @var esp_netif_t * m_psWifiNetif
         */
        esp_netif_t * m_psWifiNetif;

        /**
         * @brief Private enum definition for class WifiDriver. Defines WifiConnectedBit and WifiFailBit used in FreeRTOS 
         * Event Group during xEventGroupWaitBits finction call.
         * 
         * @enum eWifiBits
         * 
         */
        enum eWifiBits
        {
            WifiConnectedBit = (1 << 0),
            WifiFailBit = (1 << 1)
        };

        /**
         * @brief static and private member of class WifiDriver. Contains current state of Wifi driver.
         * 
         */
        static eDriverState m_eState;

        /**
         * @brief static and private member of class WifiDriver. It is a pointer to structure returned by xEventGroupCreate() function call.
         * 
         * @var static EventGroupHandle_t m_psWifiEventGroup
         */
        static EventGroupHandle_t m_psWifiEventGroup;

        /**
         * @brief static and private method of class WifiDriver. It is a function which will be called after 
         * dispatch of Wifi related events in default event loop.
         * 
         * @fn static void wifiEventHandler( void * a_pvArgs, esp_event_base_t a_pcEventBase, int32_t a_i32EventID, void * a_pvEventData )
         * @param a_pvArgs [IN] void pointer to argument passed to event handler function
         * @param a_pcEventBase [IN] pointer to char event base name of event  
         * @param a_i32EventID [IN] id of event passed to event handler function
         * @param a_pvEventData [IN] void pointer to data passed to event handler function after generation of particular event.
         * @returns void
         */
        static void wifiEventHandler( void * a_pvArgs, esp_event_base_t a_pcEventBase, int32_t a_i32EventID, void * a_pvEventData );
    
    public:

        /**
         * @brief Construct a new Wifi Driver object
         * 
         * @fn  WifiDriver( IDriverManager & a_roIDriverManager, eDriverID a_eDriverID )
         * @param a_roIDriverManager [IN] reference to Driver Manager object
         * @param a_eDriverID [IN] enum describing specific driver 
         */
        WifiDriver();

        /**
         * @brief Destroy the Wifi Driver object
         * 
         * @fn ~WifiDriver()
         */
        virtual ~WifiDriver();

        /**
         * @brief Initializes WifiDriver object. Sets up all neccessary structures and default initializers 
         * for Wifi driver to be ready for start.
         * 
         * @fn  eStatus init()
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */

        execStatus init();

        /**
         * @brief stops and deinitializes WifiDriver object. Clears all previuosly initialized structures and parameters.
         * 
         * @fn deInit()
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */
        execStatus deinit();

        /**
         * @brief Starts the operation of wifi driver
         * 
         * @fn eStatus start()
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */
        execStatus start();

        /**
         * @brief Stops the operation of wifi driver. 
         * 
         * @fn eStatus stop()
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */
        execStatus stop();

        /**
         * @brief method used by Driver Manager module to send data to Wifi driver object.
         * 
         * @fn  eStatus sendDataToDriver( sPdu & a_rsMessage )
         * @param a_rsMessage [IN] reference to sPdu structure containing message to send to driver from driver manager module.
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */
        execStatus sendDataToDriver();

        /**
         * @brief Sets the Wifi Credentials like password and SSID of Wifi network
         * 
         * @param a_pcWifiPassword [IN] pointer to const char being a Wifi password
         * @param a_pcWifiSSID [IN] pointer to const char naming a Wifi SSID
         * @return eStatus enum decribing result of function execution: SUCCESS or FAILURE
         */
        execStatus setWifiCredentials( const char * a_pcWifiPassword, const char * a_pcWifiSSID );
    };
}

#endif // WIFIDRIVER_HPP