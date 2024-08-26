#include <string.h>

#include <esp_log.h>
#include <nvs_flash.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <esp_system.h>

#include "WifiDriver.hpp"
#include "WifiCredentials.hpp"


static const char * TAG = "WIFI TEST";

/**
 * @brief defines how many tries WifiDriver should take to connect Wifi network if fails for the first time.
 * 
 * @def MAX_RETRY_NUM
 */
#define MAX_RETRY_NUM 5


using namespace Driver;

WifiDriver::WifiDriver() : 
        m_u8RetryNum( 0 ),
        m_eState( eDriverState::UNINITIALIZED ),
        m_psWifiEventGroup( NULL ), 
        m_sWifiInitConfig(), 
        m_uWifiConfig(), 
        m_psWifiNetif( NULL )
{
    m_pcWifiPassword = defaultWifiPassword;
    m_pcWifiSSID = defaultWifiSSID;
    memcpy( m_uWifiConfig.sta.ssid, defaultWifiSSID, strlen( defaultWifiSSID ) );
    memcpy( m_uWifiConfig.sta.password, defaultWifiPassword, strlen( defaultWifiPassword ) );
}

WifiDriver::~WifiDriver()
{
    execStatus eStatus = execStatus::FAILURE;
    
    eStatus = this->deinit();
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE(TAG, "Deinit unsuccessful");
    }
}

execStatus WifiDriver::init()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;
   
    //Initialize NVS - non volatile storage for Wifi relevant data
    esp_err = nvs_flash_init();
    if( ESP_ERR_NVS_NO_FREE_PAGES == esp_err || ESP_ERR_NVS_NEW_VERSION_FOUND == esp_err ) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        esp_err = nvs_flash_init();
    }
    //checking potential error status returned by nvs_flash_init()
    ESP_ERROR_CHECK(esp_err);

    if( ESP_OK == esp_err )
    {
        esp_err = esp_netif_init();
        ESP_LOGI( "WIFI_TEST", "WIFI nvs_flash init SUCCESSFUL" );
    }
    
    if( ESP_OK == esp_err )
    {
        m_psWifiEventGroup = xEventGroupCreate();
        if( NULL != m_psWifiEventGroup )
        {
            esp_err = ESP_OK;
        }
        else
        {
            esp_err = ESP_FAIL;
        }
    }

    if( ESP_OK == esp_err)
    {
        esp_err = esp_event_loop_create_default();
        ESP_LOGI("WIFI_TEST","WIFI event group create SUCCESSFUL");
    }
    
    if( ESP_OK == esp_err )
    {
        m_psWifiNetif = esp_netif_create_default_wifi_sta();
        ESP_LOGI("WIFI_TEST","WIFI event loop SUCCESSFUL");
    }

    if( NULL == m_psWifiNetif )
    {
        esp_err = ESP_FAIL;
        ESP_LOGI("WIFI_TEST","WIFI netif create default wifi station unSUCCESSFUL");
    }

    if( ESP_OK == esp_err )
    {
        m_sWifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
        esp_err = esp_wifi_init( &m_sWifiInitConfig );
        ESP_LOGI("WIFI_TEST","WIFI netif create default wifi station SUCCESSFUL");
    }

    if( ESP_OK == esp_err )
    {
        /* as last argument passing pointer to this wifi driver object instance */
        esp_err = esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, (void *)this );
    }

    if( ESP_OK == esp_err )
    {
        /* as last argument passing pointer to this wifi driver object instance */
        esp_err = esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, (void *)this );
    }
    
    if( ESP_OK == esp_err )
    {
        esp_err = esp_wifi_set_mode( WIFI_MODE_STA );
        ESP_LOGI( "WIFI_TEST", "IP event handler register SUCCESSFUL" );
    }

    if( ESP_OK == esp_err )
    {
        //check if m_uWifiConfig was already initialized before
        if( 0 == m_uWifiConfig.sta.ssid[0] )
        {
            memcpy( m_uWifiConfig.sta.ssid, m_pcWifiSSID, strlen( m_pcWifiSSID ) );
            memcpy( m_uWifiConfig.sta.password, m_pcWifiPassword, strlen( m_pcWifiPassword ) );
        }

        /*search for all available wifi networks*/
        m_uWifiConfig.sta.threshold.authmode = WIFI_AUTH_OPEN;
        esp_err = esp_wifi_set_config( WIFI_IF_STA, &m_uWifiConfig );
    }
    
    if( ESP_OK == esp_err )
    {
        ESP_LOGI("WIFI_TEST","WIFI INIT SUCCESSFUL");
        m_eState = eDriverState::INITIALIZED;
        ret = execStatus::SUCCESS;
    }
    
    return ret;
}

execStatus WifiDriver::deinit()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;
    
    m_eState = eDriverState::STOPPED;
    esp_err = esp_wifi_stop();
    if( ESP_OK == esp_err )
    {
        esp_err = esp_event_handler_unregister( WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler );
    }

    if( ESP_OK == esp_err )
    {
        esp_err = esp_event_handler_unregister( IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler );
    }
    
    if( ESP_OK == esp_err )
    {
        esp_err = esp_event_loop_delete_default();
    }

    
    if( ESP_OK == esp_err )
    {
        esp_netif_destroy_default_wifi( m_psWifiNetif );
        m_psWifiNetif = NULL;
        esp_err = esp_wifi_deinit();
    }

    if( ESP_OK == esp_err )
    {
        esp_err = nvs_flash_deinit();
    }

    if( ESP_OK == esp_err )
    {
        //clears previously initialized structures 
        memset( &m_uWifiConfig, 0, sizeof( m_uWifiConfig ) );
        memset( &m_sWifiInitConfig, 0, sizeof( m_sWifiInitConfig ) );
        vEventGroupDelete( m_psWifiEventGroup );
        m_psWifiEventGroup = NULL;
        
        ESP_LOGI("WIFI_TEST","WIFI DEINIT SUCCESSFUL");
        m_eState = eDriverState::DEINITIALIZED;
        ret = execStatus::SUCCESS;
        
    }

    return ret;
}
    
execStatus WifiDriver::start()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;
    
    if( ( eDriverState::INITIALIZED == m_eState ) ||  ( eDriverState::STOPPED == m_eState ) )
    {
        m_eState = eDriverState::STARTED;
        esp_err = esp_wifi_start();
    }
   
    if( ESP_OK == esp_err )
    {
        ret = execStatus::SUCCESS;
        ESP_LOGI("WIFI_TEST","WIFI START SUCCESSFUL");    
    }

    /* waits for specific bits to be set in event handler and returns the bits before the call returned */
    EventBits_t bits = xEventGroupWaitBits
                        (
                            m_psWifiEventGroup, 
                            WifiConnectedBit | WifiFailBit,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY
                        );

    /* test which event actually happened. */
    if( 0 != ( bits & WifiConnectedBit ) ) 
    {
        ESP_LOGI( TAG, "connected to ap SSID:%s password:%s", m_uWifiConfig.sta.ssid, m_uWifiConfig.sta.password );
    } 
    else if( 0 != ( bits & WifiFailBit ) ) 
    {
        ESP_LOGI( TAG, "Failed to connect to SSID:%s, password:%s", m_uWifiConfig.sta.ssid, m_uWifiConfig.sta.password );
    }   
    else 
    {
        ESP_LOGE( TAG, "UNEXPECTED EVENT" );
    }

    return ret;
}

execStatus WifiDriver::stop()
{
    execStatus ret = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;
    
    m_eState = eDriverState::STOPPED;
    esp_err = esp_wifi_stop();
    if( ESP_OK == esp_err )
    {
        ret = execStatus::SUCCESS;
        ESP_LOGI("WIFI_TEST","WIFI STOP SUCCESSFUL");
    }

    return ret;
}


execStatus WifiDriver::setWifiCredentials( const char * a_pcWifiPassword, const char * a_pcWifiSSID )
{
    execStatus ret = execStatus::FAILURE;
    uint8_t u8SsidSize = strlen( a_pcWifiSSID );
    uint8_t u8PasswordSize = strlen( a_pcWifiPassword );
    
    if( u8SsidSize <= sizeof(m_uWifiConfig.sta.ssid) && u8PasswordSize <= sizeof(m_uWifiConfig.sta.password) )
    {
        memcpy( m_uWifiConfig.sta.ssid, a_pcWifiSSID, strlen( a_pcWifiSSID ) );
        memcpy( m_uWifiConfig.sta.password, a_pcWifiPassword, strlen( a_pcWifiPassword ) );
        m_pcWifiPassword = a_pcWifiPassword;
        m_pcWifiSSID = a_pcWifiSSID;

        execStatus ret = execStatus::SUCCESS;
        ESP_LOGI("WIFI_TEST","WIFI SET CREDENTIALS SUCCESSFUL");
    }
    
    return ret;
}

void WifiDriver::wifiEventHandler( void * a_pvArgs, esp_event_base_t a_pcEventBase, int32_t a_i32EventID, void * a_pvEventData )
{
    WifiDriver * wifi_driver = static_cast<WifiDriver *>(a_pvArgs);

    if(  WIFI_EVENT == a_pcEventBase )
    {
        wifi_event_t eWifiEvent = static_cast<wifi_event_t>(a_i32EventID);

        /* check what kind of event id was the cause of event handler call */
        switch( eWifiEvent )
        {
            case WIFI_EVENT_STA_START:
            {
                ESP_LOGI(TAG, "WIFI EVENT STA START");
                ESP_ERROR_CHECK(esp_wifi_connect());
                break;
            }

            case WIFI_EVENT_STA_DISCONNECTED:
            {
                if( ( wifi_driver->m_u8RetryNum < MAX_RETRY_NUM ) && ( eDriverState::STARTED == wifi_driver->m_eState ) )
                {
                    ESP_ERROR_CHECK(esp_wifi_connect());
                    ESP_LOGI(TAG, "trying to connect:%d", wifi_driver->m_u8RetryNum);
                    wifi_driver->m_u8RetryNum++;
                }
                else
                {
                    xEventGroupSetBits( wifi_driver->m_psWifiEventGroup, WifiFailBit );
                }

                break;
            }
        
            case WIFI_EVENT_STA_STOP:
            {
                ESP_LOGI( TAG, "STA STOP EVENT" );
                if ( NULL != wifi_driver->m_psWifiEventGroup )
                {
                    xEventGroupClearBits( wifi_driver->m_psWifiEventGroup, WifiConnectedBit | WifiFailBit );
                }

                break;
            }

            default:
            {
                break;
            }
        
        }
    }
    else if( IP_EVENT == a_pcEventBase )
    {
        ip_event_t eIpEvent = static_cast<ip_event_t>(a_i32EventID);

        switch( eIpEvent )
        {
            case IP_EVENT_STA_GOT_IP:
            {
                ip_event_got_ip_t * psGotIpEventData = static_cast<ip_event_got_ip_t *>(a_pvEventData);

                ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR( &psGotIpEventData->ip_info.ip ) );
                xEventGroupSetBits( wifi_driver->m_psWifiEventGroup, WifiConnectedBit );
                wifi_driver->m_u8RetryNum = 0;
                break;
            }

            default:
            {
                break;
            }
        }
    }
}