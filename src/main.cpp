#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "WifiDriver.hpp"
#include "ADC_driver.hpp"


#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        1000   

static const char * TAG = "ADC_test";

extern "C" void app_main() 
{
    Driver::ADC_driver adc_device;
    adc_device.init();
    

    while( true )
    {
        vTaskDelay( pdMS_TO_TICKS( ONE_SECOND_DELAY ) );
    }
} 