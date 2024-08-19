#include <iostream>

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
    
    execStatus eStatus = adc_device.init();
    vTaskDelay( pdMS_TO_TICKS(2000) );
    if(eStatus == execStatus::SUCCESS) ESP_LOGI( TAG, "Init finished ok" );
    eStatus = adc_device.start();
    vTaskDelay( pdMS_TO_TICKS(2000) );
    if(eStatus == execStatus::SUCCESS) ESP_LOGI( TAG, "Start finished ok" );
    eStatus = adc_device.stop();
    if(eStatus == execStatus::SUCCESS) ESP_LOGI( TAG, "Stop finished ok" );
    vTaskDelay(pdMS_TO_TICKS(1000));
    eStatus = adc_device.start();
    if(eStatus == execStatus::SUCCESS) ESP_LOGI( TAG, "Start finished ok" );

    while( true )
    {
        //ESP_LOGI( TAG, "adc test" );
        vTaskDelay( pdMS_TO_TICKS( ONE_SECOND_DELAY ) );
    }
} 