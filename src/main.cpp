#include <iostream>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "WifiDriver.hpp"
#include "MqttDriver.hpp"
#include "ADC_driver.hpp"
#include "FIR_filter.hpp"
#include "IIR_filer.hpp"
#include "Cascade_filter.hpp"

static Filter::IIR_filter iir_filter_h1( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_h2( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_low( iir_filter_a_coeff_low, iir_filter_b_coeff_low, 5 );
static Filter::IIR_filter iir_filter_stop( iir_filter_a_coeff_stop, iir_filter_b_coeff_stop, 5 );

static const char message1[11] = {'1','2','3','4','5','6','7','8','9','0' , '\0'};
static const char message2[11] = {'1','2','3','4','5','6','7','8','9','0', '\0'};

#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        1000   

static const char * TAG = "ADC_test";


extern "C" void app_main() 
{

    execStatus eStatus = execStatus::FAILURE;

    Filter::Cascade_filter cascade_filter;
    cascade_filter.add_filter( &iir_filter_h1 );
    cascade_filter.add_filter( &iir_filter_h2 );
    cascade_filter.add_filter( &iir_filter_low );
    cascade_filter.add_filter( &iir_filter_stop );

    Driver::ADC_driver adc_device( cascade_filter, false );
    adc_device.init();
    adc_device.start();
    adc_device.stop();

    Driver::WifiDriver wifi_driver;
    Driver::MqttDriver mqtt_driver;
    
    eStatus = wifi_driver.init();
    if( execStatus::SUCCESS == eStatus )
    {
        eStatus = wifi_driver.start();
    }

    if( execStatus::SUCCESS == eStatus )
    {
        eStatus = mqtt_driver.init();
    }
    
    if( execStatus::SUCCESS == eStatus )
    {
        eStatus = mqtt_driver.start();
    }

    if( execStatus::SUCCESS == eStatus )
    {
        eStatus = mqtt_driver.sendDataToDriver( message1 );
    }
    
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE( TAG, "Error while sending message");
    }

    //float iir_out = 0.0f;
    //int i = 0;

    while( true )
    {
        // cascade_filter.calculate_output( ecg_noisy[i], iir_out );
    
        // printf("input");
        // printf("%f", ecg_noisy[i]);
        // printf("cascadeFiltered");
        // printf("%f",(float)iir_out);
        // printf("\r\n");

        // i++;
        // if(i>=(sizeof(ecg_noisy) / sizeof(ecg_noisy[0])))
        // {
        //     i= 0;
        // }


    
        eStatus = mqtt_driver.sendDataToDriver(message2);
      
        

        UBaseType_t t = uxTaskGetStackHighWaterMark(NULL);
        printf(" WATER MARK: %d\n ", t);
        vTaskDelay( ONE_SECOND_DELAY );
    }
} 