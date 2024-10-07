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
#include "common.hpp"
#include "DriverManager.hpp"
#include "StateManager.hpp"
#include "max30102Driver.hpp"


#define DEBUG

static Filter::IIR_filter iir_filter_h1( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_h2( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_low( iir_filter_a_coeff_low_40, iir_filter_b_coeff_low_40, 5 );
static Filter::IIR_filter iir_filter_stop( iir_filter_a_coeff_stop, iir_filter_b_coeff_stop, 5 );

#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        pdMS_TO_TICKS(1000)   

static const char * TAG = "ADC_test";


extern "C" void app_main() 
{

    Service::StateManager stateManager;
    Service::DriverManager driverManager( stateManager, stateManager );
    Driver::WifiDriver wifi_driver( driverManager );
    Driver::MqttDriver mqtt_driver( driverManager );
    
    execStatus eStatus = driverManager.initCommDrivers();
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Comm drivers initialized" );
        eStatus = driverManager.startCommDrivers();
    }
    
    if( execStatus::SUCCESS == eStatus )
    {   
        ESP_LOGI( TAG, "Comm drivers started" );
        eStatus = driverManager.start();   
    }
    
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "Driver manager start its thread" );
    }

    //Driver::Max30102Driver maxDriver( driverManager );  

    Filter::Cascade_filter cascade_filter;
    //cascade_filter.add_filter( &iir_filter_h1 );
    cascade_filter.add_filter( &iir_filter_h2 );
    cascade_filter.add_filter( &iir_filter_low );
    //cascade_filter.add_filter( &iir_filter_stop );

    Driver::ADC_driver adc_device( driverManager, cascade_filter, true );

    // adc_device.init();
    // adc_device.start();
    //adc_device.stop();

      


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


        //driverManager.sendMessage( message1 );
        //driverManager.sendMessage( message2);
        //eStatus = mqtt_driver.forwardMessage(message2);
        //mqtt_driver.forwardMessage(message1);
        //UBaseType_t t = uxTaskGetStackHighWaterMark(NULL);
        //printf(" WATER MARK: %d\n ", t);
        vTaskDelay( pdMS_TO_TICKS(100) );
        //driverManager.sendMessage( message2 );

        //vTaskDelay(pdMS_TO_TICKS(5000));
    }
} 