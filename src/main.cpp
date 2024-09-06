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
#include "max30102_driver.h"


#define DEBUG

static Filter::IIR_filter iir_filter_h1( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_h2( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_low( iir_filter_a_coeff_low, iir_filter_b_coeff_low, 5 );
static Filter::IIR_filter iir_filter_stop( iir_filter_a_coeff_stop, iir_filter_b_coeff_stop, 5 );

#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        pdMS_TO_TICKS(1000)   

static const char * TAG = "ADC_test";


extern "C" void app_main() 
{

    // Service::StateManager stateManager;
    // Service::DriverManager driverManager( stateManager, stateManager );
    // driverManager.start();
    // pduMessage_t message1, message2;
    
    // message1.header.driverID = (uint8_t)eDriverID::AD8232_DRIVER;
    // message1.header.deviceID = 5;
    // message1.header.externalDev = 0;

    // char mess1[15] = "Hello ad8232_d";
    // char mess2[15] = "Hello max30102";

    // memcpy(message1.signals,mess1,15);
    // memcpy(message2.signals,mess2,15);

    // message2.header.deviceID = 4;
    // message2.header.driverID = (uint8_t)eDriverID::MAX30102_DRIVER;
    // message2.header.externalDev = 1; 

    // execStatus eStatus = execStatus::FAILURE;

    // Filter::Cascade_filter cascade_filter;
    // cascade_filter.add_filter( &iir_filter_h1 );
    // cascade_filter.add_filter( &iir_filter_h2 );
    // cascade_filter.add_filter( &iir_filter_low );
    // cascade_filter.add_filter( &iir_filter_stop );

    // Driver::ADC_driver adc_device( driverManager, cascade_filter, false );
    // //adc_device.init();
    // //adc_device.start();
    // // adc_device.stop();

    // Driver::WifiDriver wifi_driver( driverManager );
    // Driver::MqttDriver mqtt_driver( driverManager );
    
    // eStatus = wifi_driver.init();
    // if( execStatus::SUCCESS == eStatus )
    // {
    //     eStatus = wifi_driver.start();
    // }

    // if( execStatus::SUCCESS == eStatus )
    // {
    //     eStatus = mqtt_driver.init();
    // }
    
    // if( execStatus::SUCCESS == eStatus )
    // {
    //     eStatus = mqtt_driver.start();
    // }

    // if( execStatus::SUCCESS == eStatus )
    // {
    //     eStatus = mqtt_driver.forwardMessage( message1 );
    // }
    
    // if( execStatus::SUCCESS != eStatus )
    // {
    //     ESP_LOGE( TAG, "Error while sending message");
    // }

    //default configuration of max30102 device
    max30102_device_t max30102_device;
    max30102_default_config_init(&max30102_device);

    //initialization of max30102 device
    esp_err_t error = max30102_device_init(&max30102_device);

    if(error != ESP_OK){
        ESP_LOGE(TAG, "max30102 device initialization failed!\r\n");
        while(1);
    }    
    
   max30102_device.registers.intr_enable_1_reg.ppg_rdy_en_bit = 1;
   max30102_generic_register_t * reg = reinterpret_cast<max30102_generic_register_t *>(&max30102_device.registers.intr_enable_1_reg);
   max30102_set_register( reg );
   max30102_start_data_acquisition(&max30102_device);
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


        //driverManager.sendMessage( message1 );
        //driverManager.sendMessage( message2);
        //eStatus = mqtt_driver.forwardMessage(message2);
        //mqtt_driver.forwardMessage(message1);
        //UBaseType_t t = uxTaskGetStackHighWaterMark(NULL);
        //printf(" WATER MARK: %d\n ", t);
        vTaskDelay( pdMS_TO_TICKS(100) );
        //driverManager.sendMessage( message2 );

        //vTaskDelay(pdMS_TO_TICKS(5000));
       
       max30102_get_die_temp( &max30102_device );
    }
} 