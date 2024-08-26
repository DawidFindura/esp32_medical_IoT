#include <iostream>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "WifiDriver.hpp"
#include "ADC_driver.hpp"
#include "FIR_filter.hpp"
#include "IIR_filer.hpp"
#include "Cascade_filter.hpp"
//#include "input_signals.hpp"

static Filter::IIR_filter iir_filter_h1( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_h2( iir_filter_a_coeff_high, iir_filter_b_coeff_high, 3 );
static Filter::IIR_filter iir_filter_low( iir_filter_a_coeff_low, iir_filter_b_coeff_low, 5 );
static Filter::IIR_filter iir_filter_stop( iir_filter_a_coeff_stop, iir_filter_b_coeff_stop, 5 );

#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        1000   

//static const char * TAG = "ADC_test";

//const float input_signal[117]={13.666381,14.604120,14.224701,13.962778,12.086626,12.817649,12.067151,14.258498,12.735096,13.662148,10.648832,11.984905,10.356098,13.180306,11.616559,13.080660,10.254234,11.421075,10.182756,12.695526,12.292816,13.387365,11.879589,11.980140,11.947065,13.205676,14.463734,14.429000,14.526786,13.015139,14.104713,13.695926,16.201518,14.954921,16.112386,13.275491,14.728031,13.151633,15.962105,14.318480,15.638657,12.608370,13.517383,11.974877,14.146060,13.374343,14.083099,12.183636,11.897365,11.492568,12.403904,13.347272,13.037089,12.903623,11.208043,12.162218,11.665882,14.129287,12.881718,14.073850,11.300530,12.837988,11.359809,14.273639,12.730607,14.141312,11.185120,12.146635,10.631315,12.802251,12.002422,12.656487,10.678693,10.294917,9.779240,10.573196,11.400331,10.983140,10.760044,9.000062,9.922254,9.432550,11.946096,10.795520,12.133102,9.553429,11.330562,10.134036,13.365671,12.169172,13.946357,11.366762,12.704448,11.554001,14.067699,13.578172,14.500564,12.740802,12.517947,12.101016,12.928428,13.722677,13.207308,12.823853,10.846390,11.500794,10.701311,12.872596,11.357627,12.319492,9.363647,10.774691,9.231989,12.146137,10.668260,12.206002,9.432943};

extern "C" void app_main() 
{

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
    wifi_driver.init();
    wifi_driver.start();

    float iir_out = 0.0f;
    int i = 0;

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

        vTaskDelay( pdMS_TO_TICKS( 200 ) );
    }
} 