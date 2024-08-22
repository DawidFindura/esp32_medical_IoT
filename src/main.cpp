#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "WifiDriver.hpp"
#include "ADC_driver.hpp"
#include "FIR_filter.hpp"
#include "IIR_filer.hpp"


#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        1000   

//static const char * TAG = "ADC_test";

static const float fir_filter_coeff[5] = { 0.2, 0.2, 0.2, 0.2, 0.2 };
static const float iir_filter_a_coeff[5]={1.000000,-3.971566,5.915101,-3.915501,0.971966};
static const float iir_filter_b_coeff[5]={0.985883,-3.943533,5.915300,-3.943533,0.985883};
static const float input_signal[50]={92.000000,99.000000,1.000000,8.000000,15.000000,67.000000,74.000000,51.000000,58.000000,40.000000,92.000000,99.000000,1.000000,8.000000,15.000000,67.000000,74.000000,51.000000,58.000000,40.000000,92.000000,99.000000,1.000000,8.000000,15.000000,67.000000,74.000000,51.000000,58.000000,40.000000,92.000000,99.000000,1.000000,8.000000,15.000000,67.000000,74.000000,51.000000,58.000000,40.000000,92.000000,99.000000,1.000000,8.000000,15.000000,67.000000,74.000000,51.000000,58.000000,40.000000};

extern "C" void app_main() 
{
    Driver::ADC_driver adc_device;
    adc_device.init();
    Filter::FIR_filter fir_filter( fir_filter_coeff, 5);
    Filter::IIR_filter iir_filter( iir_filter_a_coeff, iir_filter_b_coeff, 5 );
    float fir_out = 0.0f;
    float iir_out = 0.0f;
    int i = 0;

    while( true )
    {
        fir_filter.calculate_output( input_signal[i], fir_out );
        iir_filter.calculate_output( input_signal[i], iir_out );
    
        printf("input");
        printf("%f", input_signal[i]);
        printf("firFiltered");
        printf("%f", (float)fir_out);
        printf("iirFiltered");
        printf("%f",(float)iir_out);
        printf("\r\n");

        i++;
        if(i>=(sizeof(input_signal) / sizeof(input_signal[0])))
        {
            i= 0;
        }

        vTaskDelay( pdMS_TO_TICKS( ONE_SECOND_DELAY ) );
    }
} 