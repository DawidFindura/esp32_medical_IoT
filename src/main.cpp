#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "WifiDriver.hpp"
#include "ADC_driver.hpp"
#include "FIR_filter.hpp"


#define _VAL_TO_STRING( val )   #val
#define VAL_TO_STRING( val )    _VAL_TO_STRING( val )

#define ONE_SECOND_DELAY        1000   

//static const char * TAG = "ADC_test";

static const float fir_filter_coeff[5] = { 0.2, 0.2, 0.2, 0.2, 0.2 };
static const float input_signal[100]={0.203681,0.163024,-0.094846,0.039093,-0.093058,-0.287648,-0.302038,-0.293075,-0.246820,-0.299419,-0.553505,-0.400139,-0.450787,-0.613248,-0.576076,-0.779104,-0.744285,-0.652519,-0.711580,-0.694275,-0.790967,-0.962884,-0.772525,-0.760340,-0.829184,-0.810439,-0.811072,-0.891765,-0.814933,-0.921045,-0.768489,-0.914396,-0.826763,-0.854483,-0.808287,-0.589897,-0.582042,-0.633419,-0.429213,-0.609547,-0.457374,-0.418288,-0.266847,-0.202131,-0.295302,-0.159291,-0.108914,0.003577,0.082285,0.156944,0.100734,0.264982,0.321776,0.260963,0.311482,0.466611,0.640867,0.543323,0.659994,0.623013,0.805976,0.730543,0.839183,0.930519,1.018488,1.072393,1.002829,0.930650,0.959678,1.009378,1.174021,1.042373,1.193393,1.057736,1.232190,1.086363,1.042987,1.047579,1.125823,1.073224,1.022063,1.117339,1.027769,0.987156,1.043874,0.847606,0.923892,0.878511,0.737899,0.734863,0.559604,0.499684,0.562494,0.566454,0.545536,0.283625,0.331457,0.243940,0.066399,0.084281};

extern "C" void app_main() 
{
    Driver::ADC_driver adc_device;
    adc_device.init();
    Filter::FIR_filter fir_filter( fir_filter_coeff, 5);
    float out = 0.0f;
    int i = 0;
    while( true )
    {
        fir_filter.calculate_output( input_signal[i], out );
        //printf("Filter output[%d]: %f \n",i, out );
        printf("input");
        printf("%f", input_signal[i]);
        printf("filtered");
        printf("%f", (float)out);
        printf("\r\n");
        i++;
        if(i>=100)
        {
            i= 0;
        }
        vTaskDelay( pdMS_TO_TICKS( ONE_SECOND_DELAY ) );
    }
} 