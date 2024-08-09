#ifndef ADC_DRIVER_HPP
#define ADC_DRIVER_HPP

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali_scheme.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>

#include "Interfaces/IDriver.hpp"
#include "Enums/eExecStatus.hpp"
#include "Enums/eDriverState.hpp"

#define ADC_ATTEN_DEFAULT       ADC_ATTEN_DB_0
#define ADC_BITWIDTH_DEFAULT    ADC_BITWIDTH_12


namespace Driver
{

    class ADC_driver : public Interface::IDriver
    {
    public:

        ADC_driver();
        ~ADC_driver();

        execStatus init();
        execStatus deinit();
        execStatus start();
        execStatus stop();

        execStatus setBitwidth( adc_bitwidth_t adc_bitwidth );
        execStatus setAttenuation( adc_atten_t adc_atten );

        execStatus getBitwidth( adc_bitwidth_t & adc_bitwidth );
        execStatus getAttenuation( adc_atten_t & adc_atten );
        execStatus getCaliSchemeHandle( adc_cali_handle_t & a_adc_cali_scheme_handle );



    
    private:

        typedef struct adc_dev
        {
            adc_dev() : adc_unit( ADC_UNIT_1 ), adc_atten( ADC_ATTEN_DEFAULT ), adc_bitwidth( ADC_BITWIDTH_DEFAULT ), adc_cali_scheme_handle( NULL ), adc_continuous_mode_drv_handle( NULL ) {}

            adc_unit_t adc_unit;
            adc_atten_t adc_atten;
            adc_bitwidth_t adc_bitwidth;
            adc_channel_t adc_channel;
            adc_cali_handle_t adc_cali_scheme_handle;
            adc_continuous_handle_t adc_continuous_mode_drv_handle;

        } adc_dev_t;

        /**
         * @brief structure representing ADC device
         */
        adc_dev_t m_adc_device;

        /**
         * @brief enum representing current state of ADC driver
         */
        eDriverState m_adc_driver_state;

        /**
         * @brief ADC data reader task handle
         */
        TaskHandle_t m_adc_data_reader_task_handle;

        /**
         * @brief handle to ring buffer for storing ADC result data shared beetween reader and processor tasks
         */
        RingbufHandle_t m_adc_data_ring_buff_handle;

        /**
         * semaphore used to synchronize adc data reader task
         */
        SemaphoreHandle_t m_adc_data_reader_semphr;
        /**
         * @brief Create supported calibration scheme for ADC unit
         * 
         * @returns enum code execStatus: SUCCESS or FAILURE 
         */
        execStatus createCaliScheme();

        /**
         * @brief Initialize ADC driver for continuous operation mode
         * 
         * @returns enum code execStatus: SUCCESS or FAILURE 
         */
        execStatus adc_continuous_mode_drv_init();


        /* Static functions declarations */

        /**
         * @brief callback function executed after "adc conversion done" event
         */
        static bool IRAM_ATTR conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data );

        /**
         * @brief function executed as FreeRTOS task
         */
        static void adc_data_reader_task( void * pvUserData );

    };

} // Driver namespace

#endif // ADC_DRIVER_HPP