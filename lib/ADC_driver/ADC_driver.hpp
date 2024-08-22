#ifndef ADC_DRIVER_HPP
#define ADC_DRIVER_HPP

#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali_scheme.h>

#include "Interfaces/IDriver.hpp"
#include "Enums/execStatus.hpp"

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

        typedef enum multisampling_mode
        {
            MULTISAMPLING_MODE_NONE,
            MULTISAMPLING_MODE_8,
            MULTISAMPLING_MODE_16,
            MULTISAMPLING_MODE_32,
            MULTISAMPLING_MODE_64

        } multisampling_mode_t;

    
    private:

        typedef struct adc_dev
        {
            adc_dev() : adc_unit( ADC_UNIT_1 ), adc_atten( ADC_ATTEN_DEFAULT ), adc_bitwidth( ADC_BITWIDTH_DEFAULT ), adc_cali_scheme_handle( NULL ), adc_continuous_mode_drv_handle( NULL ), multisampling_mode( MULTISAMPLING_MODE_NONE ) {}

            adc_unit_t adc_unit;
            adc_atten_t adc_atten;
            adc_bitwidth_t adc_bitwidth;
            adc_channel_t adc_channel;
            adc_cali_handle_t adc_cali_scheme_handle;
            adc_continuous_handle_t adc_continuous_mode_drv_handle;
            multisampling_mode_t multisampling_mode;

        } adc_dev_t;

        adc_dev_t m_adc_device;

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

    };

} // Driver namespace

#endif // ADC_DRIVER_HPP