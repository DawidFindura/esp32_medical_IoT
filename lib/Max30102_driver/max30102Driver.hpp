#ifndef MAX30102_DRIVER_HPP
#define MAX30102_DRIVER_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>

#include "Interfaces/IDriver.hpp"
#include "Interfaces/IDriverManager.hpp"
#include "Enums/eDriverState.hpp"
#include "max30102_driver.h"

namespace Driver
{
    class Max30102Driver : public Interface::IDriver
    {
    public:

        typedef struct RingBuffItem
        {
            RingBuffItem() : red_led( 0 ), ir_led( 0 ) {}

            uint32_t red_led;
            uint32_t ir_led;
       
        } ring_buff_item_t;
        
        Max30102Driver( Interface::IDriverManager & driverManager );
        ~Max30102Driver();

        execStatus init();
        execStatus deinit();
        execStatus start();
        execStatus stop();
        execStatus forwardMessage( const pduMessage_t & pduMessage );
    
    private:
        
        Interface::IDriverManager & m_driverManager;
        
        const eDriverID m_driverID = eDriverID::MAX30102_DRIVER;
        eDriverState m_driverState;

        max30102_device_t m_max30102Device;

        TaskHandle_t m_dataReaderTaskHandle;

        TaskHandle_t m_dataProcessorTaskHandle;

        RingbufHandle_t m_ring_buff_handle;

        SemaphoreHandle_t m_readerTaskStartSmphrHandle;

        SemaphoreHandle_t m_processorTaskStartSmphrHandle;

        SemaphoreHandle_t m_readerTaskFinishedSmphrHandle;

        SemaphoreHandle_t m_processorTaskFinishedSmphrHandle;

        execStatus gpioIntrInit();
        
        execStatus getInterruptSrc( max30102_device_t & dev, uint8_t & intr_src );

        static void dataReaderTask( void * pvUserData );

        static void dataProcessorTask( void * pvUserData );

        static void IRAM_ATTR max30102_intr_handler( void * pvArgs );


    };
}

#endif // MAX30102_DRIVER_HPP