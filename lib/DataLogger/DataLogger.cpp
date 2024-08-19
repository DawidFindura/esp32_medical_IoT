#include <driver/uart.h>
#include <esp_log.h>

#include "DataLogger.hpp"

#define UART_RX_BUFFER_SIZE     1024
#define UART_TX_BUFFER_SIZE     1024

static const char * TAG =  "DataLogger";

using namespace Logger;

DataLogger::DataLogger() : m_isInitialized( false )
{
    execStatus eStatus = this->init();
    if( execStatus::SUCCESS == eStatus )
    {
        m_isInitialized = true;
    }
    else
    {
        ESP_LOGE( TAG, "Failed to init data logger" );
    }
}

DataLogger::~DataLogger()
{

}

execStatus DataLogger::write_out( int data )
{
    execStatus eStatus = execStatus::FAILURE;
    int data_buffer = data;

    if( true == m_isInitialized )
    {
        int ret = uart_write_bytes(UART_NUM_0, &data_buffer, sizeof( data_buffer ) );
        if( ret >= 0 )
        {
            eStatus = execStatus::SUCCESS;
        }
    }
    else
    {
        eStatus = execStatus::NOT_INITIALIZED;
    }

    return eStatus;
}

execStatus DataLogger::init()
{
    execStatus eStatus = execStatus::FAILURE;
    esp_err_t esp_err = ESP_FAIL;

    uart_config_t uart_config = 
            {
                .baud_rate           = 115200,
                .data_bits           = UART_DATA_8_BITS,
                .parity              = UART_PARITY_DISABLE,
                .stop_bits           = UART_STOP_BITS_1,
                .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh = 122,
                .source_clk          = UART_SCLK_DEFAULT
            };

    // Configure UART parameters
    esp_err = uart_param_config(UART_NUM_0, &uart_config);
    if( ESP_OK == esp_err )
    {
        // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
        esp_err = uart_set_pin(UART_NUM_0, 13, 26, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    if( ESP_OK == esp_err )
    {
        esp_err = uart_driver_install(UART_NUM_0, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 0, NULL, 0);
    }
    
    if( ESP_OK == esp_err )
    {
        eStatus = execStatus::SUCCESS;
    }
    
    return eStatus;
}



    
    
    

   