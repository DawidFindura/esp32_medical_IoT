#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <stdint.h>

#include "Enums/eExecStatus.hpp"

namespace Logger
{
    class DataLogger
    {
    public:

        DataLogger();
        ~DataLogger();

        execStatus write_out( int data_buffer );
    
    private:

        bool m_isInitialized;
        execStatus init();
    };
};

#endif // DATA_LOGGER_HPP