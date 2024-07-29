#ifndef IDRIVER_HPP
#define IDRIVER_HPP

#include "Enums/execStatus.hpp"

namespace Interface
{
    class IDriver
    {    
    public:

        virtual ~IDriver() = default;
        
        /**
         * @brief Initialize driver object
         */
        virtual execStatus init() = 0;

        /**
         * @brief Deinitialize driver object
         */
        virtual execStatus deinit() = 0;

        /**
         * @brief Start execution of driver object task
         */
        virtual execStatus start() = 0;

        /**
         * @brief Stop execution of driver object task
         */
        virtual execStatus stop() = 0;
    
    };
};

#endif // IDRIVER_HPP