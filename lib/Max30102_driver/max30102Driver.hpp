#ifndef MAX30102_DRIVER_HPP
#define MAX30102_DRIVER_HPP

#include "Interfaces/IDriver.hpp"
#include "Interfaces/IDriverManager.hpp"

namespace Driver
{
    class Max30102Driver : public Interface::IDriver
    {
    public:

        Max30102Driver( Interface::IDriverManager & driverManager );
        ~Max30102Driver();

        execStatus init();
        execStatus deinit();
        execStatus start();
        execStatus stop();
        execStatus forwardMessage( const pduMessage_t & pduMessage );
    
    private:

        

    };
}

#endif // MAX30102_DRIVER_HPP