#ifndef DRIVER_MANAGER_HPP
#define DRIVER_MANAGER_HPP

#include <pthread.h>

#include "Queue.hpp"

#include "Interfaces/IDriverManager.hpp"
#include "Interfaces/IDriver.hpp"
#include "Interfaces/IStateManager.hpp"
#include "Interfaces/IStateProvider.hpp"
#include "Enums/eExecStatus.hpp"
#include "Enums/eDriverID.hpp"
#include "common.hpp"
#include "Enums/eDriverState.hpp"

namespace Service
{
    class DriverManager : public Interface::IDriverManager, public Interface::IObserver
    {
    public:

        DriverManager( Interface::IStateManager & stateManager, Interface::IStateProvider & stateProvider );
        ~DriverManager();

        execStatus registerDriver( Interface::IDriver * const driver, const eDriverID driverID );
        execStatus unregisterDriver( Interface::IDriver * const driver, const eDriverID driverID );
        execStatus sendMessage( const pduMessage_t & message );
        
        execStatus registerCommDriver( Interface::IDriver * const driver, const eCommDriverID driverID );
        execStatus unregisterCommDriver( Interface::IDriver * const driver, const eCommDriverID driverID );

        /**
         * @brief initialize the registered communication drivers 
         */
        execStatus initCommDrivers();

        /**
         * @brief start the registerd comm drivers
         */
        execStatus startCommDrivers();

        /* methods for thread routine */

        /**
         * @brief create and start driver manager's thread 
         */
        execStatus start();

        /**
         * @brief stop driver manager's thread
         */
        execStatus stop();

        /**
         * @brief thread routine 
         */
        void work();


        /* method from IObserver interface */
        execStatus update();

    protected:
        execStatus runDrivers();
        execStatus stopDrivers();
        execStatus resetDrivers();
        execStatus changeState( const eDeviceState deviceState );

    private:

        static const uint8_t maxNumOfDrivers = 10;
        static const uint8_t maxNumOfCommDrivers = 5;

        eDeviceState m_deviceState;
        eDriverState m_driversState;

        bool m_threadStarted;
        bool m_threadRunning;

        Interface::IStateManager & m_stateManager;
        Interface::IStateProvider & m_stateProvider;
        
        Interface::IDriver * m_registeredDrivers[ maxNumOfDrivers ];
        Interface::IDriver * m_registeredCommDrivers[ maxNumOfCommDrivers ];

        
        Utils::CQueue<pduMessage_t> m_messageQueue;

        static void * driverManagerThread( void * pvArgs );

        pthread_mutex_t m_threadMutex;
        pthread_cond_t m_startCondVar;

        pthread_t m_thread;  

    };
}

#endif // DRIVER_MANAGER_HPP