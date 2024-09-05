#include <esp_log.h>

#include "DriverManager.hpp"

static const char * TAG = "DriverManager";

using namespace Service;

DriverManager::DriverManager( Interface::IStateManager & stateManager, Interface::IStateProvider & stateProvider ) : 
                                m_deviceState(eDeviceState::SLEEP), 
                                m_driversState(eDriverState::UNINITIALIZED), 
                                m_threadStarted( false ), 
                                m_threadRunning( false ),
                                m_stateManager( stateManager ),
                                m_stateProvider( stateProvider )
{
    for( int driverIndex = 0; driverIndex < maxNumOfDrivers; driverIndex++ )
    {
        m_registeredDrivers[driverIndex] = nullptr;
    }

    pthread_cond_init( &m_startCondVar, NULL );
    pthread_mutex_init( &m_threadMutex, NULL );

    execStatus eStatus = m_stateProvider.registerObserver( this );
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE( TAG, "Failed to register in state provider ");
    }
}

DriverManager::~DriverManager()
{
    this->stop();
    
    pthread_mutex_destroy( &m_threadMutex );
    pthread_cond_destroy( &m_startCondVar );

    execStatus eStatus = m_stateProvider.unregisterObserver( this );
    if( execStatus::SUCCESS != eStatus )
    {
        ESP_LOGE( TAG, "Failed to unregister from state provider ");
    }
}

execStatus DriverManager::registerDriver( Interface::IDriver * const driver, const eDriverID driverID )
{
    execStatus eStatus = execStatus::FAILURE;
    
    if( (int)driverID < maxNumOfDrivers )
    {
        if( nullptr != driver )
        {
            m_registeredDrivers[ (int)driverID ] = driver;
            eStatus = execStatus::SUCCESS;
        }
        else
        {
            eStatus = execStatus::NULL_POINTER;
        }
    }

    return eStatus;
}

execStatus DriverManager::unregisterDriver( Interface::IDriver * const driver, const eDriverID driverID )
{
    execStatus eStatus = execStatus::FAILURE;

    if( (int)driverID < maxNumOfDrivers )
    {
        if( m_registeredDrivers[ (int)driverID ] == driver )
        {
            m_registeredDrivers[ (int)driverID ] = nullptr;
            eStatus = execStatus::SUCCESS;
        }
        else
        {
            eStatus = execStatus::NULL_POINTER;
        }
    }

    return eStatus;
}


execStatus DriverManager::registerCommDriver( Interface::IDriver * const driver, const eCommDriverID CommDriverID )
{
    execStatus eStatus = execStatus::FAILURE;
    
    if( (int)CommDriverID < maxNumOfCommDrivers )
    {
        if( nullptr != driver )
        {
            m_registeredCommDrivers[ (int)CommDriverID ] = driver;
            eStatus = execStatus::SUCCESS;
        }
        else
        {
            eStatus = execStatus::NULL_POINTER;
        }
    }

    return eStatus;
}

execStatus DriverManager::unregisterCommDriver( Interface::IDriver * const driver, const eCommDriverID commDriverID )
{
    execStatus eStatus = execStatus::FAILURE;

    if( (int)commDriverID < maxNumOfCommDrivers )
    {
        if( m_registeredDrivers[ (int)commDriverID ] == driver )
        {
            m_registeredDrivers[ (int)commDriverID ] = nullptr;
            eStatus = execStatus::SUCCESS;
        }
        else
        {
            eStatus = execStatus::NULL_POINTER;
        }
    }

    return eStatus;
}

execStatus DriverManager::sendMessage( const pduMessage_t & message )
{
    execStatus eStatus = execStatus::SUCCESS;
    ESP_LOGI( TAG, "In driver manager. Sending the message into the queue" );
    m_messageQueue.push( message );

    return eStatus;
}

execStatus DriverManager::runDrivers()
{
    execStatus eStatus = execStatus::FAILURE;

    ESP_LOGI( TAG, "In runDrivers. Current state: %d and device state: %d ", (int)m_driversState, (int)m_deviceState );

    for( int driverIndex = 0; driverIndex < maxNumOfDrivers; driverIndex++ )
    {
        if( nullptr == m_registeredDrivers[ driverIndex ] )
        {
            continue;
        }

        if( ( m_driversState == eDriverState::DEINITIALIZED ) || ( m_driversState == eDriverState::UNINITIALIZED ) )
        {
            ESP_LOGI( TAG, "trying to init registered drivers" );
            eStatus = m_registeredDrivers[ driverIndex ]->init();
            if( execStatus::SUCCESS != eStatus )
            {
                return eStatus;
            }

            ESP_LOGI( TAG, "Successfully initialized driver: %d", driverIndex );
            m_driversState = eDriverState::INITIALIZED;
        }
        
        if( ( eDriverState::INITIALIZED == m_driversState ) || ( eDriverState::STOPPED == m_driversState ) )
        {
            ESP_LOGI( TAG, "trying to start registered drivers" );
            eStatus = m_registeredDrivers[ driverIndex ]->start();
            if( execStatus::SUCCESS != eStatus )
            {
                return eStatus;
            }

            ESP_LOGI( TAG, "Successfully started driver: %d", driverIndex );
            m_driversState = eDriverState::STARTED;
        }
       
    }

    return eStatus = execStatus::SUCCESS;
}

execStatus DriverManager::stopDrivers()
{
    execStatus eStatus = execStatus::FAILURE;

    ESP_LOGI( TAG, "In stopDrivers. Current state: %d and device state: %d ", (int)m_driversState, (int)m_deviceState );

    for( int driverIndex = 0; driverIndex < maxNumOfDrivers; driverIndex++)
    {
        if( nullptr == m_registeredDrivers[ driverIndex ] )
        {
            continue;
        }

        if( m_driversState == eDriverState::STARTED )
        {
            eStatus = m_registeredDrivers[ driverIndex ]->stop();
            if( execStatus::SUCCESS != eStatus )
            {
                return eStatus;
            }

            m_driversState = eDriverState::STOPPED;
        }
    }

    return eStatus = execStatus::SUCCESS;
}

execStatus DriverManager::resetDrivers()
{
    execStatus eStatus = execStatus::FAILURE;

    ESP_LOGI( TAG, "In resetDrivers. Current state: %d and device state: %d ", (int)m_driversState, (int)m_deviceState );

    for( int driverIndex = 0; driverIndex < maxNumOfDrivers; driverIndex++)
    {
        if( nullptr == m_registeredDrivers[ driverIndex ] )
        {
            continue;
        }

        eStatus = m_registeredDrivers[ driverIndex ]->deinit();
        if( execStatus::SUCCESS != eStatus )
        {
            return eStatus;
        }

        eStatus = m_registeredDrivers[ driverIndex ]->init();
        if( execStatus::SUCCESS != eStatus )
        {
            return eStatus;
        }

        m_driversState = eDriverState::INITIALIZED;
    }

    return eStatus = execStatus::SUCCESS;
}

execStatus DriverManager::changeState( const eDeviceState deviceState )
{
    execStatus eStatus = execStatus::FAILURE;

    m_deviceState = deviceState;
    
    int stateID = static_cast<int>(deviceState);

    switch( stateID )
    {
        case (int)eDeviceState::SLEEP:
            eStatus = stopDrivers();
            ESP_LOGI( TAG, "SLEEP ");
            break;
        
        case (int)eDeviceState::RESET:
            eStatus = resetDrivers();
            ESP_LOGI( TAG, "RESET");
            break;
        
        case (int)eDeviceState::RUN:
            ESP_LOGI( TAG, "Successful change state");
            eStatus = runDrivers();
            ESP_LOGI( TAG, "RUN");
            break;
        
        case (int)eDeviceState::STOP:
            eStatus = stopDrivers();
            ESP_LOGI( TAG, "STOP");
            break;
        
        default:
            ESP_LOGI( TAG, "UNKONOWN STATE");
            break;
    }

    return eStatus;
}

void * DriverManager::driverManagerThread(void * pvArgs)
{
    DriverManager * driverManager = static_cast<DriverManager*>(pvArgs);

    pthread_mutex_lock( &driverManager->m_threadMutex );
    while( false == driverManager->m_threadRunning )
    {
        pthread_cond_wait(&driverManager->m_startCondVar, &driverManager->m_threadMutex);
    }
    pthread_mutex_unlock(&driverManager->m_threadMutex);

    while( true == driverManager->m_threadRunning )
    {
        driverManager->work();

        /* minimal blocking time */
        usleep(1000); // 1 ms in blocked state
    }

    return NULL;
}


execStatus DriverManager::start()
{
    execStatus eStatus = execStatus::FAILURE;
    
    if( false == m_threadRunning )
    {
        pthread_create( &m_thread, NULL, DriverManager::driverManagerThread, this );
        m_threadRunning = true;
        eStatus = execStatus::SUCCESS;
    }
    
    pthread_cond_signal( &m_startCondVar );

    return eStatus;
}

execStatus DriverManager::stop()
{
    execStatus eStatus = execStatus::FAILURE;

    if( true == m_threadRunning )
    {
        pthread_join(m_thread, NULL);
        m_threadRunning = false;
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}

void DriverManager::work()
{
    execStatus eStatus = execStatus::FAILURE;

    pduMessage_t receivedMessage;

    /* blocking the thread if queue is empty */
    m_messageQueue.pop( receivedMessage );

    if( receivedMessage.header.externalDev )
    {
        ESP_LOGI( TAG, "Get state change message: %s", receivedMessage.signals );
        eStatus = m_stateManager.forwardMessage( receivedMessage );
        if( execStatus::SUCCESS != eStatus )
        {
            ESP_LOGW( TAG, "Failed to forward the message to state manager" );
        }
    }
    else /* internal devices ( drivers )*/
    {
        ESP_LOGI( TAG, "Get the internal message: %s", receivedMessage.signals );

        if( m_registeredDrivers[(int)eCommDriverID::MQTT_DRIVER] != nullptr )
        {
            execStatus eStatus = m_registeredCommDrivers[(int)eCommDriverID::MQTT_DRIVER]->forwardMessage( receivedMessage );
            if( execStatus::SUCCESS != eStatus )
            {
                ESP_LOGW( TAG, "Failed to forward the message to mqtt driver" );
            }
        }   
    }
}

execStatus DriverManager::update()
{
    eDeviceState state;
    execStatus eStatus = m_stateManager.getState( state );
    
    if( execStatus::SUCCESS == eStatus )
    {
        ESP_LOGI( TAG, "successful get state");
        eStatus = changeState( state );
    }

    return eStatus;
}