#include <esp_log.h>

#include "StateManager.hpp"

static const char * TAG = "State Manager";

using namespace Service;

StateManager::StateManager() : m_observerIndex( 0 ), m_deviceState( eDeviceState::SLEEP )
{
    for( int observerIndex = 0; observerIndex < maxNumOfObservers; observerIndex++ )
    {
        m_registeredObservers[ observerIndex ] = nullptr;
    }

}

StateManager::~StateManager()
{

}

execStatus StateManager::forwardMessage( const pduMessage_t & message )
{
    execStatus eStatus = parseMessage( message );
    
    if( execStatus::SUCCESS == eStatus )
    {   
        ESP_LOGI( TAG, "parsing message successful" );
        eStatus = notifyObservers();
    }
    
    return eStatus;
}

execStatus StateManager::registerObserver( Interface::IObserver * stateObserver )
{

    execStatus eStatus = execStatus::FAILURE;

    if( nullptr == stateObserver )
    {
        return eStatus = execStatus::NULL_POINTER;
    }

    if( m_observerIndex < maxNumOfObservers )
    {
        m_registeredObservers[ m_observerIndex ] = stateObserver;
        m_observerIndex++;
        eStatus = execStatus::SUCCESS;
    }

    return eStatus;
}

execStatus StateManager::unregisterObserver( Interface::IObserver * stateObserver )
{
    execStatus eStatus = execStatus::FAILURE;

    if( nullptr == stateObserver )
    {
        return eStatus = execStatus::NULL_POINTER;
    }

    int observer;
    for( observer = 0; observer < m_observerIndex; observer++ )
    {
        if( m_registeredObservers[ observer ] == stateObserver )
        {
            m_registeredObservers[ observer ] = nullptr;
            eStatus = execStatus::SUCCESS;
        }
    }

    if( observer == m_observerIndex )
    {
        m_observerIndex--;
    }

    return eStatus;
}

execStatus StateManager::notifyObservers()
{
    execStatus eStatus = execStatus::FAILURE;

    for( int observerIdx = 0; observerIdx < m_observerIndex; observerIdx++ )
    {
        if( m_registeredObservers[ observerIdx ] != nullptr )
        {
            ESP_LOGI( TAG, "In notify observers" );
            eStatus = m_registeredObservers[ observerIdx ]->update();
            if( execStatus::SUCCESS != eStatus )
            {
                ESP_LOGI( TAG, "unsuccessful update on observer: %d", observerIdx );
                return eStatus;
            }
        }
    }

    return eStatus;
}

execStatus StateManager::parseMessage( const pduMessage_t & message )
{
    execStatus eStatus = execStatus::FAILURE;

    char * command = (char *)message.signals;

    for( int commandIdx = 0; commandIdx < MAX_NUM_OF_COMMANDS; commandIdx++ )
    {
        if( strcmp( command, command_list[commandIdx]) == 0 )
        {
            m_deviceState = static_cast<eDeviceState>(commandIdx);
            eStatus = execStatus::SUCCESS;
        }
    } 

    return eStatus;
}

execStatus StateManager::getState( eDeviceState & state )
{
    state = m_deviceState;

    return execStatus::SUCCESS;
}