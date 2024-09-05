#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include "Interfaces/IStateManager.hpp"
#include "Interfaces/IStateProvider.hpp"
#include "Interfaces/IObserver.hpp"

namespace Service
{
    class StateManager : public Interface::IStateManager, public Interface::IStateProvider
    {
    public:

        StateManager();
        ~StateManager();

        /* method from IStateManager interface */
        execStatus forwardMessage( const pduMessage_t & message );
        execStatus getState( eDeviceState & state );

        /* methods from IStateProvider interface */
        execStatus registerObserver( Interface::IObserver * stateObserver );
        execStatus unregisterObserver( Interface::IObserver * stateObserver );

    protected:

        execStatus notifyObservers();
        execStatus parseMessage( const pduMessage_t & message );
    
    private:

        static const uint8_t maxNumOfObservers = 10;
        uint8_t m_observerIndex;

        eDeviceState m_deviceState;

        Interface::IObserver * m_registeredObservers[ maxNumOfObservers ];   
    };
}

#endif // STATE_MANAGER_HPP