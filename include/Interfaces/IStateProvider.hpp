#ifndef ISTATE_PROVIDER_HPP
#define ISTATE_PROVIDER_HPP

#include "Enums/eExecStatus.hpp"
#include "Interfaces/IObserver.hpp"

namespace Interface
{
    class IStateProvider
    {
    public:

        virtual ~IStateProvider() = default;

        virtual execStatus registerObserver( Interface::IObserver * stateObserver ) = 0;
        virtual execStatus unregisterObserver( Interface::IObserver * stateObserver ) = 0;
        
    };
}

#endif // ISTATE_PROVIDER_HPP