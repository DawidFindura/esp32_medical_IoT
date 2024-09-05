#ifndef ISTATE_MANAGER_HPP
#define ISTATE_MANAGER_HPP

#include "Enums/eExecStatus.hpp"
#include "common.hpp"

namespace Interface
{
    class IStateManager
    {
    public:

        virtual ~IStateManager() = default;

        virtual execStatus forwardMessage( const pduMessage_t & message ) = 0;
        virtual execStatus getState( eDeviceState & state ) = 0;
    };
}

#endif // ISTATE_MANAGER_HPP