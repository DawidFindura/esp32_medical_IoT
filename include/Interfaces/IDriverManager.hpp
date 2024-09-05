#ifndef IDRIVER_MANAGER_HPP
#define IDRIVER_MANAGER_HPP

#include "common.hpp"
#include "Enums/eExecStatus.hpp"
#include "Interfaces/IDriver.hpp"
#include "Enums/eDriverID.hpp"

namespace Interface
{
    class IDriverManager
    {
    public:

        virtual ~IDriverManager() = default;

        virtual execStatus registerDriver( IDriver * const driver, const eDriverID driverID ) = 0;
        virtual execStatus unregisterDriver( IDriver * const driver, const eDriverID driverID ) = 0;

        virtual execStatus registerCommDriver( IDriver * const driver, const eCommDriverID driverID ) = 0;
        virtual execStatus unregisterCommDriver( IDriver * const driver, const eCommDriverID driverID ) = 0;

        virtual execStatus sendMessage( const pduMessage_t & message ) = 0;

    };
};

#endif // IDRIVER_MANAGER_HPP