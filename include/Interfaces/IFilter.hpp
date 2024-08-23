#ifndef I_FILTER_HPP
#define I_FILTER_HPP

#include "Enums/eExecStatus.hpp"

namespace Interface
{
    class IFilter
    {
    public:

        virtual ~IFilter() = default;

        virtual execStatus calculate_output( const float filter_input, float & filter_output ) = 0;

    };
}

#endif // I_FILTER_HPP