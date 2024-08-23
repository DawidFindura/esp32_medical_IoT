#ifndef CASCADE_FILTER_HPP
#define CASCADE_FILTER_HPP

#include <stdint.h>

#include "Interfaces/IFilter.hpp"
#include "Enums/eExecStatus.hpp"

namespace Filter
{
    class Cascade_filter : public Interface::IFilter
    {
    public:

        Cascade_filter();
        ~Cascade_filter();

        execStatus add_filter( IFilter * in_filter );
        execStatus calculate_output( const float filter_input, float & filter_output );

        static const uint8_t max_num_of_filt_elems = 10;

    private:
         
        IFilter * m_filters[ max_num_of_filt_elems ];
        uint32_t m_num_of_filt_elems;

    };
}
#endif // CASCADE_FILTER_HPP