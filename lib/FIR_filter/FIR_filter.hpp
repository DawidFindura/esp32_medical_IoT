#ifndef FIR_FILTER_HPP
#define FIR_FILTER_HPP

#include <stdint.h>

#include "Interfaces/IFilter.hpp"
#include "Enums/execStatus.hpp"

namespace Filter
{
    class FIR_filter : public Interface::IFilter
    {
    public:

        FIR_filter( const float * const in_filter_coeff, const uint32_t in_filter_len );
        ~FIR_filter();

        execStatus calculate_output( const float filter_input, float & filter_output );

    private:

        const float * const m_filter_coeff;
        const uint32_t m_filter_length;

        float * m_circular_buffer;
        uint32_t m_buffer_index;
    };
}

#endif // FIR_FILTER_HPP