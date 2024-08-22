#ifndef IIR_FILTER_HPP
#define IIR_FILTER_HPP

#include <stdint.h>

#include "Interfaces/IFilter.hpp"

namespace Filter
{
    class IIR_filter : public Interface::IFilter
    {
    public:
        
        IIR_filter( const float * const in_a_filter_coeff, const float * const in_b_filter_coeff, const uint32_t in_filter_len );
        ~IIR_filter();

        execStatus calculate_output( const float filter_input, float & filter_output );
    
    private:

        const float * const m_a_filter_coeff;
        const float * const m_b_filter_coeff;
        const uint32_t m_filter_length;

        float * m_prev_input_buffer;
        float * m_prev_output_buffer;
        
        uint32_t m_input_buffer_index;
        uint32_t m_output_buffer_index;
    };
}

#endif // IIR_FILTER_HPP