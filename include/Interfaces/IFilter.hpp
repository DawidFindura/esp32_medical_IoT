#ifndef I_FILTER_HPP
#define I_FILTER_HPP

#include "Enums/execStatus.hpp"

namespace Interface
{
    class IFilter
    {
    public:

        virtual ~IFilter() = default;

        typedef struct filter_input
        {
            filter_input( float in_x_sample, float in_y_sample = 0.0f ) : x_sample( in_x_sample ) {}

            float x_sample;
            float y_sample;

        } filter_input_t;

        virtual execStatus calculate_output( const filter_input_t & filter_input, float & filter_output ) = 0;

    };
}

#endif // I_FILTER_HPP