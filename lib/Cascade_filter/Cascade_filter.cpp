#include <stdlib.h>

#include "Cascade_filter.hpp"

using namespace Filter;

Cascade_filter::Cascade_filter() : m_num_of_filt_elems( 0 )
{
    for( int i = 0; i < max_num_of_filt_elems; i++ )
    {
        m_filters[ i ] = NULL;
    }
}

Cascade_filter::~Cascade_filter()
{

}

execStatus Cascade_filter::calculate_output( const float filter_input, float & filter_output )
{
    execStatus eStatus = execStatus::FAILURE;
    float temp_signal_out = 0.0f;
    float temp_signal_in = filter_input;

    for( int filter_index = 0; filter_index < m_num_of_filt_elems; filter_index++ )
    {
        if( NULL == m_filters[ filter_index ] )
        {
            return eStatus = execStatus::NULL_POINTER;
        }

        eStatus = m_filters[filter_index]->calculate_output( temp_signal_in, temp_signal_out );
        if( eStatus != execStatus::SUCCESS )
        {
            return eStatus;
        }

        temp_signal_in = temp_signal_out;
    }

    if( execStatus::SUCCESS == eStatus )
    {
        filter_output = temp_signal_out;
    }

    return eStatus;   
 }

 execStatus Cascade_filter::add_filter( IFilter * in_filter )
 {
    execStatus eStatus = execStatus::FAILURE;

    if( NULL == in_filter )
    {
        return eStatus = execStatus::NULL_POINTER;
    }

    if( m_num_of_filt_elems < max_num_of_filt_elems )
    {
        m_filters[ m_num_of_filt_elems ] = in_filter;
        m_num_of_filt_elems++;
        eStatus = execStatus::SUCCESS;
    }
    else
    {
        eStatus = execStatus::FAILURE;
    }

    return eStatus;
 }