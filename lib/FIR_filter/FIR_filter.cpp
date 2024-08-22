#include <iostream>

#include "FIR_filter.hpp"

using namespace Filter;

FIR_filter::FIR_filter( const float * const in_filter_coeff, const uint32_t in_filter_len ) : m_filter_coeff( in_filter_coeff ), m_filter_length( in_filter_len ), m_buffer_index( 0 )
{
    if( NULL == in_filter_coeff ) 
    {
        std::cout<< "Invalid argument: null pointer for filter coefficients" << std::endl;
    }

    if( in_filter_len > 0 )
    {
        m_circular_buffer = new float [ in_filter_len ](); // zero initialized newly created array
    }
    else
    {
        m_circular_buffer = NULL;
        std::cout<< "Invalid argument: zero filter length" << std::endl;
    }
}

FIR_filter::~FIR_filter()
{
    if( NULL != m_circular_buffer )
    {
        delete [] m_circular_buffer;
        m_circular_buffer = NULL;
    }
}

execStatus FIR_filter::calculate_output( const float filter_input, float & filter_output )
{
    execStatus eStatus = execStatus::FAILURE;
    float temp_output = 0.0f;

    if( NULL == m_circular_buffer || NULL == m_filter_coeff )
    {
        return eStatus = execStatus::NULL_POINTER;
    }

    m_circular_buffer[ m_buffer_index ] = filter_input; 
    
    m_buffer_index++;
    if( m_buffer_index == m_filter_length )
    {
        m_buffer_index = 0;
    }

    uint32_t sampleIndex = m_buffer_index;

    for( uint32_t convIndex = 0; convIndex < m_filter_length; convIndex++ )
    {   
        if( sampleIndex > 0 )
        {
            sampleIndex--;
        }
        else
        {
            sampleIndex = m_filter_length - 1;
        }

        temp_output += m_filter_coeff[ convIndex ] * m_circular_buffer[ sampleIndex ]; 
    }

    filter_output = temp_output;

    return eStatus = execStatus::SUCCESS;
}

