#include <iostream>
#include "IIR_filer.hpp"

/* highpass filter coefficients for fc = 0.5 Hz*/
//const float iir_filter_a_coeff_high[5]={1.000000,-3.971566,5.915101,-3.915501,0.971966};
//const float iir_filter_b_coeff_high[5]={0.985883,-3.943533,5.915300,-3.943533,0.985883};
const float iir_filter_a_coeff_high[3]={1.000000,-1.971567,0.971966};
const float iir_filter_b_coeff_high[3]={0.985883,-1.971767,0.985883};

/* stopband filter coefficients for power line noise; fc1 = 49 Hz fc2 = 51 Hz */
const float iir_filter_a_coeff_stop[5]={1.000000,-2.113270,3.059843,-2.054015,0.944718};
const float iir_filter_b_coeff_stop[5]={0.971966,-2.083643,3.060629,-2.083643,0.971966};

/* lowpass filter coefficients for fc = 150 Hz */
const float iir_filter_a_coeff_low[5]={1.000000,3.671729,5.067998,3.115967,0.719910};
const float iir_filter_b_coeff_low[5]={0.848475,3.393901,5.090852,3.393901,0.848475};

/* cascade of lowpass, stopband and high pass filters coefficients */
const float iir_filter_a_coeff_cascade[13]={1.000000,-2.413107,0.094064,5.426292,-6.207457,-1.564225,8.490730,-4.920080,-2.883555,4.694780,-1.139442,-1.239047,0.661046};
const float iir_filter_b_coeff_cascade[13]={0.813047,-1.742962,-0.691980,5.228887,-4.549506,-3.485925,8.856877,-3.485925,-4.549506,5.228887,-0.691980,-1.742962,0.813047};


using namespace Filter;

IIR_filter::IIR_filter( const float * const in_a_filter_coeff, const float * const in_b_filter_coeff, const uint32_t in_filter_len ) : 

    m_a_filter_coeff( in_a_filter_coeff ),
    m_b_filter_coeff( in_b_filter_coeff ),
    m_filter_length( in_filter_len ),
    m_input_buffer_index( 0 ),
    m_output_buffer_index( 0 )

{
    if( NULL == in_a_filter_coeff || NULL == in_b_filter_coeff ) 
    {
        std::cout<< "Invalid argument: null pointer for filter coefficients" << std::endl;
    }

    if( in_filter_len > 0 )
    {
        m_prev_input_buffer = new float [ in_filter_len ](); // zero initialized newly created array
        m_prev_output_buffer = new float [ in_filter_len - 1 ](); // zero initialized newly created array
    }
    else
    {
        m_prev_input_buffer = NULL;
        m_prev_output_buffer = NULL;
        std::cout<< "Invalid argument: zero filter length" << std::endl;
    }
}

IIR_filter::~IIR_filter()
{
    if( NULL != m_prev_input_buffer )
    {
        delete [] m_prev_input_buffer;
        m_prev_input_buffer = NULL;
    }

    if( NULL != m_prev_output_buffer)
    {
        delete [] m_prev_output_buffer;
        m_prev_output_buffer = NULL;
    }
}

execStatus IIR_filter::calculate_output( const float filter_input, float & filter_output )
{
    execStatus eStatus = execStatus::FAILURE;
    float temp_output_x = 0.0f;
    float temp_output_y = 0.0f;

    if( NULL == m_a_filter_coeff || NULL == m_b_filter_coeff  || NULL == m_prev_input_buffer || NULL == m_prev_output_buffer )
    {
        return eStatus = execStatus::NULL_POINTER;
    }

    /* add current filter input into the input samples buffer*/
    m_prev_input_buffer[ m_input_buffer_index ] = filter_input;
     
    m_input_buffer_index++;
    if( m_input_buffer_index == m_filter_length )
    {
        m_input_buffer_index = 0;
    }

    uint32_t prev_input_index = m_input_buffer_index;

    for( uint32_t convIndex = 0; convIndex < m_filter_length; convIndex++ )
    {   
        if( prev_input_index > 0 )
        {
            prev_input_index--;
        }
        else
        {
            prev_input_index = m_filter_length - 1;
        }

        temp_output_x += m_b_filter_coeff[ convIndex ] * m_prev_input_buffer[ prev_input_index ]; 
    }

    uint32_t prev_output_index = m_output_buffer_index;

    for( uint32_t convIndex = 1; convIndex < m_filter_length; convIndex++ )
    {
        if( prev_output_index > 0 )
        {
            prev_output_index--;
        }
        else
        {
            prev_output_index = m_filter_length - 2;
        }

        temp_output_y += m_a_filter_coeff[ convIndex ] * m_prev_output_buffer[ prev_output_index ];    
    }

    float temp_filter_output = temp_output_x - temp_output_y;

    /* add current output sample into the output samples buffer */
    m_prev_output_buffer[ m_output_buffer_index ] = temp_filter_output;
    
    m_output_buffer_index++;
    if( m_output_buffer_index == ( m_filter_length - 1) )
    {
        m_output_buffer_index = 0;
    }
    
    filter_output = temp_filter_output;

    return eStatus = execStatus::SUCCESS;
}
