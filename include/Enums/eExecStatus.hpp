#ifndef ENUM_EXEC_STATUS
#define ENUM_EXEC_STATUS

/**
 * @brief enum describing execution status returned after function call.
 *  
 */
enum class execStatus
{
    FAILURE,
    SUCCESS,
    NOT_INITIALIZED,
    NULL_POINTER
};

#endif // ENUM_EXEC_STATUS