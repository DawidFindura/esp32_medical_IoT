#ifndef ENUM_DRIVER_STATE
#define ENUM_DRIVER_STATE

/**
 * @brief enum describing current internal state of device driver.
 *  
 */
enum class eDriverState
{
    UNINITIALIZED,
    DEINITIALIZED,
    INITIALIZED,
    STARTED,
    STOPPED
};

#endif // ENUM_DRIVER_STATE