/**
 *
 */
 
#ifndef RTOS_ERR_STR_H
#define RTOS_ERR_STR_H
 
#define RTOS_ERR_STR_MAJOR  (0)
#define RTOS_ERR_STR_MINOR  (1)
 #include "mbed.h"
#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */
/**
 * @fn getOsStatusStr
 * @param [in] status 
 */
extern const char * getOsStatusStr(osStatus status);
 
#ifdef  __cplusplus
}
#endif /* __cplusplus */
 
#endif /* RTOS_ERR_STR_H */