
#include "mbed.h"
#include "rtos.h"
#include "RTOSerrstr.h"
 
#ifdef  __cplusplus
extern "C" {
#endif /* __cplusplus */
 
const char *
getOsStatusStr(osStatus status)
{
    const char *str;
    
    switch (status) {
        case osOK:
            str = "osOK";
            break;
        case osEventSignal:
            str = "osEventSignal";
            break;
        case osEventMessage:
            str = "osEventMessage";
            break;
        case osEventMail:
            str = "osEventMail";
            break;
        case osEventTimeout:
            str = "osEventTimeout";
            break;
        case osErrorParameter:
            str = "osErrorParameter";
            break;
        case osErrorResource:
            str = "osErrorResource";
            break;
        case osErrorTimeoutResource:
            str = "osErrorTimeoutResource";
            break;
        case osErrorISR:
            str = "osErrorISR";
            break;
        case osErrorISRRecursive:
            str = "osErrorISRRecursive";
            break;
        case osErrorPriority:
            str = "osErrorPriority";
            break;
        case osErrorNoMemory:
            str = "osErrorNoMemory";
            break;
        case osErrorValue:
            str = "osErrorValue";
            break;
        case osErrorOS:
            str = "osErrorOS";
            break;
        default:
            str = "Unknown";
            break; 
    }
    return(str);
}
 
#ifdef  __cplusplus
}
#endif /* __cplusplus */