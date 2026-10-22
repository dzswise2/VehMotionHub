#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mutexes */
extern osMutexId g_vehicleDataMutex;
extern osMutexId g_telemetryMutex;
extern osMutexId g_logMutex;

/* Binary/counting semaphores (ISR -> task) */
extern osSemaphoreId g_canRxSem;
extern osSemaphoreId g_usart3RxSem;
extern osSemaphoreId g_imuTickSem;

void App_RtosInitObjects(void);

void App_LogLock(void);
void App_LogUnlock(void);
int App_LogPrintf(const char *fmt, ...);

void App_NotifyCanRxFromIsr(void);
void App_NotifyImuTickFromIsr(void);
void App_NotifyUsart3FrameFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RTOS_H */
