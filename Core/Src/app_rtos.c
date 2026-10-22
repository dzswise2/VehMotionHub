#include "app_rtos.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>

osMutexId g_vehicleDataMutex;
osMutexId g_telemetryMutex;
osMutexId g_logMutex;

osSemaphoreId g_canRxSem;
osSemaphoreId g_usart3RxSem;
osSemaphoreId g_imuTickSem;

osMutexDef(VehicleDataMutex);
osMutexDef(TelemetryMutex);
osMutexDef(LogMutex);

osSemaphoreDef(CanRxSem);
osSemaphoreDef(Usart3RxSem);
osSemaphoreDef(ImuTickSem);

void App_RtosInitObjects(void)
{
  g_vehicleDataMutex = osMutexCreate(osMutex(VehicleDataMutex));
  g_telemetryMutex = osMutexCreate(osMutex(TelemetryMutex));
  g_logMutex = osMutexCreate(osMutex(LogMutex));

  /* Start at 0; ISR gives tokens when events occur. CAN uses counting sem for bursts. */
  /* CMSIS wrapper: count==1 creates an empty binary semaphore. */
  g_canRxSem = osSemaphoreCreate(osSemaphore(CanRxSem), 1);
  g_usart3RxSem = osSemaphoreCreate(osSemaphore(Usart3RxSem), 1);
  g_imuTickSem = osSemaphoreCreate(osSemaphore(ImuTickSem), 1);
}

void App_LogLock(void)
{
  if (g_logMutex != NULL)
  {
    (void)osMutexWait(g_logMutex, osWaitForever);
  }
}

void App_LogUnlock(void)
{
  if (g_logMutex != NULL)
  {
    (void)osMutexRelease(g_logMutex);
  }
}

int App_LogPrintf(const char *fmt, ...)
{
  char line[160];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return len;
  }

  if ((size_t)len >= sizeof(line))
  {
    len = (int)sizeof(line) - 1;
  }

  App_LogLock();
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, 0xFFFF);
  App_LogUnlock();

  return len;
}

void App_NotifyCanRxFromIsr(void)
{
  (void)osSemaphoreRelease(g_canRxSem);
}

void App_NotifyImuTickFromIsr(void)
{
  (void)osSemaphoreRelease(g_imuTickSem);
}

void App_NotifyUsart3FrameFromIsr(void)
{
  (void)osSemaphoreRelease(g_usart3RxSem);
}
