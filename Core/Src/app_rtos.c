#include "app_rtos.h"
#include "main.h"
#include "usart.h"

#define APP_LOG_CAPACITY (160U)

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

static void App_LogLock(void)
{
  if (g_logMutex != NULL)
  {
    (void)osMutexWait(g_logMutex, osWaitForever);
  }
}

static void App_LogUnlock(void)
{
  if (g_logMutex != NULL)
  {
    (void)osMutexRelease(g_logMutex);
  }
}

void App_RtosInitObjects(void)
{
  /* CMSIS: count == 1 creates an empty binary semaphore. */
  g_vehicleDataMutex = osMutexCreate(osMutex(VehicleDataMutex));
  g_telemetryMutex = osMutexCreate(osMutex(TelemetryMutex));
  g_logMutex = osMutexCreate(osMutex(LogMutex));
  g_canRxSem = osSemaphoreCreate(osSemaphore(CanRxSem), (int32_t)1);
  g_usart3RxSem = osSemaphoreCreate(osSemaphore(Usart3RxSem), (int32_t)1);
  g_imuTickSem = osSemaphoreCreate(osSemaphore(ImuTickSem), (int32_t)1);

  if ((g_vehicleDataMutex == NULL) ||
      (g_telemetryMutex == NULL) ||
      (g_logMutex == NULL) ||
      (g_canRxSem == NULL) ||
      (g_usart3RxSem == NULL) ||
      (g_imuTickSem == NULL))
  {
    Error_Handler();
  }
}

void App_LogWrite(const uint8_t *data, uint16_t length)
{
  uint8_t local[APP_LOG_CAPACITY];
  uint16_t i;
  uint16_t n;

  if ((data != NULL) && (length > (uint16_t)0))
  {
    n = length;
    if (n > (uint16_t)APP_LOG_CAPACITY)
    {
      n = (uint16_t)APP_LOG_CAPACITY;
    }

    for (i = (uint16_t)0; i < n; i++)
    {
      local[i] = data[i];
    }

    App_LogLock();
    (void)HAL_UART_Transmit(&huart1, local, n, 0xFFFFU);
    App_LogUnlock();
  }
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
