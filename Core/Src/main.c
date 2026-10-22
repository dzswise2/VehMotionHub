/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : VehMotionHub — vehicle IMU + CAN motion logger
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scha63x.h"
#include "app_rtos.h"
#include "app_tasks.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Vehicle CAN standard IDs */
#define CAN_ID_STEER_WHL_AG  0x040U  /* SteerWhlAgSafe */
#define CAN_ID_WHL_FRNT      0x1A0U  /* WhlSpdCircumlFrntLe / FrntRi */
#define CAN_ID_WHL_REAR      0x1C0U  /* WhlSpdCircumlReLe / ReRi */
#define CAN_ID_VEH_SPD_LGT   0x0E0U  /* VehSpdLgtSafe */
#define CAN_ID_GEAR_LVR      0x120U  /* GearLvrIndcn */

#define GRODO_CAN_UPDATES_MIN  5U    /* one update from each handled ID above */

static scha63x_raw_data_summed scha63x_data_summed_raw;

scha63x_raw_data scha63x_raw_data_last;
int status;

typedef struct {
  float veh_speed;   /* longitudinal vehicle speed (m/s scaled on bus) */
  float gear;        /* gear lever indication (raw, scale per DBC) */
  float steer;       /* steering wheel angle (raw, scale per DBC) */
  float whl_ft_l;    /* front left wheel circumferential speed */
  float whl_ft_r;    /* front right wheel circumferential speed */
  float whl_rr_l;    /* rear left wheel circumferential speed */
  float whl_rr_r;    /* rear right wheel circumferential speed */
} vehicle_can_payload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t CheckXOR(char *pbuff)
{
  uint8_t tmp = 0;

  if (pbuff == NULL || pbuff[0] != '$')
  {
    return 0;
  }

  for (int i = 1; i < 200 && pbuff[i] != '\0'; i++)
  {
    if (pbuff[i] == '*')
    {
      return tmp;
    }
    tmp ^= (uint8_t)pbuff[i];
  }
  return 0;
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
RTC_DateTypeDef sdatestructure;
RTC_TimeTypeDef stimestructure;

static const uint16_t monDays[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

/* RTC holds Beijing (UTC+8) civil time; output is Unix seconds (UTC). */
uint32_t time2Stamp(RTC_DateTypeDef date, RTC_TimeTypeDef time)
{
  double result;
  uint16_t year = (uint16_t)(date.Year + 2000U);

  if (date.Month < 1U || date.Month > 12U)
  {
    return 0U;
  }

  result = (year - 1970U) * 365.0 * 24.0 * 3600.0
         + (monDays[date.Month - 1U] + date.Date - 1U) * 24.0 * 3600.0
         + ((int32_t)time.Hours - 8) * 3600.0 + time.Minutes * 60.0 + time.Seconds;
  result += (date.Month > 2U && ((year % 4U) == 0U) && ((year % 100U) != 0U || (year % 400U) == 0U)) * 24.0 * 3600.0;
  year -= 1969U;
  result += ((year / 4U) - (year / 100U) + (year / 400U)) * 24.0 * 3600.0;

  return (uint32_t)result;
}

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
void FDCAN1_Config(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void sync_rtc_from_usart3(void);
static void vehicle_can_handle_frame(uint32_t can_id, const uint8_t *payload);
static void publish_grodo_sentence(void);
static void publish_grimu_sentence(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char rx4_time[4];
static char imu_field[8][20];
static char grodo_field[7][16];
static char grimu_sentence[130];
static char grodo_sentence[130];

static double rtc_unix_sec = 0.0;
static double imu_time_offset = 0.0;
static double time_ms = 0.0;
static uint8_t xor_byte = 0;
static int nmea_field_index = 0;
static int can_update_count = 0;

static scha63x_real_data scha63x_data;
static vehicle_can_payload_t vehicle_data;

static void copy_two_digits(char *dest, const uint8_t *src)
{
  dest[0] = (char)src[0];
  dest[1] = (char)src[1];
  dest[2] = '\0';
}

static void usart3_forward_to_uart2(void)
{
  uint16_t len = USART3_RxFrameLen;

  if (len > 0U)
  {
    (void)HAL_UART_Transmit(&huart2, USART3_Buff, len, 0xFFFF);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 0xFFFF);
  }
}

static void sync_rtc_from_usart3(void)
{
  nmea_field_index = 0;
  for (int idx = 0; idx < 100 && USART3_Buff[idx] != '\0'; idx++)
  {
    if (USART3_Buff[idx] != ',')
    {
      continue;
    }
    nmea_field_index++;
    if (nmea_field_index == 1 && (idx + 6) < (int)sizeof(USART3_Buff))
    {
      copy_two_digits(rx4_time, &USART3_Buff[idx + 1]);
      stimestructure.Hours = (uint8_t)atoi(rx4_time);
      copy_two_digits(rx4_time, &USART3_Buff[idx + 3]);
      stimestructure.Minutes = (uint8_t)atoi(rx4_time);
      copy_two_digits(rx4_time, &USART3_Buff[idx + 5]);
      stimestructure.Seconds = (uint8_t)atoi(rx4_time);
    }
    else if (nmea_field_index == 9 && (idx + 6) < (int)sizeof(USART3_Buff))
    {
      copy_two_digits(rx4_time, &USART3_Buff[idx + 1]);
      sdatestructure.Date = (uint8_t)atoi(rx4_time);
      copy_two_digits(rx4_time, &USART3_Buff[idx + 3]);
      sdatestructure.Month = (uint8_t)atoi(rx4_time);
      copy_two_digits(rx4_time, &USART3_Buff[idx + 5]);
      sdatestructure.Year = (uint8_t)atoi(rx4_time);
    }
  }

  if (osMutexWait(g_telemetryMutex, osWaitForever) != osOK)
  {
    return;
  }

  if (sdatestructure.Year != 0U)
  {
    HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
  }
  else
  {
    HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
  }

  imu_time_offset = 0.0;
  rtc_unix_sec = (double)time2Stamp(sdatestructure, stimestructure);
  time_ms = rtc_unix_sec + imu_time_offset;
  USART3_Receive_Flag = false;
  (void)osMutexRelease(g_telemetryMutex);
}

static void handle_usart3_frame(void)
{
  if (!USART3_Receive_Flag)
  {
    return;
  }

  usart3_forward_to_uart2();
  sync_rtc_from_usart3();
}

static void vehicle_can_handle_frame(uint32_t can_id, const uint8_t *payload)
{
  switch (can_id)
  {
  case CAN_ID_VEH_SPD_LGT:
    vehicle_data.veh_speed = (float)payload[5] * 0.00391f;
    can_update_count++;
    break;
  case CAN_ID_WHL_FRNT:
    vehicle_data.whl_ft_l = (float)payload[1] * 0.00391f;
    vehicle_data.whl_ft_r = (float)payload[5] * 0.00391f;
    can_update_count++;
    break;
  case CAN_ID_WHL_REAR:
    vehicle_data.whl_rr_l = (float)payload[3] * 0.00391f;
    vehicle_data.whl_rr_r = (float)payload[7] * 0.00391f;
    can_update_count++;
    break;
  case CAN_ID_STEER_WHL_AG:
    /* SteerWhlAgSafe: byte layout/scale per vehicle DBC; adjust if needed. */
    vehicle_data.steer = (float)payload[1];
    can_update_count++;
    break;
  case CAN_ID_GEAR_LVR:
    /* GearLvrIndcn: raw lever position in payload[0] (DBC-specific). */
    vehicle_data.gear = (float)payload[0];
    can_update_count++;
    break;
  default:
    break;
  }
}

static void publish_grimu_sentence(void)
{
  strcpy(grimu_sentence, "$GRIMU,");
  sprintf(imu_field[0], "%.3f", time_ms);
  sprintf(imu_field[1], "%.3f", scha63x_data.acc_x);
  sprintf(imu_field[2], "%.3f", scha63x_data.acc_y);
  sprintf(imu_field[3], "%.3f", scha63x_data.acc_z);
  sprintf(imu_field[4], "%.3f", scha63x_data.gyro_x);
  sprintf(imu_field[5], "%.3f", scha63x_data.gyro_y);
  sprintf(imu_field[6], "%.3f", scha63x_data.gyro_z);
  sprintf(imu_field[7], "%.3f", scha63x_data.temp_due);

  for (int i = 0; i < 8; i++)
  {
    strcat(grimu_sentence, imu_field[i]);
    strcat(grimu_sentence, (i < 7) ? "," : "*");
  }

  xor_byte = CheckXOR(grimu_sentence);
  (void)App_LogPrintf("%s%02x\r\n", grimu_sentence, xor_byte);
  imu_time_offset += 0.01;
  time_ms = rtc_unix_sec + imu_time_offset;
}

static void publish_grodo_sentence(void)
{
  /* $GRODO: time, FL, FR, RL, RR, longitudinal speed, gear */
  strcpy(grodo_sentence, "$GRODO,");
  sprintf(grodo_field[0], "%.3f", time_ms);
  sprintf(grodo_field[1], "%.3f", vehicle_data.whl_ft_l);
  sprintf(grodo_field[2], "%.3f", vehicle_data.whl_ft_r);
  sprintf(grodo_field[3], "%.3f", vehicle_data.whl_rr_l);
  sprintf(grodo_field[4], "%.3f", vehicle_data.whl_rr_r);
  sprintf(grodo_field[5], "%.3f", vehicle_data.veh_speed);
  sprintf(grodo_field[6], "%.3f", vehicle_data.gear);

  for (int i = 0; i < 7; i++)
  {
    strcat(grodo_sentence, grodo_field[i]);
    strcat(grodo_sentence, (i < 6) ? "," : "*");
  }

  xor_byte = CheckXOR(grodo_sentence);
  (void)App_LogPrintf("%s%02x\r\n", grodo_sentence, xor_byte);
  can_update_count = 0;
}

static void imu_sample_and_publish(void)
{
  scha63x_read_data(&scha63x_raw_data_last);
  scha63x_data_summed_raw.acc_x_lsb = scha63x_raw_data_last.acc_x_lsb;
  scha63x_data_summed_raw.acc_y_lsb = scha63x_raw_data_last.acc_y_lsb;
  scha63x_data_summed_raw.acc_z_lsb = scha63x_raw_data_last.acc_z_lsb;
  scha63x_data_summed_raw.gyro_x_lsb = scha63x_raw_data_last.gyro_x_lsb;
  scha63x_data_summed_raw.gyro_y_lsb = scha63x_raw_data_last.gyro_y_lsb;
  scha63x_data_summed_raw.gyro_z_lsb = scha63x_raw_data_last.gyro_z_lsb;
  scha63x_data_summed_raw.temp_due_lsb = scha63x_raw_data_last.temp_due_lsb;
  scha63x_data_summed_raw.temp_uno_lsb = scha63x_raw_data_last.temp_uno_lsb;
  scha63x_convert_data(&scha63x_data_summed_raw, &scha63x_data);
  scha63x_cross_axis_compensation(&scha63x_data);

  if (osMutexWait(g_telemetryMutex, osWaitForever) != osOK)
  {
    return;
  }

  rtc_unix_sec = (double)time2Stamp(sdatestructure, stimestructure);
  time_ms = rtc_unix_sec + imu_time_offset;
  publish_grimu_sentence();
  (void)osMutexRelease(g_telemetryMutex);
}

void data_output(void const *argument)
{
  (void)argument;

  for (;;)
  {
    if (osSemaphoreWait(g_usart3RxSem, 0) == osOK)
    {
      handle_usart3_frame();
    }

    if (osSemaphoreWait(g_imuTickSem, 0) == osOK)
    {
      imu_sample_and_publish();
    }
    else
    {
      osDelay(1);
    }
  }
}

void work_state(void const *argument)
{
  for (;;)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    osDelay(500);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    osDelay(500);
  }
}

void can_com(void const *argument)
{
  int publish_grodo = 0;

  (void)argument;

  for (;;)
  {
    if (osSemaphoreWait(g_canRxSem, osWaitForever) != osOK)
    {
      continue;
    }

    if (osMutexWait(g_vehicleDataMutex, osWaitForever) != osOK)
    {
      continue;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
    {
      if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
      {
        break;
      }
      vehicle_can_handle_frame(RxHeader.Identifier, RxData);
    }

    publish_grodo = (can_update_count >= (int)GRODO_CAN_UPDATES_MIN) ? 1 : 0;
    (void)osMutexRelease(g_vehicleDataMutex);

    if (publish_grodo)
    {
      if (osMutexWait(g_telemetryMutex, osWaitForever) == osOK)
      {
        if (osMutexWait(g_vehicleDataMutex, osWaitForever) == osOK)
        {
          publish_grodo_sentence();
          (void)osMutexRelease(g_vehicleDataMutex);
        }
        (void)osMutexRelease(g_telemetryMutex);
      }
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  FDCAN1_Config();
  HAL_TIM_Base_Start_IT(&htim16);
  static char scha63x_serial_num[14];
  status = scha63x_init(scha63x_serial_num);
  // Initialize SCHA63X sensor
  if (status != SCHA63X_OK)
  {
    printf("\r\n IMU INIT fail!\r\n");
  }

  HAL_Delay(1000);
  /* USER CODE END 2 */

  MX_FREERTOS_Init();
  osKernelStart();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI2;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 5;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void FDCAN1_ConfigFilter(uint32_t index, uint32_t id)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = id;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN1_Config(void)
{
  FDCAN1_ConfigFilter(0U, CAN_ID_STEER_WHL_AG);
  FDCAN1_ConfigFilter(1U, CAN_ID_WHL_FRNT);
  FDCAN1_ConfigFilter(2U, CAN_ID_WHL_REAR);
  FDCAN1_ConfigFilter(3U, CAN_ID_VEH_SPD_LGT);
  FDCAN1_ConfigFilter(4U, CAN_ID_GEAR_LVR);

  /* Reject non-matching standard/extended IDs and remote frames. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)hfdcan;
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    App_NotifyCanRxFromIsr();
  }
}



/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* TIM16: IMU/GRODO sentence tick (see tim.c prescaler/period). */
  if (htim == &htim16)
  {
    App_NotifyImuTickFromIsr();
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
