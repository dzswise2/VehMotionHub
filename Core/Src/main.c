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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAN_ID_STEER_WHL_AG      (0x040U)
#define CAN_ID_WHL_FRNT          (0x1A0U)
#define CAN_ID_WHL_REAR          (0x1C0U)
#define CAN_ID_VEH_SPD_LGT       (0x0E0U)
#define CAN_ID_GEAR_LVR          (0x120U)
#define GRODO_CAN_UPDATES_MIN    ((uint8_t)5)
#define CAN_SPEED_SCALE          (0.00391f)
#define NMEA_SENTENCE_CAP        (160U)
#define NMEA_PREFIX_CAP          (16U)
#define UNIX_EPOCH_YEAR          (INT64_C(1970))
#define DAYS_PER_YEAR            (INT64_C(365))
#define SECONDS_PER_DAY          (INT64_C(86400))
#define SECONDS_PER_HOUR         (INT64_C(3600))
#define SECONDS_PER_MINUTE       (INT64_C(60))
#define BEIJING_UTC_OFFSET_H     (INT64_C(8))
#define STAMP_U32_MAX            (INT64_C(4294967295))
#define CAN_BYTE_GEAR            ((uint8_t)0)
#define CAN_BYTE_STEER           ((uint8_t)1)
#define CAN_BYTE_WHL_FL          ((uint8_t)1)
#define CAN_BYTE_WHL_RL          ((uint8_t)3)
#define CAN_BYTE_WHL_FR          ((uint8_t)5)
#define CAN_BYTE_WHL_RR          ((uint8_t)7)
#define CAN_BYTE_VEH_SPD         ((uint8_t)5)
#define NMEA_FIELD_TIME          ((uint8_t)1)
#define NMEA_FIELD_DATE          ((uint8_t)9)
#define UART_TX_TIMEOUT_MS       (0xFFFFU)

typedef struct
{
  float veh_speed;
  float gear;
  float steer;
  float whl_ft_l;
  float whl_ft_r;
  float whl_rr_l;
  float whl_rr_r;
} vehicle_can_payload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static RTC_DateTypeDef sdatestructure;
static RTC_TimeTypeDef stimestructure;
static double rtc_unix_sec = 0.0;
static double imu_time_offset = 0.0;
static double time_ms = 0.0;
static uint8_t can_update_count = (uint8_t)0;
static scha63x_real_data scha63x_data;
static vehicle_can_payload_t vehicle_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t ascii_hex(uint8_t nibble);
static void append_char(uint8_t *dst, uint32_t cap, uint32_t *pos, uint8_t ch);
static void nmea_begin(uint8_t *dst, uint32_t cap, uint32_t *pos, const char *prefix);
static void append_fixed3(uint8_t *dst, uint32_t cap, uint32_t *pos, double value);
static double f64_from_float(float value);
static void append_float_field(uint8_t *dst, uint32_t cap, uint32_t *pos, double value, bool last);
static uint8_t nmea_checksum(const uint8_t *sentence);
static void log_nmea(uint8_t *sentence, uint32_t cap, uint32_t pos);
static void log_cstring(const char *text);
static float u8_scaled(uint8_t raw);
static float u8_as_float(uint8_t raw);
static void note_can_update(void);
static uint8_t parse_two_digits(const uint8_t *src);
static uint32_t time2Stamp(const RTC_DateTypeDef *date, const RTC_TimeTypeDef *time);
static void usart3_forward_to_uart2(uint8_t *frame, uint16_t len);
static void sync_rtc_from_usart3(const uint8_t *frame, uint16_t len);
static void handle_usart3_frame(void);
static void vehicle_can_handle_frame(uint32_t can_id, const uint8_t *payload);
static void publish_grodo_sentence(void);
static void publish_grimu_sentence(void);
static void imu_sample_and_publish(void);
static void FDCAN1_ConfigFilter(uint32_t index, uint32_t id);
static void FDCAN1_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t ascii_hex(uint8_t nibble)
{
  static const uint8_t hex_chars[16] =
  {
    (uint8_t)'0', (uint8_t)'1', (uint8_t)'2', (uint8_t)'3',
    (uint8_t)'4', (uint8_t)'5', (uint8_t)'6', (uint8_t)'7',
    (uint8_t)'8', (uint8_t)'9', (uint8_t)'a', (uint8_t)'b',
    (uint8_t)'c', (uint8_t)'d', (uint8_t)'e', (uint8_t)'f'
  };
  uint8_t index;
  uint8_t ch;

  index = nibble;
  if (index > (uint8_t)15)
  {
    index = (uint8_t)0;
  }
  ch = hex_chars[index];
  return ch;
}

static void append_char(uint8_t *dst, uint32_t cap, uint32_t *pos, uint8_t ch)
{
  uint32_t index;

  if ((dst != NULL) && (pos != NULL))
  {
    index = *pos;
    if ((index + 1U) < cap)
    {
      dst[index] = ch;
      index++;
      dst[index] = 0U;
      *pos = index;
    }
  }
}

static void nmea_begin(uint8_t *dst, uint32_t cap, uint32_t *pos, const char *prefix)
{
  uint32_t i;
  char ch;
  uint8_t byte;
  bool stop;

  if (pos != NULL)
  {
    *pos = 0U;
  }
  if ((dst != NULL) && (cap > 0U))
  {
    dst[0] = 0U;
  }

  stop = false;
  if (prefix != NULL)
  {
    for (i = 0U; i < NMEA_PREFIX_CAP; i++)
    {
      if (stop == false)
      {
        ch = prefix[i];
        if (ch == '\0')
        {
          stop = true;
        }
        else
        {
          byte = (uint8_t)ch;
          append_char(dst, cap, pos, byte);
        }
      }
    }
  }
}

static void append_fixed3(uint8_t *dst, uint32_t cap, uint32_t *pos, double value)
{
  uint64_t scaled;
  uint64_t whole;
  uint64_t digit;
  uint32_t frac;
  uint32_t n;
  uint32_t part;
  double mag;
  double scaled_d;
  uint8_t digits[20];
  uint8_t d8;
  bool negative;

  negative = false;
  mag = value;
  if (mag < 0.0)
  {
    negative = true;
    mag = -mag;
  }
  if (mag != mag)
  {
    mag = 0.0;
    negative = false;
  }

  scaled_d = (mag * 1000.0) + 0.5;
  if (scaled_d > 1000000000000000.0)
  {
    scaled_d = 1000000000000000.0;
  }
  scaled = (uint64_t)scaled_d;
  if (scaled == 0ULL)
  {
    negative = false;
  }

  whole = scaled / 1000ULL;
  digit = scaled % 1000ULL;
  frac = (uint32_t)digit;

  if (negative == true)
  {
    append_char(dst, cap, pos, (uint8_t)'-');
  }

  n = 0U;
  if (whole == 0ULL)
  {
    d8 = (uint8_t)0;
    digits[0] = ascii_hex(d8);
    n = 1U;
  }
  else
  {
    while ((whole > 0ULL) && (n < 20U))
    {
      digit = whole % 10ULL;
      whole /= 10ULL;
      d8 = (uint8_t)digit;
      digits[n] = ascii_hex(d8);
      n++;
    }
  }

  while (n > 0U)
  {
    n--;
    append_char(dst, cap, pos, digits[n]);
  }

  append_char(dst, cap, pos, (uint8_t)'.');
  part = frac / 100U;
  d8 = (uint8_t)part;
  append_char(dst, cap, pos, ascii_hex(d8));
  part = (frac / 10U) % 10U;
  d8 = (uint8_t)part;
  append_char(dst, cap, pos, ascii_hex(d8));
  part = frac % 10U;
  d8 = (uint8_t)part;
  append_char(dst, cap, pos, ascii_hex(d8));
}

static double f64_from_float(float value)
{
  double out;

  out = (double)value;
  return out;
}

static void append_float_field(uint8_t *dst, uint32_t cap, uint32_t *pos, double value, bool last)
{
  uint8_t sep;

  append_fixed3(dst, cap, pos, value);
  if (last == true)
  {
    sep = (uint8_t)'*';
  }
  else
  {
    sep = (uint8_t)',';
  }
  append_char(dst, cap, pos, sep);
}

static uint8_t nmea_checksum(const uint8_t *sentence)
{
  uint32_t acc;
  uint32_t i;
  uint8_t byte;
  bool started;
  bool found_star;
  bool stop;

  acc = 0U;
  started = false;
  found_star = false;
  stop = false;
  i = 0U;

  if (sentence != NULL)
  {
    while ((i < NMEA_SENTENCE_CAP) && (stop == false))
    {
      byte = sentence[i];
      if (byte == 0U)
      {
        stop = true;
      }
      else if (byte == (uint8_t)'$')
      {
        started = true;
      }
      else if (byte == (uint8_t)'*')
      {
        found_star = true;
        stop = true;
      }
      else if (started == true)
      {
        acc ^= (uint32_t)byte;
      }
      else
      {
        /* Bytes before '$' are not part of the checksum. */
      }
      i++;
    }
  }

  if ((started == false) || (found_star == false))
  {
    acc = 0U;
  }
  return (uint8_t)acc;
}

static void log_nmea(uint8_t *sentence, uint32_t cap, uint32_t pos)
{
  uint8_t sum;
  uint32_t sum_u;
  uint32_t nibble;
  uint8_t hi;
  uint8_t lo;

  sum = nmea_checksum(sentence);
  sum_u = (uint32_t)sum;
  nibble = sum_u >> 4U;
  nibble &= 0x0FU;
  hi = (uint8_t)nibble;
  nibble = sum_u & 0x0FU;
  lo = (uint8_t)nibble;
  append_char(sentence, cap, &pos, ascii_hex(hi));
  append_char(sentence, cap, &pos, ascii_hex(lo));
  append_char(sentence, cap, &pos, (uint8_t)'\r');
  append_char(sentence, cap, &pos, (uint8_t)'\n');
  if (pos <= 0xFFFFU)
  {
    App_LogWrite(sentence, (uint16_t)pos);
  }
}

static void log_cstring(const char *text)
{
  uint8_t buf[40];
  uint32_t i;
  uint32_t n;
  uint32_t cap;
  char ch;
  bool stop;

  n = 0U;
  stop = false;
  cap = sizeof(buf);
  if (text != NULL)
  {
    for (i = 0U; i < cap; i++)
    {
      if (stop == false)
      {
        ch = text[i];
        if (ch == '\0')
        {
          stop = true;
        }
        else
        {
          buf[n] = (uint8_t)ch;
          n++;
        }
      }
    }
  }

  if ((n > 0U) && (n <= 0xFFFFU))
  {
    App_LogWrite(buf, (uint16_t)n);
  }
}

static float u8_scaled(uint8_t raw)
{
  float value;

  value = (float)raw;
  value = value * CAN_SPEED_SCALE;
  return value;
}

static float u8_as_float(uint8_t raw)
{
  float value;

  value = (float)raw;
  return value;
}

static void note_can_update(void)
{
  if (can_update_count < (uint8_t)255)
  {
    can_update_count++;
  }
}

static uint8_t parse_two_digits(const uint8_t *src)
{
  uint8_t hi;
  uint8_t lo;
  uint8_t value;
  uint32_t acc;
  bool digits;

  value = (uint8_t)0;
  digits = false;
  if (src != NULL)
  {
    hi = src[0];
    lo = src[1];
    if ((hi >= (uint8_t)'0') && (hi <= (uint8_t)'9') &&
        (lo >= (uint8_t)'0') && (lo <= (uint8_t)'9'))
    {
      digits = true;
    }
    if (digits == true)
    {
      acc = (uint32_t)hi;
      acc -= (uint32_t)'0';
      acc *= 10U;
      acc += (uint32_t)lo;
      acc -= (uint32_t)'0';
      value = (uint8_t)acc;
    }
  }
  return value;
}

static uint32_t time2Stamp(const RTC_DateTypeDef *date, const RTC_TimeTypeDef *time)
{
  static const uint16_t mon_days[12] =
  {
    (uint16_t)0, (uint16_t)31, (uint16_t)59, (uint16_t)90,
    (uint16_t)120, (uint16_t)151, (uint16_t)181, (uint16_t)212,
    (uint16_t)243, (uint16_t)273, (uint16_t)304, (uint16_t)334
  };
  uint32_t result;
  uint32_t year_full;
  uint32_t month_index;
  uint32_t leaps;
  uint16_t mdays;
  uint8_t month = (uint8_t)0;
  uint8_t day;
  uint8_t hour_u8;
  uint8_t min_u8;
  uint8_t sec_u8;
  uint8_t year_u8;
  int64_t stamp;
  int64_t yday;
  int64_t hours;
  int64_t mins;
  int64_t secs;
  int64_t leap_days;
  int64_t year_i;
  bool is_leap;
  bool valid;

  result = 0U;
  valid = false;
  if ((date != NULL) && (time != NULL))
  {
    month = date->Month;
    if ((month >= (uint8_t)1) && (month <= (uint8_t)12))
    {
      valid = true;
    }
  }

  if (valid == true)
  {
    year_u8 = date->Year;
    year_full = (uint32_t)year_u8;
    year_full += 2000U;
    month_index = (uint32_t)month;
    month_index -= 1U;

    stamp = (int64_t)year_full;
    stamp -= UNIX_EPOCH_YEAR;
    stamp *= DAYS_PER_YEAR;
    stamp *= SECONDS_PER_DAY;

    mdays = mon_days[month_index];
    day = date->Date;
    yday = (int64_t)mdays;
    yday += (int64_t)day;
    yday -= INT64_C(1);
    yday *= SECONDS_PER_DAY;
    stamp += yday;

    hour_u8 = time->Hours;
    hours = (int64_t)hour_u8;
    hours -= BEIJING_UTC_OFFSET_H;
    stamp += hours * SECONDS_PER_HOUR;

    min_u8 = time->Minutes;
    mins = (int64_t)min_u8;
    stamp += mins * SECONDS_PER_MINUTE;

    sec_u8 = time->Seconds;
    secs = (int64_t)sec_u8;
    stamp += secs;

    is_leap = false;
    if (((year_full % 4U) == 0U) &&
        (((year_full % 100U) != 0U) || ((year_full % 400U) == 0U)))
    {
      is_leap = true;
    }
    if ((month > (uint8_t)2) && (is_leap == true))
    {
      stamp += SECONDS_PER_DAY;
    }

    leaps = year_full - 1969U;
    year_i = (int64_t)leaps;
    leap_days = year_i / INT64_C(4);
    leap_days -= year_i / INT64_C(100);
    leap_days += year_i / INT64_C(400);
    leap_days *= SECONDS_PER_DAY;
    stamp += leap_days;

    if ((stamp >= INT64_C(0)) && (stamp <= STAMP_U32_MAX))
    {
      result = (uint32_t)stamp;
    }
  }

  return result;
}

static void usart3_forward_to_uart2(uint8_t *frame, uint16_t len)
{
  uint8_t crlf[2];

  if ((frame != NULL) && (len > (uint16_t)0))
  {
    crlf[0] = (uint8_t)'\r';
    crlf[1] = (uint8_t)'\n';
    (void)HAL_UART_Transmit(&huart2, frame, len, UART_TX_TIMEOUT_MS);
    (void)HAL_UART_Transmit(&huart2, crlf, (uint16_t)2, UART_TX_TIMEOUT_MS);
  }
}

static void sync_rtc_from_usart3(const uint8_t *frame, uint16_t len)
{
  uint16_t idx;
  uint32_t end_index;
  uint32_t at;
  uint32_t need;
  uint8_t field;
  uint8_t year;

  field = (uint8_t)0;
  if (frame != NULL)
  {
    for (idx = (uint16_t)0; idx < len; idx++)
    {
      if (frame[idx] == (uint8_t)',')
      {
        if (field < (uint8_t)255)
        {
          field++;
        }
        end_index = (uint32_t)idx;
        end_index += 6U;
        need = (uint32_t)len;
        if ((field == NMEA_FIELD_TIME) && (end_index < need))
        {
          at = (uint32_t)idx;
          at += 1U;
          stimestructure.Hours = parse_two_digits(&frame[at]);
          at += 2U;
          stimestructure.Minutes = parse_two_digits(&frame[at]);
          at += 2U;
          stimestructure.Seconds = parse_two_digits(&frame[at]);
        }
        else if ((field == NMEA_FIELD_DATE) && (end_index < need))
        {
          at = (uint32_t)idx;
          at += 1U;
          sdatestructure.Date = parse_two_digits(&frame[at]);
          at += 2U;
          sdatestructure.Month = parse_two_digits(&frame[at]);
          at += 2U;
          sdatestructure.Year = parse_two_digits(&frame[at]);
        }
        else
        {
          /* Other CSV fields are not used for RTC sync. */
        }
      }
    }
  }

  if (osMutexWait(g_telemetryMutex, osWaitForever) == osOK)
  {
    year = sdatestructure.Year;
    if (year != (uint8_t)0)
    {
      (void)HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
      (void)HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
    }
    else
    {
      (void)HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
      (void)HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
    }

    imu_time_offset = 0.0;
    rtc_unix_sec = (double)time2Stamp(&sdatestructure, &stimestructure);
    time_ms = rtc_unix_sec + imu_time_offset;
    USART3_Receive_Flag = false;
    (void)osMutexRelease(g_telemetryMutex);
  }
}

static void handle_usart3_frame(void)
{
  uint8_t frame[USART3_RX_CAPACITY];
  uint16_t len;
  uint16_t i;
  uint32_t count;

  if (USART3_Receive_Flag == true)
  {
    len = USART3_RxFrameLen;
    count = (uint32_t)len;
    if (count >= USART3_RX_CAPACITY)
    {
      count = USART3_RX_CAPACITY - 1U;
      len = (uint16_t)count;
    }
    for (i = (uint16_t)0; i < len; i++)
    {
      frame[i] = USART3_Buff[i];
    }
    frame[len] = 0U;
    usart3_forward_to_uart2(frame, len);
    sync_rtc_from_usart3(frame, len);
  }
}

static void vehicle_can_handle_frame(uint32_t can_id, const uint8_t *payload)
{
  if (payload != NULL)
  {
    switch (can_id)
    {
    case CAN_ID_VEH_SPD_LGT:
      vehicle_data.veh_speed = u8_scaled(payload[CAN_BYTE_VEH_SPD]);
      note_can_update();
      break;
    case CAN_ID_WHL_FRNT:
      vehicle_data.whl_ft_l = u8_scaled(payload[CAN_BYTE_WHL_FL]);
      vehicle_data.whl_ft_r = u8_scaled(payload[CAN_BYTE_WHL_FR]);
      note_can_update();
      break;
    case CAN_ID_WHL_REAR:
      vehicle_data.whl_rr_l = u8_scaled(payload[CAN_BYTE_WHL_RL]);
      vehicle_data.whl_rr_r = u8_scaled(payload[CAN_BYTE_WHL_RR]);
      note_can_update();
      break;
    case CAN_ID_STEER_WHL_AG:
      vehicle_data.steer = u8_as_float(payload[CAN_BYTE_STEER]);
      note_can_update();
      break;
    case CAN_ID_GEAR_LVR:
      vehicle_data.gear = u8_as_float(payload[CAN_BYTE_GEAR]);
      note_can_update();
      break;
    default:
      break;
    }
  }
}

static void publish_grimu_sentence(void)
{
  uint8_t sentence[NMEA_SENTENCE_CAP];
  uint32_t pos;

  nmea_begin(sentence, NMEA_SENTENCE_CAP, &pos, "$GRIMU,");
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, time_ms, false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.acc_x), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.acc_y), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.acc_z), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.gyro_x), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.gyro_y), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.gyro_z), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(scha63x_data.temp_due), true);
  log_nmea(sentence, NMEA_SENTENCE_CAP, pos);
  imu_time_offset += 0.01;
  time_ms = rtc_unix_sec + imu_time_offset;
}

static void publish_grodo_sentence(void)
{
  uint8_t sentence[NMEA_SENTENCE_CAP];
  uint32_t pos;

  nmea_begin(sentence, NMEA_SENTENCE_CAP, &pos, "$GRODO,");
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, time_ms, false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.whl_ft_l), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.whl_ft_r), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.whl_rr_l), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.whl_rr_r), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.veh_speed), false);
  append_float_field(sentence, NMEA_SENTENCE_CAP, &pos, f64_from_float(vehicle_data.gear), true);
  log_nmea(sentence, NMEA_SENTENCE_CAP, pos);
  can_update_count = (uint8_t)0;
}

static void imu_sample_and_publish(void)
{
  scha63x_raw_data raw;
  scha63x_raw_data_summed summed;

  scha63x_read_data(&raw);
  summed.acc_x_lsb = raw.acc_x_lsb;
  summed.acc_y_lsb = raw.acc_y_lsb;
  summed.acc_z_lsb = raw.acc_z_lsb;
  summed.gyro_x_lsb = raw.gyro_x_lsb;
  summed.gyro_y_lsb = raw.gyro_y_lsb;
  summed.gyro_z_lsb = raw.gyro_z_lsb;
  summed.temp_due_lsb = raw.temp_due_lsb;
  summed.temp_uno_lsb = raw.temp_uno_lsb;
  scha63x_convert_data(&summed, &scha63x_data);
  scha63x_cross_axis_compensation(&scha63x_data);

  if (osMutexWait(g_telemetryMutex, osWaitForever) == osOK)
  {
    rtc_unix_sec = (double)time2Stamp(&sdatestructure, &stimestructure);
    time_ms = rtc_unix_sec + imu_time_offset;
    publish_grimu_sentence();
    (void)osMutexRelease(g_telemetryMutex);
  }
}

void data_output(void const *argument)
{
  (void)argument;

  for (;;)
  {
    if (osSemaphoreWait(g_usart3RxSem, 0U) == osOK)
    {
      handle_usart3_frame();
    }

    if (osSemaphoreWait(g_imuTickSem, 0U) == osOK)
    {
      imu_sample_and_publish();
    }
    else
    {
      (void)osDelay(1U);
    }
  }
}

void work_state(void const *argument)
{
  (void)argument;

  for (;;)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    (void)osDelay(500U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    (void)osDelay(500U);
  }
}

void can_com(void const *argument)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  bool publish;
  uint32_t fill;

  (void)argument;

  for (;;)
  {
    publish = false;
    if (osSemaphoreWait(g_canRxSem, osWaitForever) == osOK)
    {
      if (osMutexWait(g_vehicleDataMutex, osWaitForever) == osOK)
      {
        fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
        while (fill > 0U)
        {
          if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
          {
            fill = 0U;
          }
          else
          {
            vehicle_can_handle_frame(rx_header.Identifier, rx_data);
            fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
          }
        }

        if (can_update_count >= GRODO_CAN_UPDATES_MIN)
        {
          publish = true;
        }
        (void)osMutexRelease(g_vehicleDataMutex);
      }
    }

    if (publish == true)
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
  {
    char scha63x_serial_num[14] = { '\0' };
    int32_t imu_status;

    FDCAN1_Config();
    if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK)
    {
      Error_Handler();
    }
    imu_status = scha63x_init(scha63x_serial_num);
    if (imu_status != SCHA63X_OK)
    {
      log_cstring("\r\n IMU INIT fail!\r\n");
    }
    HAL_Delay(1000U);
  }
  /* USER CODE END 2 */

  MX_FREERTOS_Init();
  osKernelStart();
  return 0;
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

static void FDCAN1_Config(void)
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

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  (void)hfdcan;
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
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
  for (;;)
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
