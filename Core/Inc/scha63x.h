#ifndef SCHA63X_H
#define SCHA63X_H

#include <stdint.h>
#include <stdbool.h>

/* 2.6 kHz internal rate; this build publishes each sample at the TIM16 tick. */
#define SCHA63X_X03

#ifdef SCHA63X_X01
#define SENSITIVITY_GYRO_X (160)
#define SENSITIVITY_GYRO_Y (160)
#define SENSITIVITY_GYRO_Z (160)
#elif defined SCHA634_D02
#define SENSITIVITY_GYRO_X (160)
#define SENSITIVITY_GYRO_Y (160)
#define SENSITIVITY_GYRO_Z (80)
#elif defined SCHA63X_X03
#define SENSITIVITY_GYRO_X (80)
#define SENSITIVITY_GYRO_Y (80)
#define SENSITIVITY_GYRO_Z (80)
#endif

#define SENSITIVITY_ACC (4905)

typedef struct
{
    int16_t acc_x_lsb;
    int16_t acc_y_lsb;
    int16_t acc_z_lsb;
    int16_t gyro_x_lsb;
    int16_t gyro_y_lsb;
    int16_t gyro_z_lsb;
    int16_t temp_due_lsb;
    int16_t temp_uno_lsb;
    bool rs_error_due;
    bool rs_error_uno;
} scha63x_raw_data;

typedef struct
{
    int32_t acc_x_lsb;
    int32_t acc_y_lsb;
    int32_t acc_z_lsb;
    int32_t gyro_x_lsb;
    int32_t gyro_y_lsb;
    int32_t gyro_z_lsb;
    int32_t temp_due_lsb;
    int32_t temp_uno_lsb;
} scha63x_raw_data_summed;

/* Physical units. float is a Dir 4.6 deviation for sensor quantities. */
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp_due;
    float temp_uno;
} scha63x_real_data;

typedef struct
{
    float cxx;
    float cxy;
    float cxz;
    float cyx;
    float cyy;
    float cyz;
    float czx;
    float czy;
    float czz;
    float bxx;
    float bxy;
    float bxz;
    float byx;
    float byy;
    float byz;
    float bzx;
    float bzy;
    float bzz;
} scha63x_cacv;

typedef struct
{
    uint16_t summary_status;
    uint16_t rate_status1;
    uint16_t rate_status2;
    uint16_t acc_status1;
    uint16_t common_status1;
    uint16_t common_status2;
} scha63x_sensor_status;

/* Negative values are errors. */
#define SCHA63X_OK                       (0)
#define SCHA63X_ERR_TEST_MODE_ACTIVATION (-1)
#define SCHA63X_ERR_RS_STATUS_NOK        (-2)

int32_t scha63x_init(const char *serial_num);
void scha63x_read_data(scha63x_raw_data *data);
void scha63x_convert_data(const scha63x_raw_data_summed *data_in, scha63x_real_data *data_out);
void scha63x_cross_axis_compensation(scha63x_real_data *data);
void scha63x_read_sensor_status_uno(scha63x_sensor_status *status);
void scha63x_read_sensor_status_due(scha63x_sensor_status *status);

#endif /* SCHA63X_H */
