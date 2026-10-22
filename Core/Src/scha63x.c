/*----------------------------------------------------------------------------

    Driver for SCHA63X sensor.

 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "scha63x.h"



/*----------------------------------------------------------------------------
  Define product type and the sensitivities
 *----------------------------------------------------------------------------*/






// Pre-calculated SPI frames for various operations
//
#define SPI_FRAME_READ_GYRO_X           0x040000F7
#define SPI_FRAME_READ_GYRO_Y           0x0C0000FB
#define SPI_FRAME_READ_GYRO_Z           0x040000F7
#define SPI_FRAME_READ_ACC_X            0x100000E9
#define SPI_FRAME_READ_ACC_Y            0x140000EF
#define SPI_FRAME_READ_ACC_Z            0x180000E5
#define SPI_FRAME_READ_TEMP             0x1C0000E3

#define SPI_FRAME_READ_SUMMARY_STATUS   0x380000D5
#define SPI_FRAME_READ_RATE_STATUS_1    0x40000091
#define SPI_FRAME_READ_RATE_STATUS_2    0x44000097
#define SPI_FRAME_READ_COMMON_STATUS_1  0x50000089
#define SPI_FRAME_READ_COMMON_STATUS_2  0x5400008F
#define SPI_FRAME_READ_ACC_STATUS_1     0x4800009D

#define SPI_FRAME_WRITE_RESET           0xE000017C
#define SPI_FRAME_WRITE_REG_BANK_0      0xFC000073 // Select register bank 0
#define SPI_FRAME_READ_TRC_0            0x740000BF // Traceability 0 register 1Dh
#define SPI_FRAME_READ_TRC_1            0x780000B5 // Traceability 1 register 1Eh
#define SPI_FRAME_READ_TRC_2            0x700000B9 // Traceability 2 Register 1Ch

#define SPI_FRAME_WRITE_OP_MODE_NORMAL   0xE4000067 // Set normal operating mode (register 19h)
#define SPI_FRAME_WRITE_FILTER_46HZ_RATE 0xD812129E // Set 46 Hz filter for rate (register 16h)
#define SPI_FRAME_WRITE_FILTER_46HZ_ACC  0xE8022248 // Set 46 Hz filter for acceleration (register 1Ah)
#define SPI_FRAME_WRITE_EOI_BIT          0xE000025B

// Frames needed for test mode activation. Mode register address: 19h
#define SPI_FRAME_READ_MODE             0x640000A7
#define SPI_FRAME_WRITE_MODE_ASM_010    0xE40010AA  // Unlock_ASM[2:0] = 010
#define SPI_FRAME_WRITE_MODE_ASM_001    0xE400088F  // Unlock_ASM[2:0] = 001
#define SPI_FRAME_WRITE_MODE_ASM_100    0xE40020E0  // Unlock_ASM[2:0] = 100


/**

  Static function prototypes

**/
static bool scha63x_check_init_due(void);
static bool scha63x_check_init_uno(void);
static bool scha63x_check_rs_error(uint32_t *data, int size);
static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3);
static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                     uint32_t miso_4, uint32_t miso_5);


/**

  Internal data structures

**/
static scha63x_cacv scha63x_cac_values; // Cross-axis compensation values



void Wait_ms(int time)
{
	HAL_Delay(time);
}
/**

    Init SCHA63X sensor

    Parameters:         serial_num - string where to read component serial number. Minimum size 14 bytes.

    Return value:       SCHA63X_OK = success, SCHA63X_ERR_* = failure, SCHA63X_WRN_* = warning. Please see header file for definitions.

**/
uint32_t resp;
uint16_t trc_2;
uint16_t trc_0;
uint16_t trc_1;

int scha63x_init(char *serial_num)
{
    bool status_DUE = false;
    bool status_UNO = false;
    int attempt;
    int const num_attempts = 2;

    (void)serial_num;

    // HW Reset (reset via SPI)
    SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);
    SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);   // Make sure we are in bank 0, otherwise SPI reset is not available.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);

    // Wait 25 ms for the non-volatile memory (NVM) Read
    Wait_ms(25);

    // Read UNO asic serial number
    SPI_ASIC_UNO(SPI_FRAME_READ_TRC_2);
    trc_2 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_0));
    trc_0 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1));
    trc_1 = SPI_DATA_UINT16(SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1));

    (void)trc_0;
    (void)trc_1;
    (void)trc_2;

    // Activate DUE asic test mode to be able to read cross-axis
    // compensation values from DUE NVM.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_010);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_001);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_100);
    SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    resp = SPI_ASIC_DUE(SPI_FRAME_READ_MODE);

    // Check if device is correctly set to test mode
    if ((SPI_DATA_UINT16(resp) & 0x7) == 7) {

        // Read cross-axis compensation values
        SPI_ASIC_DUE(0xFC00051A);
        SPI_ASIC_DUE(0x2C0000CB);
        uint32_t cxx_cxy = SPI_ASIC_DUE(0x4C00009B);
        uint32_t cxz_cyx = SPI_ASIC_DUE(0x50000089);
        uint32_t cyy_cyz = SPI_ASIC_DUE(0x5400008F);
        uint32_t czx_czy = SPI_ASIC_DUE(0x58000085);
        uint32_t czz_bxx = SPI_ASIC_DUE(0x5C000083);
        uint32_t bxy_bxz = SPI_ASIC_DUE(0x600000A1);
        uint32_t byx_byy = SPI_ASIC_DUE(0x6C0000AB);
        uint32_t byz_bzx = SPI_ASIC_DUE(0x700000B9);
        uint32_t bzy_bzz = SPI_ASIC_DUE(0x700000B9);

        scha63x_cac_values.cxx = SPI_DATA_INT8_LOWER(cxx_cxy) / 4096.0 + 1;
        scha63x_cac_values.cxy = SPI_DATA_INT8_UPPER(cxx_cxy) / 4096.0;
        scha63x_cac_values.cxz = SPI_DATA_INT8_LOWER(cxz_cyx) / 4096.0;
        scha63x_cac_values.cyx = SPI_DATA_INT8_UPPER(cxz_cyx) / 4096.0;
        scha63x_cac_values.cyy = SPI_DATA_INT8_LOWER(cyy_cyz) / 4096.0 + 1;
        scha63x_cac_values.cyz = SPI_DATA_INT8_UPPER(cyy_cyz) / 4096.0;
        scha63x_cac_values.czx = SPI_DATA_INT8_LOWER(czx_czy) / 4096.0;
        scha63x_cac_values.czy = SPI_DATA_INT8_UPPER(czx_czy) / 4096.0;
        scha63x_cac_values.czz = SPI_DATA_INT8_LOWER(czz_bxx) / 4096.0 + 1;
        scha63x_cac_values.bxx = SPI_DATA_INT8_UPPER(czz_bxx) / 4096.0 + 1;
        scha63x_cac_values.bxy = SPI_DATA_INT8_LOWER(bxy_bxz) / 4096.0;
        scha63x_cac_values.bxz = SPI_DATA_INT8_UPPER(bxy_bxz) / 4096.0;
        scha63x_cac_values.byx = SPI_DATA_INT8_LOWER(byx_byy) / 4096.0;
        scha63x_cac_values.byy = SPI_DATA_INT8_UPPER(byx_byy) / 4096.0 + 1;
        scha63x_cac_values.byz = SPI_DATA_INT8_LOWER(byz_bzx) / 4096.0;
        scha63x_cac_values.bzx = SPI_DATA_INT8_UPPER(byz_bzx) / 4096.0;
        scha63x_cac_values.bzy = SPI_DATA_INT8_LOWER(bzy_bzz) / 4096.0;
        scha63x_cac_values.bzz = SPI_DATA_INT8_UPPER(bzy_bzz) / 4096.0 + 1;
    }
    else {
        // Return error if test mode can not be activated
        return SCHA63X_ERR_TEST_MODE_ACTIVATION;
    }


    // HW Reset to get DUE asic out from test mode (reset via SPI).
    SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);   // Return to bank 0 to make SPI reset command available.
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);        // Reset DUE after reading cross-axis registers


    // Start UNO & DUE
    Wait_ms(25);                                // Wait 25ms for the non-volatile memory (NVM) Read

    // DUE asic initial startup
    SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set UNO operation mode on
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set DUE operation mode on twice
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
    Wait_ms(70);                                        // Wait minimum 70ms (includes UNO 50ms 'SPI accessible' wait)

    SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);     // Select UNO 46Hz filter for RATE
    SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC);      // Select UNO 46Hz filter for ACC

    // Restart DUE
    SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);                // Reset DUE again
    Wait_ms(25);                                        // Wait 25 ms for the NVM read

    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // Set DUE operation mode
    SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);       // DUE operation mode must be set twice

    Wait_ms(1);                                         // Wait 1 ms for SPI to be accesible
    SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);     // Select DUE 46Hz filter for RATE

    for (attempt = 0; attempt < num_attempts; attempt++) {

        // Wait 405 ms (Gyro and ACC start up)
        Wait_ms(405);

        // Set EOI=1 (End of initialization command)
        SPI_ASIC_UNO(SPI_FRAME_WRITE_EOI_BIT);          // Set EOI bit for UNO
        SPI_ASIC_DUE(SPI_FRAME_WRITE_EOI_BIT);          // Set EOI bit for DUE

        // Check initialization status from the summary status registers
        status_UNO = scha63x_check_init_uno();
        status_DUE = scha63x_check_init_due();

        // Check if restart is needed
        if ((status_UNO == false || status_DUE == false) && attempt < (num_attempts - 1)) {
            SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);
            SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);

            Wait_ms(25);                                   // Wait 25 ms for the NVM read
            SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);  // Set UNO operation mode on
            SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);  // Set DUE operation mode on twice
            SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
            Wait_ms(50);                                   // Wait 50ms before communicating with UNO
            SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);// Select UNO 46Hz filter for RATE
            SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC); // Select UNO 46Hz filter for ACC
            SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);// Select DUE 46Hz filter for RATE
            Wait_ms(45);                                   // Adjust restart duration to 500 ms
        }
        else {
            break;
        }
    }

    if (status_DUE == false || status_UNO == false) {
        return SCHA63X_ERR_RS_STATUS_NOK;
    }

    return SCHA63X_OK;

}


/**

    Check initialization success of SCHA63X sensor DUE asic

    Return value:       true = success, false = error

**/
static bool scha63x_check_init_due(void)
{
    uint32_t resp;
    bool due_ok;

    // Read summary status two times (first time may show incorrectly FAIL after start-up)
    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    Wait_ms(3);
    resp = SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    due_ok = SPI_DATA_CHECK_RS_ERROR(resp) == true ? false: true;

    return due_ok;
}

/**

    Check initialization success of SCHA63X sensor UNO asic

    Return value:       true = success, false = error

**/
static bool scha63x_check_init_uno(void)
{
    uint32_t resp;
    bool uno_ok;

    // Read summary status two times (first time may show incorrectly FAIL after start-up)
    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    Wait_ms(3);
    resp = SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    uno_ok = SPI_DATA_CHECK_RS_ERROR(resp) == true ? false: true;

    return uno_ok;
}


/**

    Read acceleration, rate and temperature data from sensor.

    SPI responses are pipelined: each transfer returns the MISO word for the
    previous command. The variable names match the register that was requested
    one frame earlier on that ASIC.

    Parameters:         *data               - pointer to "raw" data from sensor

    Return value:       None

**/
void scha63x_read_data(scha63x_raw_data *data)
{
    SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Y);
    uint32_t gyro_y_lsb   = SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Z);
    uint32_t gyro_z_lsb   = SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);
    uint32_t temp_due_lsb = SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);

    SPI_ASIC_UNO(SPI_FRAME_READ_GYRO_X);
    uint32_t gyro_x_lsb   = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_X);
    uint32_t acc_x_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Y);
    uint32_t acc_y_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Z);
    uint32_t acc_z_lsb    = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);
    uint32_t temp_uno_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);

    // Get possible errors
    data->rs_error_due = scha63x_check_rs_error_3(gyro_y_lsb, gyro_z_lsb, temp_due_lsb);
    data->rs_error_uno = scha63x_check_rs_error_5(gyro_x_lsb, acc_x_lsb, acc_y_lsb, acc_z_lsb, temp_uno_lsb);

    // Parse MISO data to structure
    data->acc_x_lsb = SPI_DATA_INT16(acc_x_lsb);
    data->acc_y_lsb = SPI_DATA_INT16(acc_y_lsb);
    data->acc_z_lsb = SPI_DATA_INT16(acc_z_lsb);
    data->gyro_x_lsb = SPI_DATA_INT16(gyro_x_lsb);
    data->gyro_y_lsb = SPI_DATA_INT16(gyro_y_lsb);
    data->gyro_z_lsb = SPI_DATA_INT16(gyro_z_lsb);
    data->temp_due_lsb = SPI_DATA_INT16(temp_due_lsb);
    data->temp_uno_lsb = SPI_DATA_INT16(temp_uno_lsb);

}


/**

    Check if MISO frames have RS error bits set

    Parameters:         *data       - pointer to 32-bit MISO frames from sensor
                        size        - number of frames to check

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error(uint32_t *data, int size)
{
    for (int i = 0; i < size; i++)
    {
        uint32_t value = data[i];
        if (SPI_DATA_CHECK_RS_ERROR(value))
        {
            return true;
        }
    }

    return false;
}


/**

    Check if any of three MISO frames have RS error bits set

    Parameters:         miso_1 .. miso_3       - MISO words 1 .. 3

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3)
{
    uint32_t miso_words[] = { miso_1, miso_2, miso_3 };
    return scha63x_check_rs_error(miso_words, (sizeof(miso_words) / sizeof(uint32_t)));
}


/**

    Check if any of five MISO frames have RS error bits set

    Parameters:         miso_1 .. miso_5       - MISO words 1 .. 5

    Return value:       true  = RS error bits set
                        false = no RS error

**/
static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                  uint32_t miso_4, uint32_t miso_5)
{
    uint32_t miso_words[] = { miso_1, miso_2, miso_3, miso_4, miso_5 };
    return scha63x_check_rs_error(miso_words, (sizeof(miso_words) / sizeof(uint32_t)));
}


/**

    Convert raw (or summed) LSB values from the sensor to physical units.

    Parameters:         *data_in    - pointer to summed "raw" data from sensor
                        *data_out   - pointer to converted values

    Return value:       None

*/
void scha63x_convert_data(scha63x_raw_data_summed *data_in, scha63x_real_data *data_out)
{
    data_out->acc_x = data_in->acc_x_lsb;
    data_out->acc_y = data_in->acc_y_lsb;
    data_out->acc_z = data_in->acc_z_lsb;
    data_out->gyro_x = data_in->gyro_x_lsb;
    data_out->gyro_y = data_in->gyro_y_lsb;
    data_out->gyro_z = data_in->gyro_z_lsb;

    /* Divide by per-axis sensitivity (see scha63x.h). */
//    data_out->acc_x = data_out->acc_x / (SENSITIVITY_ACC * SAMPLE_COUNT);
//    data_out->acc_y = data_out->acc_y / (SENSITIVITY_ACC * SAMPLE_COUNT);
//    data_out->acc_z = data_out->acc_z / (SENSITIVITY_ACC * SAMPLE_COUNT);
//    data_out->gyro_x = data_out->gyro_x / (SENSITIVITY_GYRO_X * SAMPLE_COUNT);
//    data_out->gyro_y = data_out->gyro_y / (SENSITIVITY_GYRO_Y * SAMPLE_COUNT);
//    data_out->gyro_z = data_out->gyro_z / (SENSITIVITY_GYRO_Z * SAMPLE_COUNT);
//
//    // Convert temperatures and calculate averages
//    data_out->temp_due = GET_TEMPERATURE(data_in->temp_due_lsb / SAMPLE_COUNT);
//    data_out->temp_uno = GET_TEMPERATURE(data_in->temp_uno_lsb / SAMPLE_COUNT);


		data_out->acc_x = data_out->acc_x / (SENSITIVITY_ACC);
    data_out->acc_y = data_out->acc_y / (SENSITIVITY_ACC);
    data_out->acc_z = data_out->acc_z / (SENSITIVITY_ACC);
    data_out->gyro_x = data_out->gyro_x / (SENSITIVITY_GYRO_X);
    data_out->gyro_y = data_out->gyro_y / (SENSITIVITY_GYRO_Y);
    data_out->gyro_z = data_out->gyro_z / (SENSITIVITY_GYRO_Z);

    // Convert temperatures and calculate averages
    data_out->temp_due = GET_TEMPERATURE(data_in->temp_due_lsb);
    data_out->temp_uno = GET_TEMPERATURE(data_in->temp_uno_lsb);
}


/**

    Calculate cross-axis compensation

    Parameters:         *data   - pointer to sensor data

    Return value:       None

**/
void scha63x_cross_axis_compensation(scha63x_real_data *data)
{
    float acc_x_comp, acc_y_comp, acc_z_comp;
    float gyro_x_comp, gyro_y_comp, gyro_z_comp;

    acc_x_comp = (scha63x_cac_values.bxx * data->acc_x) + (scha63x_cac_values.bxy * data->acc_y) + (scha63x_cac_values.bxz * data->acc_z);
    acc_y_comp = (scha63x_cac_values.byx * data->acc_x) + (scha63x_cac_values.byy * data->acc_y) + (scha63x_cac_values.byz * data->acc_z);
    acc_z_comp = (scha63x_cac_values.bzx * data->acc_x) + (scha63x_cac_values.bzy * data->acc_y) + (scha63x_cac_values.bzz * data->acc_z);
    gyro_x_comp = (scha63x_cac_values.cxx * data->gyro_x) + (scha63x_cac_values.cxy * data->gyro_y) + (scha63x_cac_values.cxz * data->gyro_z);
    gyro_y_comp = (scha63x_cac_values.cyx * data->gyro_x) + (scha63x_cac_values.cyy * data->gyro_y) + (scha63x_cac_values.cyz * data->gyro_z);
    gyro_z_comp = (scha63x_cac_values.czx * data->gyro_x) + (scha63x_cac_values.czy * data->gyro_y) + (scha63x_cac_values.czz * data->gyro_z);

    data->acc_x = acc_x_comp;
    data->acc_y = acc_y_comp;
    data->acc_z = acc_z_comp;
    data->gyro_x = gyro_x_comp;
    data->gyro_y = gyro_y_comp;
    data->gyro_z = gyro_z_comp;
}


/**

    Read sensor status from UNO ASIC

    Parameters:         *status    - pointer to sensor status structure where the data is read to

    Return value:       None

**/
void scha63x_read_sensor_status_uno(scha63x_sensor_status *status)
{

    SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    uint32_t sum_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_RATE_STATUS_1);
    uint32_t rat_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_STATUS_1);
    uint32_t acc_stat   = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_1);
    uint32_t com_stat_1 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);
    uint32_t com_stat_2 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);

    status->summary_status = SPI_DATA_UINT16(sum_stat);
    status->rate_status1 = SPI_DATA_UINT16(rat_stat);
    status->acc_status1 = SPI_DATA_UINT16(acc_stat);
    status->common_status1 = SPI_DATA_UINT16(com_stat_1);
    status->common_status2 = SPI_DATA_UINT16(com_stat_2);

}


/**

    Read sensor status from DUE ASIC

    Parameters:         *status    - pointer to sensor status structure where the data is read to

    Return value:       None

**/
void scha63x_read_sensor_status_due(scha63x_sensor_status *status)
{

    SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    uint32_t sum_stat   = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_1);
    uint32_t rat_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_2);
    uint32_t rat_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_1);
    uint32_t com_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);
    uint32_t com_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);

    status->summary_status = SPI_DATA_UINT16(sum_stat);
    status->rate_status1 = SPI_DATA_UINT16(rat_stat_1);
    status->rate_status2 = SPI_DATA_UINT16(rat_stat_2);
    status->common_status1 = SPI_DATA_UINT16(com_stat_1);
    status->common_status2 = SPI_DATA_UINT16(com_stat_2);

}


/*----------------------------------------------------------------------------
  SPI communication
 *----------------------------------------------------------------------------*/
uint8_t buf[10] = {0};
uint8_t result;
uint8_t rxbuf[10] = {0};
static uint32_t SPI_ASIC(uint32_t dout, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    uint32_t resp;
    uint32_t data_out;

    // SCHA63X data frame length is 32 bits and the data must be sent out MSB first.
    // Function SPI_Transfer() does not support sending 32 bit data so the frame is divided to two 16 bit
    // transfers (SPI is configured to 16 bit mode in SPI_Initialize() earlier) and software CS is used
    // to hold the CS low during the whole transfer sequence.

    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Set CS active

    // Swap 16-bit word order of data to send. By default the SPI_Transfer() sends the lower 16 bit word first
    // so the word order is swapped here to comply with MSB first requirement.
    data_out =  (dout >> 16) & 0x0000ffff;
    data_out |= (dout << 16) & 0xffff0000;

//    SPI_Transfer(&data_out, &resp, 2);  // Transfer (write/read) of two 16 bit words.
//    while (!SPI_GetStatus()) {}

		buf[3] = dout & 0x000000ff;
		buf[2] = (dout & 0x0000ff00)>>8;
		buf[1] = (dout & 0x00ff0000)>>16;
		buf[0] = (dout & 0xff000000)>>24;

	  result = HAL_SPI_TransmitReceive(&hspi2, buf, rxbuf, 4, 0xffff);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);   // Set CS inactive

    // Collect 32-bit response frame from the received 16-bit words.
    //resp = ((resp << 16) & 0xffff0000) | ((resp >> 16) & 0x0000ffff);
    resp = rxbuf[3]|(rxbuf[2]<<8)|(rxbuf[1]<<16)|(rxbuf[0]<<24);
    return resp;
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC1 (DUE) using CS1
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_DUE(uint32_t dout)
{
    return SPI_ASIC(dout, GPIOG, GPIO_PIN_12);
}


/*----------------------------------------------------------------------------
  SPI communication to ASIC2 (UNO) using CS2
 *----------------------------------------------------------------------------*/
uint32_t SPI_ASIC_UNO(uint32_t dout)
{
    return SPI_ASIC(dout, GPIOG, GPIO_PIN_10);
}

