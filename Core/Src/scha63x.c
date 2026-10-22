#include "main.h"
#include "scha63x.h"
#include "gpio.h"
#include "spi.h"

/* SPI frames. U suffix keeps them essentially unsigned (Rules 7.2, 10.3). */
#define SPI_FRAME_READ_GYRO_X            (0x040000F7U)
#define SPI_FRAME_READ_GYRO_Y            (0x0C0000FBU)
#define SPI_FRAME_READ_GYRO_Z            (0x040000F7U)
#define SPI_FRAME_READ_ACC_X             (0x100000E9U)
#define SPI_FRAME_READ_ACC_Y             (0x140000EFU)
#define SPI_FRAME_READ_ACC_Z             (0x180000E5U)
#define SPI_FRAME_READ_TEMP              (0x1C0000E3U)
#define SPI_FRAME_READ_SUMMARY_STATUS    (0x380000D5U)
#define SPI_FRAME_READ_RATE_STATUS_1     (0x40000091U)
#define SPI_FRAME_READ_RATE_STATUS_2     (0x44000097U)
#define SPI_FRAME_READ_COMMON_STATUS_1   (0x50000089U)
#define SPI_FRAME_READ_COMMON_STATUS_2   (0x5400008FU)
#define SPI_FRAME_READ_ACC_STATUS_1      (0x4800009DU)
#define SPI_FRAME_WRITE_RESET            (0xE000017CU)
#define SPI_FRAME_WRITE_REG_BANK_0       (0xFC000073U)
#define SPI_FRAME_READ_TRC_0             (0x740000BFU)
#define SPI_FRAME_READ_TRC_1             (0x780000B5U)
#define SPI_FRAME_READ_TRC_2             (0x700000B9U)
#define SPI_FRAME_WRITE_OP_MODE_NORMAL   (0xE4000067U)
#define SPI_FRAME_WRITE_FILTER_46HZ_RATE (0xD812129EU)
#define SPI_FRAME_WRITE_FILTER_46HZ_ACC  (0xE8022248U)
#define SPI_FRAME_WRITE_EOI_BIT          (0xE000025BU)
#define SPI_FRAME_READ_MODE              (0x640000A7U)
#define SPI_FRAME_WRITE_MODE_ASM_010     (0xE40010AAU)
#define SPI_FRAME_WRITE_MODE_ASM_001     (0xE400088FU)
#define SPI_FRAME_WRITE_MODE_ASM_100     (0xE40020E0U)

#define SCHA63X_CAC_FRAME_BANK           (0xFC00051AU)
#define SCHA63X_CAC_FRAME_REQ            (0x2C0000CBU)
#define SCHA63X_CAC_FRAME_CXX_CXY        (0x4C00009BU)
#define SCHA63X_CAC_FRAME_CXZ_CYX        (0x50000089U)
#define SCHA63X_CAC_FRAME_CYY_CYZ        (0x5400008FU)
#define SCHA63X_CAC_FRAME_CZX_CZY        (0x58000085U)
#define SCHA63X_CAC_FRAME_CZZ_BXX        (0x5C000083U)
#define SCHA63X_CAC_FRAME_BXY_BXZ        (0x600000A1U)
#define SCHA63X_CAC_FRAME_BYX_BYY        (0x6C0000ABU)
#define SCHA63X_CAC_FRAME_BYZ_BZX        (0x700000B9U)
#define SCHA63X_CAC_FRAME_BZY_BZZ        (0x700000B9U)

#define SCHA63X_RS_SHIFT                 (24U)
#define SCHA63X_RS_MASK                  (0x03U)
#define SCHA63X_RS_OK                    (1U)
#define SCHA63X_TEST_MODE_MASK           (0x7U)
#define SCHA63X_TEST_MODE_OK             (7U)
#define SCHA63X_CAC_DIVISOR              (4096.0f)
#define SCHA63X_CAC_BIAS                 (1.0f)
#define SCHA63X_TEMP_LSB                 (30.0f)
#define SCHA63X_TEMP_OFFSET              (25.0f)
#define SCHA63X_NVM_WAIT_MS              (25U)
#define SCHA63X_STARTUP_WAIT_MS          (70U)
#define SCHA63X_SPI_WAIT_MS              (1U)
#define SCHA63X_GYRO_START_MS            (405U)
#define SCHA63X_STATUS_WAIT_MS           (3U)
#define SCHA63X_RESTART_UNO_MS           (50U)
#define SCHA63X_RESTART_TAIL_MS          (45U)
#define SCHA63X_INIT_ATTEMPTS            (2U)
#define SCHA63X_SPI_TIMEOUT_MS           (0xFFFFU)

static scha63x_cacv s_scha63x_cac;

static void Wait_ms(uint32_t delay_ms);
static bool scha63x_miso_rs_error(uint32_t miso);
static int8_t scha63x_miso_int8_upper(uint32_t miso);
static int8_t scha63x_miso_int8_lower(uint32_t miso);
static int16_t scha63x_miso_int16(uint32_t miso);
static uint16_t scha63x_miso_uint16(uint32_t miso);
static float scha63x_cac_coeff(int8_t raw, bool add_one);
static float scha63x_celsius(int32_t lsb);
static float scha63x_lsb_to_unit(int32_t lsb, int32_t sensitivity);
static bool scha63x_check_init_due(void);
static bool scha63x_check_init_uno(void);
static bool scha63x_check_rs_error(const uint32_t *data, uint32_t count);
static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3);
static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                     uint32_t miso_4, uint32_t miso_5);
static void scha63x_clear_status(scha63x_sensor_status *status);
static void scha63x_load_cac(void);
static uint32_t SPI_ASIC(uint32_t dout, GPIO_TypeDef *gpio, uint16_t pin);
static uint32_t SPI_ASIC_DUE(uint32_t dout);
static uint32_t SPI_ASIC_UNO(uint32_t dout);

static void Wait_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

static bool scha63x_miso_rs_error(uint32_t miso)
{
    uint32_t rs;
    bool error;

    rs = miso >> SCHA63X_RS_SHIFT;
    rs &= SCHA63X_RS_MASK;
    error = false;
    if (rs != SCHA63X_RS_OK)
    {
        error = true;
    }
    return error;
}

static int8_t scha63x_miso_int8_upper(uint32_t miso)
{
    uint32_t field;
    uint8_t byte;
    int8_t value;

    field = miso >> 16U;
    field &= 0xFFU;
    byte = (uint8_t)field;
    value = (int8_t)byte;
    return value;
}

static int8_t scha63x_miso_int8_lower(uint32_t miso)
{
    uint32_t field;
    uint8_t byte;
    int8_t value;

    field = miso >> 8U;
    field &= 0xFFU;
    byte = (uint8_t)field;
    value = (int8_t)byte;
    return value;
}

static int16_t scha63x_miso_int16(uint32_t miso)
{
    uint32_t field;
    uint16_t bits;
    int16_t value;

    field = miso >> 8U;
    field &= 0xFFFFU;
    bits = (uint16_t)field;
    value = (int16_t)bits;
    return value;
}

static uint16_t scha63x_miso_uint16(uint32_t miso)
{
    uint32_t field;
    uint16_t bits;

    field = miso >> 8U;
    field &= 0xFFFFU;
    bits = (uint16_t)field;
    return bits;
}

static float scha63x_cac_coeff(int8_t raw, bool add_one)
{
    float value;

    value = (float)raw;
    value = value / SCHA63X_CAC_DIVISOR;
    if (add_one == true)
    {
        value = value + SCHA63X_CAC_BIAS;
    }
    return value;
}

static float scha63x_celsius(int32_t lsb)
{
    float value;

    value = (float)lsb;
    value = value / SCHA63X_TEMP_LSB;
    value = value + SCHA63X_TEMP_OFFSET;
    return value;
}

static float scha63x_lsb_to_unit(int32_t lsb, int32_t sensitivity)
{
    float value;
    float scale;

    value = (float)lsb;
    scale = (float)sensitivity;
    value = value / scale;
    return value;
}

static void scha63x_clear_status(scha63x_sensor_status *status)
{
    status->summary_status = 0U;
    status->rate_status1 = 0U;
    status->rate_status2 = 0U;
    status->acc_status1 = 0U;
    status->common_status1 = 0U;
    status->common_status2 = 0U;
}

static void scha63x_load_cac(void)
{
    uint32_t cxx_cxy;
    uint32_t cxz_cyx;
    uint32_t cyy_cyz;
    uint32_t czx_czy;
    uint32_t czz_bxx;
    uint32_t bxy_bxz;
    uint32_t byx_byy;
    uint32_t byz_bzx;
    uint32_t bzy_bzz;
    int8_t raw;

    (void)SPI_ASIC_DUE(SCHA63X_CAC_FRAME_BANK);
    (void)SPI_ASIC_DUE(SCHA63X_CAC_FRAME_REQ);
    cxx_cxy = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_CXX_CXY);
    cxz_cyx = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_CXZ_CYX);
    cyy_cyz = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_CYY_CYZ);
    czx_czy = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_CZX_CZY);
    czz_bxx = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_CZZ_BXX);
    bxy_bxz = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_BXY_BXZ);
    byx_byy = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_BYX_BYY);
    byz_bzx = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_BYZ_BZX);
    bzy_bzz = SPI_ASIC_DUE(SCHA63X_CAC_FRAME_BZY_BZZ);

    raw = scha63x_miso_int8_lower(cxx_cxy);
    s_scha63x_cac.cxx = scha63x_cac_coeff(raw, true);
    raw = scha63x_miso_int8_upper(cxx_cxy);
    s_scha63x_cac.cxy = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(cxz_cyx);
    s_scha63x_cac.cxz = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(cxz_cyx);
    s_scha63x_cac.cyx = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(cyy_cyz);
    s_scha63x_cac.cyy = scha63x_cac_coeff(raw, true);
    raw = scha63x_miso_int8_upper(cyy_cyz);
    s_scha63x_cac.cyz = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(czx_czy);
    s_scha63x_cac.czx = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(czx_czy);
    s_scha63x_cac.czy = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(czz_bxx);
    s_scha63x_cac.czz = scha63x_cac_coeff(raw, true);
    raw = scha63x_miso_int8_upper(czz_bxx);
    s_scha63x_cac.bxx = scha63x_cac_coeff(raw, true);
    raw = scha63x_miso_int8_lower(bxy_bxz);
    s_scha63x_cac.bxy = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(bxy_bxz);
    s_scha63x_cac.bxz = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(byx_byy);
    s_scha63x_cac.byx = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(byx_byy);
    s_scha63x_cac.byy = scha63x_cac_coeff(raw, true);
    raw = scha63x_miso_int8_lower(byz_bzx);
    s_scha63x_cac.byz = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(byz_bzx);
    s_scha63x_cac.bzx = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_lower(bzy_bzz);
    s_scha63x_cac.bzy = scha63x_cac_coeff(raw, false);
    raw = scha63x_miso_int8_upper(bzy_bzz);
    s_scha63x_cac.bzz = scha63x_cac_coeff(raw, true);
}

int32_t scha63x_init(const char *serial_num)
{
    int32_t result;
    uint32_t resp;
    uint32_t mode_word;
    uint32_t attempt;
    uint16_t mode_bits;
    bool test_mode_ok;
    bool status_due;
    bool status_uno;
    bool restart;

    (void)serial_num;
    result = SCHA63X_OK;
    test_mode_ok = false;
    status_due = false;
    status_uno = false;

    (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);
    (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);
    (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);
    Wait_ms(SCHA63X_NVM_WAIT_MS);

    /* Traceability reads are pipelined; the values are not used. */
    (void)SPI_ASIC_UNO(SPI_FRAME_READ_TRC_2);
    (void)SPI_ASIC_UNO(SPI_FRAME_READ_TRC_0);
    (void)SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1);
    (void)SPI_ASIC_UNO(SPI_FRAME_READ_TRC_1);

    (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_010);
    (void)SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_001);
    (void)SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_MODE_ASM_100);
    (void)SPI_ASIC_DUE(SPI_FRAME_READ_MODE);
    resp = SPI_ASIC_DUE(SPI_FRAME_READ_MODE);

    mode_bits = scha63x_miso_uint16(resp);
    mode_word = (uint32_t)mode_bits;
    mode_word &= SCHA63X_TEST_MODE_MASK;
    if (mode_word == SCHA63X_TEST_MODE_OK)
    {
        test_mode_ok = true;
        scha63x_load_cac();
    }

    if (test_mode_ok == false)
    {
        result = SCHA63X_ERR_TEST_MODE_ACTIVATION;
    }
    else
    {
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_REG_BANK_0);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);
        Wait_ms(SCHA63X_NVM_WAIT_MS);

        (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
        Wait_ms(SCHA63X_STARTUP_WAIT_MS);

        (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);
        (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);
        Wait_ms(SCHA63X_NVM_WAIT_MS);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
        Wait_ms(SCHA63X_SPI_WAIT_MS);
        (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);

        for (attempt = 0U; attempt < SCHA63X_INIT_ATTEMPTS; attempt++)
        {
            Wait_ms(SCHA63X_GYRO_START_MS);
            (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_EOI_BIT);
            (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_EOI_BIT);
            status_uno = scha63x_check_init_uno();
            status_due = scha63x_check_init_due();

            restart = false;
            if (((status_uno == false) || (status_due == false)) &&
                (attempt < (SCHA63X_INIT_ATTEMPTS - 1U)))
            {
                restart = true;
            }

            if (restart == true)
            {
                (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_RESET);
                (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_RESET);
                Wait_ms(SCHA63X_NVM_WAIT_MS);
                (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_OP_MODE_NORMAL);
                (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
                (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_OP_MODE_NORMAL);
                Wait_ms(SCHA63X_RESTART_UNO_MS);
                (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_RATE);
                (void)SPI_ASIC_UNO(SPI_FRAME_WRITE_FILTER_46HZ_ACC);
                (void)SPI_ASIC_DUE(SPI_FRAME_WRITE_FILTER_46HZ_RATE);
                Wait_ms(SCHA63X_RESTART_TAIL_MS);
            }
            else
            {
                break;
            }
        }

        if ((status_due == false) || (status_uno == false))
        {
            result = SCHA63X_ERR_RS_STATUS_NOK;
        }
    }

    return result;
}

static bool scha63x_check_init_due(void)
{
    uint32_t resp;
    bool due_ok;

    (void)SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    (void)SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    Wait_ms(SCHA63X_STATUS_WAIT_MS);
    resp = SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
    due_ok = true;
    if (scha63x_miso_rs_error(resp) == true)
    {
        due_ok = false;
    }
    return due_ok;
}

static bool scha63x_check_init_uno(void)
{
    uint32_t resp;
    bool uno_ok;

    (void)SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    (void)SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    Wait_ms(SCHA63X_STATUS_WAIT_MS);
    resp = SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
    uno_ok = true;
    if (scha63x_miso_rs_error(resp) == true)
    {
        uno_ok = false;
    }
    return uno_ok;
}

void scha63x_read_data(scha63x_raw_data *data)
{
    uint32_t gyro_y_lsb;
    uint32_t gyro_z_lsb;
    uint32_t temp_due_lsb;
    uint32_t gyro_x_lsb;
    uint32_t acc_x_lsb;
    uint32_t acc_y_lsb;
    uint32_t acc_z_lsb;
    uint32_t temp_uno_lsb;

    if (data != NULL)
    {
        (void)SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Y);
        gyro_y_lsb = SPI_ASIC_DUE(SPI_FRAME_READ_GYRO_Z);
        gyro_z_lsb = SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);
        temp_due_lsb = SPI_ASIC_DUE(SPI_FRAME_READ_TEMP);

        (void)SPI_ASIC_UNO(SPI_FRAME_READ_GYRO_X);
        gyro_x_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_X);
        acc_x_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Y);
        acc_y_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_Z);
        acc_z_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);
        temp_uno_lsb = SPI_ASIC_UNO(SPI_FRAME_READ_TEMP);

        data->rs_error_due = scha63x_check_rs_error_3(gyro_y_lsb, gyro_z_lsb, temp_due_lsb);
        data->rs_error_uno = scha63x_check_rs_error_5(gyro_x_lsb, acc_x_lsb, acc_y_lsb,
                                                      acc_z_lsb, temp_uno_lsb);
        data->acc_x_lsb = scha63x_miso_int16(acc_x_lsb);
        data->acc_y_lsb = scha63x_miso_int16(acc_y_lsb);
        data->acc_z_lsb = scha63x_miso_int16(acc_z_lsb);
        data->gyro_x_lsb = scha63x_miso_int16(gyro_x_lsb);
        data->gyro_y_lsb = scha63x_miso_int16(gyro_y_lsb);
        data->gyro_z_lsb = scha63x_miso_int16(gyro_z_lsb);
        data->temp_due_lsb = scha63x_miso_int16(temp_due_lsb);
        data->temp_uno_lsb = scha63x_miso_int16(temp_uno_lsb);
    }
}

static bool scha63x_check_rs_error(const uint32_t *data, uint32_t count)
{
    uint32_t i;
    uint32_t value;
    bool error;

    error = false;
    if (data != NULL)
    {
        for (i = 0U; i < count; i++)
        {
            value = data[i];
            if (scha63x_miso_rs_error(value) == true)
            {
                error = true;
            }
        }
    }
    return error;
}

static bool scha63x_check_rs_error_3(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3)
{
    uint32_t miso_words[3];

    miso_words[0] = miso_1;
    miso_words[1] = miso_2;
    miso_words[2] = miso_3;
    return scha63x_check_rs_error(miso_words, 3U);
}

static bool scha63x_check_rs_error_5(uint32_t miso_1, uint32_t miso_2, uint32_t miso_3,
                                     uint32_t miso_4, uint32_t miso_5)
{
    uint32_t miso_words[5];

    miso_words[0] = miso_1;
    miso_words[1] = miso_2;
    miso_words[2] = miso_3;
    miso_words[3] = miso_4;
    miso_words[4] = miso_5;
    return scha63x_check_rs_error(miso_words, 5U);
}

void scha63x_convert_data(const scha63x_raw_data_summed *data_in, scha63x_real_data *data_out)
{
    if ((data_in != NULL) && (data_out != NULL))
    {
        data_out->acc_x = scha63x_lsb_to_unit(data_in->acc_x_lsb, SENSITIVITY_ACC);
        data_out->acc_y = scha63x_lsb_to_unit(data_in->acc_y_lsb, SENSITIVITY_ACC);
        data_out->acc_z = scha63x_lsb_to_unit(data_in->acc_z_lsb, SENSITIVITY_ACC);
        data_out->gyro_x = scha63x_lsb_to_unit(data_in->gyro_x_lsb, SENSITIVITY_GYRO_X);
        data_out->gyro_y = scha63x_lsb_to_unit(data_in->gyro_y_lsb, SENSITIVITY_GYRO_Y);
        data_out->gyro_z = scha63x_lsb_to_unit(data_in->gyro_z_lsb, SENSITIVITY_GYRO_Z);
        data_out->temp_due = scha63x_celsius(data_in->temp_due_lsb);
        data_out->temp_uno = scha63x_celsius(data_in->temp_uno_lsb);
    }
}

void scha63x_cross_axis_compensation(scha63x_real_data *data)
{
    float acc_x_comp;
    float acc_y_comp;
    float acc_z_comp;
    float gyro_x_comp;
    float gyro_y_comp;
    float gyro_z_comp;

    if (data != NULL)
    {
        acc_x_comp = s_scha63x_cac.bxx * data->acc_x;
        acc_x_comp += s_scha63x_cac.bxy * data->acc_y;
        acc_x_comp += s_scha63x_cac.bxz * data->acc_z;
        acc_y_comp = s_scha63x_cac.byx * data->acc_x;
        acc_y_comp += s_scha63x_cac.byy * data->acc_y;
        acc_y_comp += s_scha63x_cac.byz * data->acc_z;
        acc_z_comp = s_scha63x_cac.bzx * data->acc_x;
        acc_z_comp += s_scha63x_cac.bzy * data->acc_y;
        acc_z_comp += s_scha63x_cac.bzz * data->acc_z;
        gyro_x_comp = s_scha63x_cac.cxx * data->gyro_x;
        gyro_x_comp += s_scha63x_cac.cxy * data->gyro_y;
        gyro_x_comp += s_scha63x_cac.cxz * data->gyro_z;
        gyro_y_comp = s_scha63x_cac.cyx * data->gyro_x;
        gyro_y_comp += s_scha63x_cac.cyy * data->gyro_y;
        gyro_y_comp += s_scha63x_cac.cyz * data->gyro_z;
        gyro_z_comp = s_scha63x_cac.czx * data->gyro_x;
        gyro_z_comp += s_scha63x_cac.czy * data->gyro_y;
        gyro_z_comp += s_scha63x_cac.czz * data->gyro_z;

        data->acc_x = acc_x_comp;
        data->acc_y = acc_y_comp;
        data->acc_z = acc_z_comp;
        data->gyro_x = gyro_x_comp;
        data->gyro_y = gyro_y_comp;
        data->gyro_z = gyro_z_comp;
    }
}

void scha63x_read_sensor_status_uno(scha63x_sensor_status *status)
{
    uint32_t sum_stat;
    uint32_t rat_stat;
    uint32_t acc_stat;
    uint32_t com_stat_1;
    uint32_t com_stat_2;

    if (status != NULL)
    {
        scha63x_clear_status(status);
        (void)SPI_ASIC_UNO(SPI_FRAME_READ_SUMMARY_STATUS);
        sum_stat = SPI_ASIC_UNO(SPI_FRAME_READ_RATE_STATUS_1);
        rat_stat = SPI_ASIC_UNO(SPI_FRAME_READ_ACC_STATUS_1);
        acc_stat = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_1);
        com_stat_1 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);
        com_stat_2 = SPI_ASIC_UNO(SPI_FRAME_READ_COMMON_STATUS_2);

        status->summary_status = scha63x_miso_uint16(sum_stat);
        status->rate_status1 = scha63x_miso_uint16(rat_stat);
        status->acc_status1 = scha63x_miso_uint16(acc_stat);
        status->common_status1 = scha63x_miso_uint16(com_stat_1);
        status->common_status2 = scha63x_miso_uint16(com_stat_2);
    }
}

void scha63x_read_sensor_status_due(scha63x_sensor_status *status)
{
    uint32_t sum_stat;
    uint32_t rat_stat_1;
    uint32_t rat_stat_2;
    uint32_t com_stat_1;
    uint32_t com_stat_2;

    if (status != NULL)
    {
        scha63x_clear_status(status);
        (void)SPI_ASIC_DUE(SPI_FRAME_READ_SUMMARY_STATUS);
        sum_stat = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_1);
        rat_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_RATE_STATUS_2);
        rat_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_1);
        com_stat_1 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);
        com_stat_2 = SPI_ASIC_DUE(SPI_FRAME_READ_COMMON_STATUS_2);

        status->summary_status = scha63x_miso_uint16(sum_stat);
        status->rate_status1 = scha63x_miso_uint16(rat_stat_1);
        status->rate_status2 = scha63x_miso_uint16(rat_stat_2);
        status->common_status1 = scha63x_miso_uint16(com_stat_1);
        status->common_status2 = scha63x_miso_uint16(com_stat_2);
    }
}

static uint32_t scha63x_pack_rx(const uint8_t *rx)
{
    uint32_t b0;
    uint32_t b1;
    uint32_t b2;
    uint32_t b3;
    uint32_t word;
    uint8_t byte;

    byte = rx[0];
    b0 = (uint32_t)byte;
    byte = rx[1];
    b1 = (uint32_t)byte;
    byte = rx[2];
    b2 = (uint32_t)byte;
    byte = rx[3];
    b3 = (uint32_t)byte;

    word = b3;
    word |= (b2 << 8U);
    word |= (b1 << 16U);
    word |= (b0 << 24U);
    return word;
}

static uint32_t SPI_ASIC(uint32_t dout, GPIO_TypeDef *gpio, uint16_t pin)
{
    uint8_t tx[4];
    uint8_t rx[4];
    uint32_t field;
    uint32_t resp;
    HAL_StatusTypeDef hal_status;

    field = dout >> 24U;
    field &= 0xFFU;
    tx[0] = (uint8_t)field;
    field = dout >> 16U;
    field &= 0xFFU;
    tx[1] = (uint8_t)field;
    field = dout >> 8U;
    field &= 0xFFU;
    tx[2] = (uint8_t)field;
    field = dout & 0xFFU;
    tx[3] = (uint8_t)field;

    resp = 0U;
    if (gpio != NULL)
    {
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_RESET);
        hal_status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)4, SCHA63X_SPI_TIMEOUT_MS);
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
        if (hal_status == HAL_OK)
        {
            resp = scha63x_pack_rx(rx);
        }
    }
    return resp;
}

static uint32_t SPI_ASIC_DUE(uint32_t dout)
{
    return SPI_ASIC(dout, GPIOG, GPIO_PIN_12);
}

static uint32_t SPI_ASIC_UNO(uint32_t dout)
{
    return SPI_ASIC(dout, GPIOG, GPIO_PIN_10);
}
