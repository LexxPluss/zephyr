#define DT_DRV_COMPAT adi_adis16470

#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/spi.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(ADIS16470, CONFIG_SENSOR_LOG_LEVEL);

#define ADIS16470_REG_DIAG_STAT     0x02
#define ADIS16470_REG_X_GYRO_LOW    0x04
#define ADIS16470_REG_X_GYRO_OUT    0x06
#define ADIS16470_REG_Y_GYRO_LOW    0x08
#define ADIS16470_REG_Y_GYRO_OUT    0x0a
#define ADIS16470_REG_Z_GYRO_LOW    0x0c
#define ADIS16470_REG_Z_GYRO_OUT    0x0e
#define ADIS16470_REG_X_ACCL_LOW    0x10
#define ADIS16470_REG_X_ACCL_OUT    0x12
#define ADIS16470_REG_Y_ACCL_LOW    0x14
#define ADIS16470_REG_Y_ACCL_OUT    0x16
#define ADIS16470_REG_Z_ACCL_LOW    0x18
#define ADIS16470_REG_Z_ACCL_OUT    0x1a
#define ADIS16470_REG_TEMP_OUT      0x1c
#define ADIS16470_REG_X_DELTANG_OUT 0x26
#define ADIS16470_REG_Y_DELTANG_OUT 0x2a
#define ADIS16470_REG_Z_DELTANG_OUT 0x2e
#define ADIS16470_REG_X_DELTVEL_OUT 0x32
#define ADIS16470_REG_Y_DELTVEL_OUT 0x36
#define ADIS16470_REG_Z_DELTVEL_OUT 0x3a
#define ADIS16470_REG_MSC_CTRL      0x60
#define ADIS16470_REG_GLOB_CMD      0x68
#define ADIS16470_REG_PROD_ID       0x72

struct adis16470_data {
    const struct device *spi;
    struct spi_config spi_cfg;
    int64_t gyro[3]; // micro rad/s
    int64_t accl[3]; // micro m/s/s
    int64_t temp;    // micro degc
    int64_t delta_ang[3]; // micro rad
    int64_t delta_vel[3]; // micro m/s
};

struct adis16470_config {
    char *spi_name;
    uint32_t spi_max_frequency;
    uint16_t spi_slave;
};

static int adis16470_reg_read(const struct adis16470_data *data, uint8_t reg, uint16_t *value)
{
    uint16_t buffer = (reg & 0x7f) << 8;
    struct spi_buf spibuf;
    spibuf.buf = &buffer;
    spibuf.len = 1;
    struct spi_buf_set bufset;
    bufset.buffers = &spibuf;
    bufset.count = 1;
    int status = spi_write(data->spi, &data->spi_cfg, &bufset);
    if (status) {
        LOG_ERR("SPI write error %d", status);
        return -1;
    }
    k_usleep(20);
    buffer = 0;
    status = spi_read(data->spi, &data->spi_cfg, &bufset);
    if (status) {
        LOG_ERR("SPI read error %d", status);
        return -1;
    }
    k_usleep(20);
    *value = buffer;
    return 0;
}

static int adis16470_reg_write(const struct adis16470_data *data, uint8_t reg, uint16_t value)
{
    uint16_t buffer;
    buffer = ((0x80 |   (reg & 0x7f)) << 8) | (value & 0xff);
    struct spi_buf spibuf;
    spibuf.buf = &buffer;
    spibuf.len = 1;
    struct spi_buf_set bufset;
    bufset.buffers = &spibuf;
    bufset.count = 1;
    int status = spi_write(data->spi, &data->spi_cfg, &bufset);
    if (status) {
        LOG_ERR("SPI write error %d", status);
        return -1;
    }
    k_usleep(20);
    buffer = ((0x80 | (++reg & 0x7f)) << 8) | ((value >> 8) & 0xff);
    status = spi_write(data->spi, &data->spi_cfg, &bufset);
    if (status) {
        LOG_ERR("SPI write error %d", status);
        return -1;
    }
    k_usleep(20);
    return 0;
}

static int adis16470_software_reset(const struct adis16470_data *data)
{
    int result = adis16470_reg_write(data, ADIS16470_REG_GLOB_CMD, 0x0080);
    k_msleep(300);
    return result;
}

static int adis16470_self_test(const struct adis16470_data *data)
{
    int result = adis16470_reg_write(data, ADIS16470_REG_GLOB_CMD, 0x0004);
    if (result != 0)
        return result;
    k_msleep(14);
    uint16_t value;
    result = adis16470_reg_read(data, ADIS16470_REG_DIAG_STAT, &value);
    if (result != 0)
        return result;
    if (value != 0) {
        LOG_ERR("DIAG_STAT error %u", value);
        return -1;
    }
    result = adis16470_reg_read(data, ADIS16470_REG_PROD_ID, &value);
    if (result != 0)
        return result;
    if (value != 0x4056) {
        LOG_ERR("PROD_ID error %04x", value);
        return -1;
    }
    return 0;
}

static int adis16470_configure(const struct adis16470_data *data)
{
#if 0
    uint16_t value;
    int result = adis16470_reg_read(data, ADIS16470_REG_MSC_CTRL, &value);
    if (result != 0)
        return result;
    value &= ~0x0001;
    result = adis16470_reg_write(data, ADIS16470_REG_MSC_CTRL, value);
    if (result != 0)
        return result;
#endif
    return 0;
}

static int adis16470_init(const struct device *dev)
{
    k_msleep(300);
    const struct adis16470_config *config = dev->config;
    struct adis16470_data *data = dev->data;
    data->spi = device_get_binding(config->spi_name);
    if (data->spi == NULL) {
        LOG_DBG("spi device not found: %s", config->spi_name);
        return -EINVAL;
    }
    data->spi_cfg.operation =
        SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA;
    data->spi_cfg.frequency = config->spi_max_frequency;
    data->spi_cfg.slave = config->spi_slave;
    int err = adis16470_software_reset(data);
    if (err) {
        LOG_ERR("adis16470_software_reset failed, error %d\n", err);
        return -ENODEV;
    }
    err = adis16470_self_test(data);
    if (err) {
        LOG_ERR("adis16470_self_test failed, error %d\n", err);
        return -ENODEV;
    }
    err = adis16470_configure(data);
    if (err) {
        LOG_ERR("adis16470_configure failed, error %d\n", err);
        return -ENODEV;
    }
    return 0;
}

static int adis16470_sample_fetch_delta_ang(struct adis16470_data *data, uint32_t reg, int64_t *value)
{
    int16_t v;
    int status = adis16470_reg_read(data, reg, &v);
    if (status == 0)
        *value = (int64_t)v * SENSOR_PI * 2160LL / 32768LL / 180LL;
    return status;
}

static int adis16470_sample_fetch_delta_vel(struct adis16470_data *data, uint32_t reg, int64_t *value)
{
    int16_t v;
    int status = adis16470_reg_read(data, reg, &v);
    if (status == 0)
        *value = (int64_t)v * 400LL * 1000000LL / 32768LL;
    return status;
}

static int adis16470_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct adis16470_data *data = dev->data;
    struct {
        uint16_t header, diag_stat;
        int16_t gyro_out[3], accl_out[3], temp_out;
        uint16_t data_cntr, checksum;
    } buffer[2] = {0};
    struct spi_buf spibuf_tx, spibuf_rx;
    struct spi_buf_set tx, rx;
    buffer[0].header = (ADIS16470_REG_GLOB_CMD & 0x7f) << 8;
    spibuf_tx.buf = &buffer[0];
    spibuf_tx.len = sizeof buffer[0] / sizeof (uint16_t);
    spibuf_rx.buf = &buffer[1];
    spibuf_rx.len = sizeof buffer[1] / sizeof (uint16_t);
    tx.buffers = &spibuf_tx;
    tx.count = 1;
    rx.buffers = &spibuf_rx;
    rx.count = 1;
    int status = spi_transceive(data->spi, &data->spi_cfg, &tx, &rx);
    if (status != 0) {
        LOG_ERR("SPI transceive error %d", status);
        return -1;
    }
    const uint8_t *p = (uint8_t*)&buffer[1].diag_stat;
    uint16_t sum = 0;
    for (int i = 0; i < 18; ++i)
        sum += *p++;
    if (sum != buffer[1].checksum) {
        LOG_ERR("invalid checksum %04x %04x", sum, buffer[1].checksum);
        return -1;
    }
    for (int i = 0; i < 3; ++i)
        data->gyro[i] = (int64_t)buffer[1].gyro_out[i] * SENSOR_PI / 180LL / 10LL;
    for (int i = 0; i < 3; ++i)
        data->accl[i] = (int64_t)buffer[1].accl_out[i] * SENSOR_G / 800LL;
    data->temp = (int64_t)buffer[1].temp_out * 1000000LL / 10LL;

    status = adis16470_sample_fetch_delta_ang(data, ADIS16470_REG_X_DELTANG_OUT, &data->delta_ang[0]);
    if (status == 0)
        status = adis16470_sample_fetch_delta_ang(data, ADIS16470_REG_Y_DELTANG_OUT, &data->delta_ang[1]);
    if (status == 0)
        status = adis16470_sample_fetch_delta_ang(data, ADIS16470_REG_Z_DELTANG_OUT, &data->delta_ang[2]);
    if (status == 0)
        status = adis16470_sample_fetch_delta_vel(data, ADIS16470_REG_X_DELTVEL_OUT, &data->delta_vel[0]);
    if (status == 0)
        status = adis16470_sample_fetch_delta_vel(data, ADIS16470_REG_Y_DELTVEL_OUT, &data->delta_vel[1]);
    if (status == 0)
        status = adis16470_sample_fetch_delta_vel(data, ADIS16470_REG_Z_DELTVEL_OUT, &data->delta_vel[2]);
    return status;
}

static void store_value(struct sensor_value *val, int64_t data)
{
	val->val1 = data / 1000000;
	val->val2 = data % 1000000;
}

static int adis16470_channel_get(const struct device *dev,
        enum sensor_channel chan,
        struct sensor_value *val)
{
    struct adis16470_data *data = dev->data;
    switch (chan) {
    case SENSOR_CHAN_GYRO_X:   store_value(val, data->gyro[0]); break;
    case SENSOR_CHAN_GYRO_Y:   store_value(val, data->gyro[1]); break;
    case SENSOR_CHAN_GYRO_Z:   store_value(val, data->gyro[2]); break;
    case SENSOR_CHAN_ACCEL_X:  store_value(val, data->accl[0]); break;
    case SENSOR_CHAN_ACCEL_Y:  store_value(val, data->accl[1]); break;
    case SENSOR_CHAN_ACCEL_Z:  store_value(val, data->accl[2]); break;
    case SENSOR_CHAN_DIE_TEMP: store_value(val, data->temp);    break;
    default:
        if      (chan == SENSOR_CHAN_PRIV_START)     store_value(val, data->delta_ang[0]);
        else if (chan == SENSOR_CHAN_PRIV_START + 1) store_value(val, data->delta_ang[1]);
        else if (chan == SENSOR_CHAN_PRIV_START + 2) store_value(val, data->delta_ang[2]);
        else if (chan == SENSOR_CHAN_PRIV_START + 3) store_value(val, data->delta_vel[0]);
        else if (chan == SENSOR_CHAN_PRIV_START + 4) store_value(val, data->delta_vel[1]);
        else if (chan == SENSOR_CHAN_PRIV_START + 5) store_value(val, data->delta_vel[2]);
        else                                         return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api adis16470_api_funcs = {
    .sample_fetch = adis16470_sample_fetch,
    .channel_get = adis16470_channel_get,
};

static struct adis16470_data adis16470_data;

static const struct adis16470_config adis16470_config = {
    .spi_name = DT_INST_BUS_LABEL(0),
    .spi_max_frequency = DT_INST_PROP(0, spi_max_frequency),
    .spi_slave = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, adis16470_init, NULL, &adis16470_data,
        &adis16470_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
        &adis16470_api_funcs);
