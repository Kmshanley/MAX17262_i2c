#include <stdio.h>
#include "MAX17262_i2c.h"

#define REV16_A(X) (((X) << 8) | ((X)>>8))

static const char *TAG = "MAX17262 Driver";

esp_err_t MAX17262_init_desc(MAX17262_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    if (addr != MAX17262_I2C_ADDR)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t MAX17262_get_voltage(MAX17262_t *dev, float * volts, bool avg) 
{
    return -1;
}

esp_err_t MAX17262_get_soc(MAX17262_t *dev, float * soc) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_REPSOC, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *soc = (float)data / 256.f;
    return ESP_OK;
}

esp_err_t MAX17262_get_remCap(MAX17262_t *dev, float * remCap) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_REPCAP, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *remCap = (float)data * CAPACITY_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_fullCap(MAX17262_t *dev, float * fullCap) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_FULLREPCAP, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *fullCap = (float)data * CAPACITY_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_avgCurrent(MAX17262_t *dev, float * current) 
{
    int16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_AVGCURRENT, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *current = (float)data * CURRENT_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_instCurrent(MAX17262_t *dev, float * current) 
{
    int16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_CURRENT, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *current = (float)data * CURRENT_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_avgVbat(MAX17262_t *dev, float * avgVbat) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_AVGVCELL, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *avgVbat = (float)data * VOLTAGE_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_avgTemp(MAX17262_t *dev, float * temp) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_AVGTA, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *temp = (float)data * TEMPERATURE_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_TTE(MAX17262_t *dev, float * tte) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_TTE, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *tte = (float)data * TIME_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_TTF(MAX17262_t *dev, float * ttf) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_TTF, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *ttf = (float)data * TIME_LSB;
    return ESP_OK;
}

esp_err_t MAX17262_get_cycles(MAX17262_t *dev, float * cycles) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_CYCLES, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *cycles = (float)data * 0.01f; //1% LSB
    return ESP_OK;
}

esp_err_t MAX17262_get_minMaxCurrent(MAX17262_t *dev, float * minCurrent, float * maxCurrent) 
{
    int16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_MAXMINCURRENT, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *minCurrent = (float)(data & 0b11111111) * 160; //160 mA LSB
    *maxCurrent = (float)(data >> 8) * 160;
    return ESP_OK;
}

esp_err_t MAX17262_get_filteredPower(MAX17262_t *dev, float * power) 
{
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_CYCLES, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *power = (float)data * 1.6f; //1.6mW LSB
    return ESP_OK;
}

esp_err_t MAX17262_wakeup(MAX17262_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_HIBCFG, &dev->hibernationCfg, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    uint16_t data = 0x0090;
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_SOFTWAKEUP, &data, 2);
    data = 0x0000;
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_HIBCFG, &data, 2);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_SOFTWAKEUP, &data, 2);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t MAX17262_autoHibernate(MAX17262_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint16_t data = 0x0000;
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_SOFTWAKEUP, &data, 2);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_HIBCFG, &dev->hibernationCfg, 2);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return ESP_OK;
}

esp_err_t MAX17262_set_batt_params(MAX17262_t *dev, uint32_t cap, float vempty, float vrecovery, uint8_t model) 
{
    uint16_t designcap = (uint16_t)(cap / CAPACITY_LSB);
    uint16_t un_vempty = (uint16_t)((vempty * 1000.0f) / 10.0); //10 mv resolution
    un_vempty = un_vempty << 7;
    uint16_t un_vrec = (uint16_t)((vrecovery * 1000.0f) / 40.0); //40 mv resolution
    un_vrec = un_vrec & 0b000000001111111; //mask bottom part of register
    uint16_t combined_vset = 0;
    combined_vset = un_vempty | un_vrec;
    uint16_t modelcfg_set = 0;
    modelcfg_set = modelcfg_set | (model << 4);
    modelcfg_set = modelcfg_set | (1 << 15); //set model refresh bit
    modelcfg_set = modelcfg_set | (1 << 10); //for v charge higher than 4.2v
    modelcfg_set = modelcfg_set | (1 << 2); //datasheet says to set this bit

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_MODELCFG, &modelcfg_set, 2);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_VEMPTY, &combined_vset, 2);
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_DESIGNCAP, &designcap, 2);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

esp_err_t MAX17262_check_POR(MAX17262_t *dev, bool *POR) {
    uint16_t data;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    if (i2c_dev_read_reg(&dev->i2c_dev, MAX17262_STATUS, &data, 2) != ESP_OK) {
        I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
        return ESP_FAIL;
    }
    uint16_t mdata = data;
    mdata = mdata & 0xfffd; //clear por bit
    i2c_dev_write_reg(&dev->i2c_dev, MAX17262_STATUS, &mdata, 2);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    *POR = (data & 0x0002); //mask POR bit
    return ESP_OK;
 }
