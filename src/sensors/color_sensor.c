#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "color_sensor.h"

/* ------------------ Sensor Constants ------------------- */

#define TCS34725_ADDR       0x29
#define TCS34725_COMMAND    0x80

#define TCS34725_ENABLE     0x00
#define TCS34725_ENABLE_PON 0x01
#define TCS34725_ENABLE_AEN 0x02

#define TCS34725_ATIME      0x01
#define TCS34725_CONTROL    0x0F

#define TCS34725_CDATAL     0x14

/*  Integration time & Gain settings  */
#define INTEGRATION_TIME    0xF6   // 24ms
#define GAIN_4X             0x01

/* ------------------ Device Binding ------------------- */

#define I2C_NODE DT_NODELABEL(i2c1)

static const struct device *i2c_dev;

/* ------------------ Helper: Write Register ------------------- */
static int write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { TCS34725_COMMAND | reg, value };
    return i2c_write(i2c_dev, data, 2, TCS34725_ADDR);
}

/* ------------------ Helper: Read 16-bit ------------------- */
static int read_16(uint8_t reg, uint16_t *out)
{
    uint8_t addr = TCS34725_COMMAND | reg;
    uint8_t buffer[2];

    if (i2c_write_read(i2c_dev, TCS34725_ADDR,
                       &addr, 1,
                       buffer, 2) != 0)
        return -1;

    *out = buffer[1] << 8 | buffer[0];
    return 0;
}

/* ------------------ Public API ------------------- */

void color_sensor_init(void)
{
    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        printk("Color sensor I2C not ready!\n");
        return;
    }

    printk("TCS34725 detected on I2C1\n");

    /* Power ON */
    write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    k_msleep(3);

    /* Enable RGBC */
    write_reg(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

    /* Integration time + Gain */
    write_reg(TCS34725_ATIME, INTEGRATION_TIME);
    write_reg(TCS34725_CONTROL, GAIN_4X);

    printk("TCS34725 configured (ATIME=24ms, GAIN=4x)\n");
}

int color_sensor_read(struct color_data *data)
{
    if (!data) return -1;

    if (read_16(TCS34725_CDATAL, &data->clear) < 0) return -1;
    if (read_16(TCS34725_CDATAL+2, &data->red)   < 0) return -1;
    if (read_16(TCS34725_CDATAL+4, &data->green) < 0) return -1;
    if (read_16(TCS34725_CDATAL+6, &data->blue)  < 0) return -1;

    printk("C=%u R=%u G=%u B=%u\n",
            data->clear, data->red, data->green, data->blue);

    return 0;
}
