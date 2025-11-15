#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include "accelerometer.h"

/* ============================================================
 *  Configuración MMA8451 + I2C
 * ============================================================ */

/* Dirección I2C del MMA8451 (7 bits) */
#define MMA8451_I2C_ADDR              0x1D

/* Nodo I2C en el device tree  */
#define ACCEL_I2C_NODE                DT_NODELABEL(i2c2)

/* I2C bus device and address */
static const struct device *accel_i2c_bus;
static struct i2c_dt_spec accel_i2c;

/* Registros principales (datasheet) */
#define MMA8451_REG_STATUS            0x00
#define MMA8451_REG_OUT_X_MSB         0x01
#define MMA8451_REG_OUT_Y_MSB         0x03
#define MMA8451_REG_OUT_Z_MSB         0x05
#define MMA8451_REG_OUT_X_LSB         0x02
#define MMA8451_REG_OUT_Y_LSB         0x04
#define MMA8451_REG_OUT_Z_LSB         0x06
#define MMA8451_REG_WHO_AM_I          0x0D
#define MMA8451_REG_XYZ_DATA_CFG      0x0E
#define MMA8451_REG_CTRL_REG1         0x2A

/* Valores esperados / máscaras */
#define MMA8451_WHO_AM_I_ID           0x1A
#define MMA8451_CTRL_REG1_ACTIVE_BIT  BIT(0)

/* Rangos de medida */
#define MMA8451_RANGE_2G              0x00
#define MMA8451_RANGE_4G              0x01
#define MMA8451_RANGE_8G              0x02
#define MMA8451_RANGE_MASK            0x03

/* Sensibilidad (cuentas por g) para 14 bits (datasheet) */
#define MMA8451_SENS_2G_COUNTS_PER_G  4096.0f
#define MMA8451_SENS_4G_COUNTS_PER_G  2048.0f
#define MMA8451_SENS_8G_COUNTS_PER_G  1024.0f

/* Aceleración de la gravedad en m/s² */
#define ACCEL_GRAVITY_MS2             9.80665f

/* Tiempo entre lecturas (para ejemplo, no se usa en el driver) */
#define ACCEL_READ_DELAY_MS           500

/* ============================================================
 *  Estado interno del driver
 * ============================================================ */

static bool accel_last_error = false;
static uint8_t accel_current_range = MMA8451_RANGE_2G;
static float accel_counts_per_g = MMA8451_SENS_2G_COUNTS_PER_G;


/* ============================================================
 *  Funciones internas de ayuda (I2C + registros)
 * ============================================================ */

/* Lectura genérica de registros del MMA8451 */
static int mma8451_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read_dt(&accel_i2c, &reg, 1, buf, len);
}

/* Escritura de un registro de 8 bits */
static int mma8451_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = { reg, value };
    return i2c_write_dt(&accel_i2c, tx_buf, sizeof(tx_buf));
}

/* Verificar identidad del sensor mediante WHO_AM_I */
static int mma8451_check_id(void)
{
    uint8_t whoami;
    int ret = mma8451_read_regs(MMA8451_REG_WHO_AM_I, &whoami, 1);

    if (ret < 0) {
        printk("ACCEL: ERROR al leer WHO_AM_I (%d)\n", ret);
        return ret;
    }

    if (whoami != MMA8451_WHO_AM_I_ID) {
        printk("ACCEL: WHO_AM_I = 0x%02X, esperado 0x%02X\n",
               whoami, MMA8451_WHO_AM_I_ID);
        return -EIO;
    }

    return 0;
}

/* Configura el rango de medida (±2g, ±4g, ±8g) */
static int mma8451_set_range(uint8_t range)
{
    int ret;
    uint8_t ctrl1;

    /* Leer CTRL_REG1 para poder desactivar el sensor antes de reconfigurar */
    ret = mma8451_read_regs(MMA8451_REG_CTRL_REG1, &ctrl1, 1);
    if (ret < 0) {
        return ret;
    }

    /* Poner en standby (bit ACTIVE = 0) */
    ctrl1 &= ~MMA8451_CTRL_REG1_ACTIVE_BIT;
    ret = mma8451_write_reg(MMA8451_REG_CTRL_REG1, ctrl1);
    if (ret < 0) {
        return ret;
    }

    k_msleep(1);

    /* Escribir el nuevo rango en XYZ_DATA_CFG */
    ret = mma8451_write_reg(MMA8451_REG_XYZ_DATA_CFG, range & MMA8451_RANGE_MASK);
    if (ret < 0) {
        return ret;
    }

    /* Volver a activar el sensor */
    ctrl1 |= MMA8451_CTRL_REG1_ACTIVE_BIT;
    ret = mma8451_write_reg(MMA8451_REG_CTRL_REG1, ctrl1);
    if (ret < 0) {
        return ret;
    }

    /* Actualizar sensibilidad en función del rango */
    accel_current_range = range & MMA8451_RANGE_MASK;
    switch (accel_current_range) {
    case MMA8451_RANGE_2G:
        accel_counts_per_g = MMA8451_SENS_2G_COUNTS_PER_G;
        break;
    case MMA8451_RANGE_4G:
        accel_counts_per_g = MMA8451_SENS_4G_COUNTS_PER_G;
        break;
    case MMA8451_RANGE_8G:
        accel_counts_per_g = MMA8451_SENS_8G_COUNTS_PER_G;
        break;
    default:
        accel_counts_per_g = MMA8451_SENS_2G_COUNTS_PER_G;
        break;
    }

    return 0;
}

/* Lee los tres ejes crudos (14 bits) del MMA8451 */
static int mma8451_read_raw_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    int ret = mma8451_read_regs(MMA8451_REG_OUT_X_MSB, buf, sizeof(buf));

    if (ret < 0) {
        return ret;
    }

    /* Cada eje es un valor de 14 bits: combinar MSB/LSB y desplazar 2 bits
     * 
     * ejemplo
     *   buf[0] (MSB): [D13 D12 D11 D10 D09 D08 D07 D06]
     *   buf[1] (LSB): [D05 D04 D03 D02 D01 D00  0   0 ]
     * 
     * Paso 1 - Combinar (buf[0] << 8) | buf[1]:
     *   [D13 D12 D11 D10 D09 D08 D07 D06][D05 D04 D03 D02 D01 D00  0   0 ]
     * 
     * Paso 2 - Desplazar >> 2 para alinear a la derecha (valor de 14 bits):
     *   [ 0   0  D13 D12 D11 D10 D09 D08][D07 D06 D05 D04 D03 D02 D01 D00]
     * 
     * Resultado: int16_t con signo extendido (14 bits útiles)
     */
    *x = (int16_t)(((buf[0] << 8) | buf[1]) >> 2);
    *y = (int16_t)(((buf[2] << 8) | buf[3]) >> 2);
    *z = (int16_t)(((buf[4] << 8) | buf[5]) >> 2);

    return 0;
}

/* Convierte un valor crudo a g usando la sensibilidad actual */
static float mma8451_raw_to_g(int16_t raw)
{
    return (float)raw / accel_counts_per_g;
}

/* Convierte valor crudo a m/s² directamente */
static float mma8451_raw_to_ms2(int16_t raw)
{
    float g = mma8451_raw_to_g(raw);
    return g * ACCEL_GRAVITY_MS2;
}


/* ============================================================
 *  funciones públicas del driver
 * ============================================================ */

/*
 * Inicialización general del acelerómetro:
 *  - Verifica el estado del bus I2C.
 *  - Comprueba la identidad del MMA8451.
 *  - Configura el sensor en rango ±2g y lo activa.
 */
void accelerometer_init(void)
{
    int ret;
    uint8_t ctrl1;

    /* Initialize I2C spec at runtime */
    accel_i2c_bus = DEVICE_DT_GET(ACCEL_I2C_NODE);
    accel_i2c.bus = accel_i2c_bus;
    accel_i2c.addr = MMA8451_I2C_ADDR;

    if (!i2c_is_ready_dt(&accel_i2c)) {
        printk("ACCEL: ERROR (I2C no listo)\n");
        accel_last_error = true;
        return;
    }

    ret = mma8451_check_id();
    if (ret < 0) {
        printk("ACCEL: ERROR (sensor MMA8451 no encontrado)\n");
        accel_last_error = true;
        return;
    }

    /* Leer CTRL_REG1 actual */
    ret = mma8451_read_regs(MMA8451_REG_CTRL_REG1, &ctrl1, 1);
    if (ret < 0) {
        printk("ACCEL: ERROR al leer CTRL_REG1 (%d)\n", ret);
        accel_last_error = true;
        return;
    }

    /* Asegurarnos de que está activo */
    ctrl1 |= MMA8451_CTRL_REG1_ACTIVE_BIT;
    ret = mma8451_write_reg(MMA8451_REG_CTRL_REG1, ctrl1);
    if (ret < 0) {
        printk("ACCEL: ERROR al activar sensor (%d)\n", ret);
        accel_last_error = true;
        return;
    }

    k_msleep(10);

    /* Configurar rango por defecto a ±2g */
    ret = mma8451_set_range(MMA8451_RANGE_2G);
    if (ret < 0) {
        printk("ACCEL: ERROR al configurar rango (%d)\n", ret);
        accel_last_error = true;
        return;
    }

    accel_last_error = false;
    printk("ACCEL: MMA8451 inicializado (rango ±2g)\n");
}

/*
 * Lee los tres ejes y devuelve la aceleración en m/s².
 * Si ocurre un error I2C (sensor desconectado/apagado), devuelve -1.
 */
int accelerometer_read(struct accel_data *data)
{
    int16_t raw_x, raw_y, raw_z;
    int ret;

    if (!i2c_is_ready_dt(&accel_i2c)) {
        if (!accel_last_error) {
            printk("ACCEL: ERROR (I2C no listo)\n");
        }
        accel_last_error = true;
        return -1;
    }

    ret = mma8451_read_raw_xyz(&raw_x, &raw_y, &raw_z);
    if (ret < 0) {
        if (!accel_last_error) {
            printk("ACCEL: ERROR al leer ejes (%d)\n", ret);
        }
        accel_last_error = true;
        return -1;
    }

    /* Heurística simple: si todo es 0, puede ser fallo / sin alimentación */
    if (raw_x == 0 && raw_y == 0 && raw_z == 0) {
        if (!accel_last_error) {
            printk("ACCEL: ERROR (valores crudos todos 0)\n");
        }
        accel_last_error = true;
        return -1;
    }

    accel_last_error = false;

    data->x_ms2 = mma8451_raw_to_ms2(raw_x);
    data->y_ms2 = mma8451_raw_to_ms2(raw_y);
    data->z_ms2 = mma8451_raw_to_ms2(raw_z);

    return 0;
}
