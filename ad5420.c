/*
 * drivers/iio/dac/ad5420.c
 * AD5420 IIO DAC driver 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>


#define AD5420_REG_NOP       0x00
#define AD5420_REG_DATA      0x01
#define AD5420_REG_READBACK  0x02
#define AD5420_REG_CONTROL   0x55
#define AD5420_REG_RESET     0x56


#define AD5420_CTRL_REXT    (1 << 13)
#define AD5420_CTRL_OUTEN   (1 << 12)
#define AD5420_CTRL_SR_CLOCK_MASK (0xF << 8)
#define AD5420_CTRL_SR_STEP_MASK  (0x7 << 5)
#define AD5420_CTRL_SREN    (1 << 4)
#define AD5420_CTRL_DCEN    (1 << 3)

#define AD5420_CTRL_RANGE_MASK 0x7 

#define AD5420_READBACK_STATUS   0x00
#define AD5420_READBACK_DATA     0x01
#define AD5420_READBACK_CONTROL  0x02

#define AD5420_STATUS_IOUT_FAULT  BIT(2)  
#define AD5420_STATUS_SLEW_ACTIVE BIT(1)  
#define AD5420_STATUS_OVERTEMP    BIT(0)  

enum ad5420_current_range {
    AD5420_RANGE_4mA_20mA = 0,
    AD5420_RANGE_0mA_20mA,
    AD5420_RANGE_0mA_24mA,
};

struct ad5420_state {
    struct spi_device *spi;
    struct mutex lock;
    unsigned int ctrl_cache;
    enum ad5420_current_range current_range;
    unsigned int fault_mask;

    struct gpio_desc *latch_gpiod;  
    struct gpio_desc *clear_gpiod;  
    struct gpio_desc *fault_gpiod;  

    int fault_irq;

    union {
        __be32 d32;
        u8 d8[4];
    } xfer[2] ____cacheline_aligned;
};

static const struct iio_event_spec ad5420_current_event[] = {
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_RISING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
    }, {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_FALLING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
    }
};

static const struct iio_event_spec ad5420_temp_event[] = {
    {
        .type = IIO_EV_TYPE_THRESH,
        .dir = IIO_EV_DIR_RISING,
        .mask_separate = BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_ENABLE),
    }
};

static const struct iio_chan_spec ad5420_channels[] = {
    {
        .type = IIO_CURRENT,
        .indexed = 1,
        .output = 1,
        .channel = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
                              BIT(IIO_CHAN_INFO_CALIBSCALE) |
                              BIT(IIO_CHAN_INFO_CALIBBIAS),
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
                                    BIT(IIO_CHAN_INFO_OFFSET),
        .scan_type = {
            .sign = 'u',
            .realbits = 16,
            .storagebits = 16,
        },
        .event_spec = ad5420_current_event,
        .num_event_specs = ARRAY_SIZE(ad5420_current_event),
    }, {
        .type = IIO_TEMP,
        .channel = -1,
        .event_spec = ad5420_temp_event,
        .num_event_specs = ARRAY_SIZE(ad5420_temp_event),
    }
};


static inline void ad5420_latch_set(struct ad5420_state *st, int level)
{
    if (!st->latch_gpiod)
        return;
    gpiod_set_value_cansleep(st->latch_gpiod, level);
    udelay(1);
}

static int ad5420_spi_write_frame(struct iio_dev *indio_dev, u8 addr, u16 data)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    u8 tx_buf[3];
    int ret;

    tx_buf[0] = addr;
    tx_buf[1] = (data >> 8) & 0xFF;
    tx_buf[2] = data & 0xFF;

    dev_info(&st->spi->dev, "AD5420 WRITE: addr=0x%02x data=0x%04x\n", addr, data);

    if (st->clear_gpiod)
        gpiod_set_value_cansleep(st->clear_gpiod, 0);

    ad5420_latch_set(st, 1);
    
    ret = spi_write(st->spi, tx_buf, 3);
    if (ret) {
        dev_err(&st->spi->dev, "SPI transfer failed: %d\n", ret);
        goto cleanup;
    }

    ad5420_latch_set(st, 0);
    udelay(5); 

cleanup:
    ad5420_latch_set(st, 1);
    return ret;
}

static int ad5420_spi_read_reg(struct iio_dev *indio_dev, u8 readback_addr)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    u8 tx_buf[3] = { AD5420_REG_READBACK, 0x00, readback_addr };
    u8 rx_buf[3] = {0};
    int ret;

    dev_info(&st->spi->dev, "AD5420 READ CMD: 0x%02x 0x%02x 0x%02x\n", 
             tx_buf[0], tx_buf[1], tx_buf[2]);

    ad5420_latch_set(st, 0);  
    
    ret = spi_write(st->spi, tx_buf, 3);
    if (ret) {
        dev_err(&st->spi->dev, "Read command failed: %d\n", ret);
        goto cleanup;
    }

    ad5420_latch_set(st, 1);
    udelay(10);
    ad5420_latch_set(st, 0);

    memset(tx_buf, 0, sizeof(tx_buf));
    ret = spi_write_then_read(st->spi, tx_buf, 3, rx_buf, 3);
    if (ret) {
        dev_err(&st->spi->dev, "Read data failed: %d\n", ret);
        goto cleanup;
    }

    ret = (rx_buf[1] << 8) | rx_buf[2];
    dev_info(&st->spi->dev, "AD5420 READ RESULT: 0x%04x\n", ret);

cleanup:
    return ret;
}

static void ad5420_full_diagnose(struct iio_dev *indio_dev)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    int status, control, data;
    
    dev_info(&st->spi->dev, "=== AD5420 FULL DIAGNOSTICS ===\n");
    
    status = ad5420_spi_read_reg(indio_dev, AD5420_READBACK_STATUS);
    dev_info(&st->spi->dev, "STATUS: 0x%04x\n", status);
    dev_info(&st->spi->dev, "  IOUT Fault: %s\n", (status & AD5420_STATUS_IOUT_FAULT) ? "YES" : "NO");
    dev_info(&st->spi->dev, "  Slew Active: %s\n", (status & AD5420_STATUS_SLEW_ACTIVE) ? "YES" : "NO");
    dev_info(&st->spi->dev, "  Overtemp: %s\n", (status & AD5420_STATUS_OVERTEMP) ? "YES" : "NO");
    
    control = ad5420_spi_read_reg(indio_dev, AD5420_READBACK_CONTROL);
    dev_info(&st->spi->dev, "CONTROL: 0x%04x\n", control);
    dev_info(&st->spi->dev, "  REXT: %s\n", (control & AD5420_CTRL_REXT) ? "EXTERNAL" : "INTERNAL");
    dev_info(&st->spi->dev, "  OUTEN: %s\n", (control & AD5420_CTRL_OUTEN) ? "ENABLED" : "DISABLED");
    dev_info(&st->spi->dev, "  Range: %d%d%d\n", 
             !!(control & 0x4), !!(control & 0x2), !!(control & 0x1));
    
    data = ad5420_spi_read_reg(indio_dev, AD5420_READBACK_DATA);
    dev_info(&st->spi->dev, "DATA: 0x%04x (%d)\n", data, data);
    
    if (st->latch_gpiod) {
        dev_info(&st->spi->dev, "LATCH GPIO: %d\n", gpiod_get_value(st->latch_gpiod));
    }
    if (st->clear_gpiod) {
        dev_info(&st->spi->dev, "CLEAR GPIO: %d\n", gpiod_get_value(st->clear_gpiod));
    }
    
    dev_info(&st->spi->dev, "=== END DIAGNOSTICS ===\n");
}

static int ad5420_hardware_test(struct iio_dev *indio_dev)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    int ret;
    unsigned int ctrl = AD5420_CTRL_OUTEN | 0x5; // OUTEN + 4-20mA range
    
    dev_info(&st->spi->dev, "=== AD5420 HARDWARE TEST ===\n");
    
    // 1. Сброс
    dev_info(&st->spi->dev, "Step 1: Reset\n");
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_RESET, 0x0001);
    if (ret) return ret;
    msleep(10);
    
    // 2. Запись контрольного регистра
    dev_info(&st->spi->dev, "Step 2: Write Control Register\n");
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_CONTROL, ctrl);
    if (ret) return ret;
    
    // 3. Тестовая запись данных
    dev_info(&st->spi->dev, "Step 3: Test Data Write\n");
    
    // 4mA
    dev_info(&st->spi->dev, "Setting 4mA (code 0)\n");
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_DATA, 0x0000);
    if (ret) return ret;
    msleep(1000);
    
    // 12mA  
    dev_info(&st->spi->dev, "Setting 12mA (code 32768)\n");
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_DATA, 0x8000);
    if (ret) return ret;
    msleep(1000);
    
    // 20mA
    dev_info(&st->spi->dev, "Setting 20mA (code 65535)\n");
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_DATA, 0xFFFF);
    if (ret) return ret;
    msleep(1000);
    
    dev_info(&st->spi->dev, "=== HARDWARE TEST COMPLETE ===\n");
    return 0;
}

/* Read status register via readback mechanism */
static unsigned int ad5420_read_status(struct iio_dev *indio_dev)
{
    int val = ad5420_spi_read_reg(indio_dev, AD5420_READBACK_STATUS);
    if (val < 0) return 0; /* Error */
    return val;
}

/* write control and cache it */
static int ad5420_write_control(struct iio_dev *indio_dev, unsigned int ctrl)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    int ret;

    st->ctrl_cache = ctrl;
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_CONTROL, (u16)ctrl);
    if (ret)
        dev_err(&st->spi->dev, "ad5420: write control failed %d\n", ret);
    else
        dev_dbg(&st->spi->dev, "ad5420: control write 0x%04x\n", ctrl);
    return ret;
}

static irqreturn_t ad5420_fault_handler(int irq, void *data)
{
    struct iio_dev *indio_dev = data;
    struct ad5420_state *st = iio_priv(indio_dev);
    unsigned int status;

    status = ad5420_read_status(indio_dev);
    if (status < 0) return IRQ_NONE;

    dev_info(&st->spi->dev, "ad5420: fault status 0x%04x\n", status);

    if ((status & AD5420_STATUS_IOUT_FAULT) && (st->fault_mask & AD5420_STATUS_IOUT_FAULT)) {
        iio_push_event(indio_dev, 
                       IIO_UNMOD_EVENT_CODE(IIO_CURRENT, 0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
                       iio_get_time_ns(indio_dev));
        dev_warn(&st->spi->dev, "ad5420: output fault detected\n");
    }
    
    if ((status & AD5420_STATUS_OVERTEMP) && (st->fault_mask & AD5420_STATUS_OVERTEMP)) {
        iio_push_event(indio_dev,
                       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0, IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
                       iio_get_time_ns(indio_dev));
        dev_warn(&st->spi->dev, "ad5420: overtemperature detected\n");
    }

    return IRQ_HANDLED;
}

static int ad5420_read_raw(struct iio_dev *indio_dev,
                           struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    int ret, min, max;

    mutex_lock(&st->lock);

    if (chan->type != IIO_CURRENT) {
        ret = -EINVAL;
        goto unlock;
    }

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        ret = ad5420_spi_read_reg(indio_dev, AD5420_READBACK_DATA);
        if (ret < 0) goto unlock;
        *val = ret;
        ret = IIO_VAL_INT;
        break;

    case IIO_CHAN_INFO_SCALE:
        switch (st->current_range) {
        case AD5420_RANGE_4mA_20mA: min = 4; max = 20; break;
        case AD5420_RANGE_0mA_20mA: min = 0; max = 20; break;
        case AD5420_RANGE_0mA_24mA: min = 0; max = 24; break;
        default: min = 4; max = 20;
        }
        *val = (max - min) * 1000;  /* mA to uA */
        *val2 = 1 << 16;
        ret = IIO_VAL_FRACTIONAL;
        break;

    case IIO_CHAN_INFO_OFFSET:
        switch (st->current_range) {
        case AD5420_RANGE_4mA_20mA: 
            *val = (1 << 16) * 4 / 16;  /* 16384 for 4mA offset in 16mA span */
            break;
        default: *val = 0;
        }
        ret = IIO_VAL_INT;
        break;

    case IIO_CHAN_INFO_CALIBBIAS:
        *val = 0;  /* No calibration bias support */
        ret = IIO_VAL_INT;
        break;

    case IIO_CHAN_INFO_CALIBSCALE:
        *val = st->ctrl_cache;
        ret = IIO_VAL_INT;
        break;

    default:
        ret = -EINVAL;
    }

unlock:
    mutex_unlock(&st->lock);
    return ret;
}

static int ad5420_write_event_config(struct iio_dev *indio_dev,
                                     const struct iio_chan_spec *chan,
                                     enum iio_event_type type,
                                     enum iio_event_direction dir, int state)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    unsigned int mask;

    switch (chan->type) {
    case IIO_CURRENT:
        mask = AD5420_STATUS_IOUT_FAULT;
        break;
    case IIO_TEMP:
        mask = AD5420_STATUS_OVERTEMP;
        break;
    default:
        return -EINVAL;
    }

    mutex_lock(&st->lock);
    if (state) 
        st->fault_mask |= mask;
    else 
        st->fault_mask &= ~mask;
    mutex_unlock(&st->lock);

    return 0;
}

static int ad5420_read_event_config(struct iio_dev *indio_dev,
                                    const struct iio_chan_spec *chan,
                                    enum iio_event_type type,
                                    enum iio_event_direction dir)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    unsigned int mask;

    switch (chan->type) {
    case IIO_CURRENT:
        mask = AD5420_STATUS_IOUT_FAULT;
        break;
    case IIO_TEMP:
        mask = AD5420_STATUS_OVERTEMP;
        break;
    default:
        return -EINVAL;
    }

    return !!(st->fault_mask & mask);
}

static int ad5420_read_event_value(struct iio_dev *indio_dev,
                                   const struct iio_chan_spec *chan,
                                   enum iio_event_type type,
                                   enum iio_event_direction dir,
                                   enum iio_event_info info, int *val, int *val2)
{
    switch (chan->type) {
    case IIO_CURRENT:
        *val = 0;  /* No specific threshold */
        break;
    case IIO_TEMP:
        *val = 150000; /* millidegrees C */
        break;
    default:
        return -EINVAL;
    }

    return IIO_VAL_INT;
}

static int ad5420_write_raw(struct iio_dev *indio_dev,
                            const struct iio_chan_spec *chan, 
                            int val, int val2, long mask)
{
    struct ad5420_state *st = iio_priv(indio_dev);
    int ret;

    if (chan->type != IIO_CURRENT || mask != IIO_CHAN_INFO_RAW)
        return -EINVAL;

    if (val < 0 || val > 65535)
        return -EINVAL;

    mutex_lock(&st->lock);

    dev_info(&st->spi->dev, "AD5420: Setting output to %d (0x%04x)\n", val, val);

    // Записываем значение в регистр данных
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_DATA, (u16)val);
    if (ret)
        dev_err(&st->spi->dev, "Failed to set output: %d\n", ret);
    else
        dev_info(&st->spi->dev, "AD5420: Output set successfully\n");

    mutex_unlock(&st->lock);
    return ret;
}

static int ad5420_parse_dt(struct spi_device *spi, struct ad5420_state *st)
{
    struct device_node *np = spi->dev.of_node;
    u32 crange = 0;

    if (!np) 
        return 0;

    of_property_read_u32(np, "adi,current-range", &crange);
    switch (crange) {
    case 0: 
        st->current_range = AD5420_RANGE_4mA_20mA; 
        break;
    case 1: 
        st->current_range = AD5420_RANGE_0mA_20mA; 
        break;
    case 2: 
        st->current_range = AD5420_RANGE_0mA_24mA; 
        break;
    default: 
        st->current_range = AD5420_RANGE_4mA_20mA; 
        break;
    }

    /* LATCH: idle low, high during transfer */
    st->latch_gpiod = devm_gpiod_get_optional(&spi->dev, "ldac", GPIOD_OUT_LOW);
    if (IS_ERR(st->latch_gpiod)) 
        return PTR_ERR(st->latch_gpiod);
    if (!st->latch_gpiod)
        dev_warn(&spi->dev, "LATCH GPIO recommended for reliable operation\n");

    /* CLEAR: active HIGH, idle LOW */
    st->clear_gpiod = devm_gpiod_get_optional(&spi->dev, "clear", GPIOD_OUT_LOW);
    if (IS_ERR(st->clear_gpiod)) 
        return PTR_ERR(st->clear_gpiod);

    st->fault_gpiod = devm_gpiod_get_optional(&spi->dev, "fault", GPIOD_IN);
    if (IS_ERR(st->fault_gpiod)) 
        return PTR_ERR(st->fault_gpiod);
    
    if (st->fault_gpiod) {
        st->fault_irq = gpiod_to_irq(st->fault_gpiod);
        if (st->fault_irq < 0) {
            dev_warn(&spi->dev, "ad5420: fault gpio present but irq mapping failed\n");
            st->fault_irq = 0;
        }
    }

    return 0;
}

static const struct iio_info ad5420_info = {
    .read_raw = ad5420_read_raw,
    .write_raw = ad5420_write_raw,
    .read_event_config = ad5420_read_event_config,
    .write_event_config = ad5420_write_event_config,
    .read_event_value = ad5420_read_event_value,
};

static int ad5420_probe(struct spi_device *spi)
{
    struct ad5420_state *st;
    struct iio_dev *indio_dev;
    int ret;
    unsigned int ctrl = 0;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
    if (!indio_dev) 
        return -ENOMEM;

    st = iio_priv(indio_dev);
    st->spi = spi;
    mutex_init(&st->lock);

    spi_set_drvdata(spi, indio_dev);

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    if (spi->max_speed_hz == 0 || spi->max_speed_hz > 30000000)
        spi->max_speed_hz = 20000000;  /* Safe max */

    ret = spi_setup(spi);
    if (ret) return ret;

    st->ctrl_cache = 0;
    st->fault_mask = 0;
    
    ret = ad5420_parse_dt(spi, st);
    if (ret) return ret;

    indio_dev->dev.parent = &spi->dev;
    indio_dev->name = "ad5420";
    indio_dev->info = &ad5420_info;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = ad5420_channels;
    indio_dev->num_channels = ARRAY_SIZE(ad5420_channels);

    /* Reset */
    ret = ad5420_spi_write_frame(indio_dev, AD5420_REG_RESET, 0x0001);
    if (ret) 
        dev_warn(&spi->dev, "ad5420: reset failed %d\n", ret);
    else 
        usleep_range(100, 200);

    /* Control word */
    if (of_property_read_bool(spi->dev.of_node, "adi,external-rset"))
        ctrl |= AD5420_CTRL_REXT;

    switch (st->current_range) {
    case AD5420_RANGE_4mA_20mA: ctrl |= 0x5; break;
    case AD5420_RANGE_0mA_20mA: ctrl |= 0x6; break;
    case AD5420_RANGE_0mA_24mA: ctrl |= 0x7; break;
    }

    ctrl |= AD5420_CTRL_OUTEN;

    ret = ad5420_write_control(indio_dev, ctrl);
    if (ret) 
        dev_warn(&spi->dev, "ad5420: write control failed %d\n", ret);
    else 
        dev_info(&spi->dev, "ad5420: control initialized to 0x%04x\n", ctrl);

    /* Полная диагностика после инициализации */
    dev_info(&spi->dev, "=== RUNNING AD5420 DIAGNOSTICS ===\n");
    ad5420_full_diagnose(indio_dev);
    
    /* Аппаратный тест */
    dev_info(&spi->dev, "=== RUNNING AD5420 HARDWARE TEST ===\n");
    ret = ad5420_hardware_test(indio_dev);
    if (ret) {
        dev_err(&spi->dev, "AD5420 hardware test failed: %d\n", ret);
        /* Не прерываем probe, продолжаем регистрацию */
    } else {
        dev_info(&spi->dev, "AD5420 hardware test passed\n");
    }
    
    /* Финальная диагностика после теста */
    dev_info(&spi->dev, "=== FINAL DIAGNOSTICS AFTER TEST ===\n");
    ad5420_full_diagnose(indio_dev);

    if (st->fault_irq > 0) {
        ret = devm_request_threaded_irq(&spi->dev, st->fault_irq, NULL,
                                        ad5420_fault_handler, 
                                        IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                        "ad5420-fault", indio_dev);
        if (ret) 
            dev_err(&spi->dev, "ad5420: request fault irq failed %d\n", ret);
    }

    ret = devm_iio_device_register(&spi->dev, indio_dev);
    if (ret) return ret;

    dev_info(&spi->dev, "ad5420: probed (range: %s)\n",
             st->current_range == AD5420_RANGE_4mA_20mA ? "4-20mA" :
             st->current_range == AD5420_RANGE_0mA_20mA ? "0-20mA" : "0-24mA");
    
    return 0;
}

static int ad5420_remove(struct spi_device *spi)
{
    dev_info(&spi->dev, "ad5420: removed\n");
    return 0;
}

static const struct of_device_id ad5420_of_match[] = {
    { .compatible = "adi,ad5420", },
    { .compatible = "adi,ad5410", },
    { }
};
MODULE_DEVICE_TABLE(of, ad5420_of_match);

static struct spi_driver ad5420_driver = {
    .driver = {
        .name = "ad5420",
        .of_match_table = ad5420_of_match,
    },
    .probe = ad5420_probe,
    .remove = ad5420_remove,
};
module_spi_driver(ad5420_driver);

MODULE_AUTHOR("Adapted for user");
MODULE_DESCRIPTION("AD5420/AD5410 IIO DAC driver (SPI 24-bit frames, LATCH/CLEAR support)");
MODULE_LICENSE("GPL v2");
