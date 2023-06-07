#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/trigger.h>

#include <asm/unaligned.h>

#define MCP3914_REG_CHANNEL0 0x00
#define MCP3914_REG_CHANNEL(x) (MCP3914_REG_CHANNEL0 + x)
#define MCP3914_REG_MOD 0x08
#define MCP3914_REG_PHASE0 0x09
#define MCP3914_REG_PHASE1 0x0A
#define MCP3914_REG_GAIN 0x0B
#define MCP3914_REG_STATUSCOM 0x0C
#define MCP3914_REG_CONFIG0 0x0D
#define MCP3914_REG_CONFIG1 0x0E
#define MCP3914_REG_OFFCAL_CHANNEL0 0x0F
#define MCP3914_REG_GAINCAL_CHANNEL0 0x10
#define MCP3914_REG_OFFCAL_CHANNEL(x) (MCP3914_REG_OFFCAL_CHANNEL0 + x * 2)
#define MCP3914_REG_GAINCAL_CHANNEL(x) (MCP3914_REG_GAINCAL_CHANNEL0 + x * 2)
#define MCP3914_REG_LOCK_CRC 0x1F

// MOD
#define MCP3914_MOD_MASK(ch) (GENMASK(3, 0) << 3 * ch)

// PHASE
#define MCP3914_PHASE_MASK(pair) (GENMASK(11, 0) << 11 * pair)

// GAIN
#define MCP3914_GAIN_MASK(ch) (GENMASK(2, 0) << 3 * ch)
#define MCP3914_GAIN_VAL(ch, val) ((val << 3 * ch) & MCP3914_GAIN_MASK(ch))

// STATUSCOM
#define MCP3914_STATUSCOM_READ GENMASK(23, 22)
#define MCP3914_STATUSCOM_WRITE BIT(21)
#define MCP3914_STATUSCOM_DR_HIZ BIT(20)
#define MCP3914_STATUSCOM_DR_LINK BIT(19)
#define MCP3914_STATUSCOM_WIDTH_CRC BIT(18)
#define MCP3914_STATUSCOM_WIDTH_DATA GENMASK(17, 16)
#define MCP3914_STATUSCOM_EN_CRCCOM BIT(15)
#define MCP3914_STATUSCOM_EN_INT BIT(14)
#define MCP3914_STATUSCOM_DRSTATUS GENMASK(7, 0)

// CONFIG0
#define MCP3914_CONFIG0_EN_OFFCAL BIT(23)
#define MCP3914_CONFIG0_EN_GAINCAL BIT(22)
#define MCP3914_CONFIG0_DITHER GENMASK(21, 20)
#define MCP3914_CONFIG0_BOOST GENMASK(19, 18)
#define MCP3914_CONFIG0_PRE GENMASK(17, 16)
#define MCP3914_CONFIG0_OSR GENMASK(15, 13)
#define MCP3914_CONFIG0_VREFCAL GENMASK(7, 0)

// CONFIG1
#define MCP3914_CONFIG1_RESET GENMASK(23, 16)
#define MCP3914_CONFIG1_SHUTDOWN GENMASK(15, 8)
#define MCP3914_CONFIG1_VREFEXT BIT(7)
#define MCP3914_CONFIG1_CLKEXT BIT(6)

// LOCK/CRC
#define MCP3914_LOCKCRC_LOCK GENMASK(23, 16)
#define MCP3914_LOCKCRC_CRCREG GENMASK(15, 0)

/* Internal voltage reference in mV */
#define MCP3914_INT_VREF_MV 1200

#define MCP3914_REG_READ(reg, id) \
	((((reg) << 1) | ((id) << 6) | (1 << 0)) & 0xff)
#define MCP3914_REG_WRITE(reg, id) \
	((((reg) << 1) | ((id) << 6) | (0 << 0)) & 0xff)
#define MCP3914_REG_MASK GENMASK(4, 1)

#define MCP3914_NUM_CHANNELS 8
#define MCP3914_NUM_SCALES 6

static const int mcp3914_osr_table[] = {
	32, 64, 128, 256, 512, 1024, 2048, 4096
};
static u32 mcp3914_scale_table[MCP3914_NUM_SCALES][2];

struct mcp3914 {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	struct clk *clki;
	u32 dev_addr;
	struct iio_trigger *trig;
	u32 gain[MCP3914_NUM_CHANNELS];
	struct {
		s16 channels[MCP3914_NUM_CHANNELS];
		s64 ts __aligned(8);
	} scan;

	u8 tx_buf __aligned(IIO_DMA_MINALIGN);
	u8 rx_buf[MCP3914_NUM_CHANNELS * 2 + 2];
};

static int mcp3914_read(struct mcp3914 *adc, u8 reg, u32 *val, u8 len)
{
	int ret;

	reg = MCP3914_REG_READ(reg, adc->dev_addr);
	ret = spi_write_then_read(adc->spi, &reg, 1, val, len);
	if (ret < 0)
		return ret;

	be32_to_cpus(val);
	*val >>= ((4 - len) * 8);
	dev_dbg(&adc->spi->dev, "reading 0x%x from register 0x%lx\n", *val,
		FIELD_GET(MCP3914_REG_MASK, reg));
	return ret;
}

static int mcp3914_write(struct mcp3914 *adc, u8 reg, u32 val, u8 len)
{
	dev_dbg(&adc->spi->dev, "writing 0x%x to register 0x%x\n", val, reg);

	val <<= (3 - len) * 8;
	cpu_to_be32s(&val);
	val |= MCP3914_REG_WRITE(reg, adc->dev_addr);

	return spi_write(adc->spi, &val, len + 1);
}

static int mcp3914_update(struct mcp3914 *adc, u8 reg, u32 mask, u32 val,
			  u8 len)
{
	u32 tmp;
	int ret;

	ret = mcp3914_read(adc, reg, &tmp, len);
	if (ret)
		return ret;

	val &= mask;
	val |= tmp & ~mask;
	return mcp3914_write(adc, reg, val, len);
}

static int mcp3914_write_raw_get_fmt(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return IIO_VAL_INT;
	default:
		return IIO_VAL_INT_PLUS_NANO;
	}
}

static int mcp3914_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long info)
{
	switch (info) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*type = IIO_VAL_INT;
		*vals = mcp3914_osr_table;
		*length = ARRAY_SIZE(mcp3914_osr_table);
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SCALE:
		*type = IIO_VAL_INT_PLUS_NANO;
		*vals = (int *)mcp3914_scale_table;
		*length = ARRAY_SIZE(mcp3914_scale_table) * 2;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int mcp3914_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel, int *val,
			    int *val2, long mask)
{
	struct mcp3914 *adc = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&adc->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = mcp3914_read(adc, MCP3914_REG_CHANNEL(channel->channel),
				   val, 3);
		if (ret)
			goto out;

		*val = sign_extend32(*val, 23);

		ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_OFFSET:
		ret = mcp3914_read(adc,
				   MCP3914_REG_OFFCAL_CHANNEL(channel->channel),
				   val, 3);
		if (ret)
			goto out;

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		ret = mcp3914_read(adc, MCP3914_REG_CONFIG0, val, 3);
		if (ret)
			goto out;

		*val = FIELD_GET(MCP3914_CONFIG0_OSR, *val);
		*val = 32 << *val;
		ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		*val = mcp3914_scale_table[ilog2(adc->gain[channel->channel])]
					  [0];
		*val2 = mcp3914_scale_table[ilog2(adc->gain[channel->channel])]
					   [1];
		ret = IIO_VAL_INT_PLUS_NANO;
		break;
	}

out:
	mutex_unlock(&adc->lock);
	return ret;
}

static int mcp3914_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *channel, int val,
			     int val2, long mask)
{
	struct mcp3914 *adc = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&adc->lock);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		for (int i = 0; i < MCP3914_NUM_SCALES; i++) {
			// if (val == mcp3914_scale_table[i][0] &&
			//     val2 == mcp3914_scale_table[i][1]) {
			// 	adc->gain[channel->channel] = BIT(i);
			// 	ret = mcp3914_update(
			// 		adc, mcp3914_REG_GAIN,
			// 		mcp3914_GAIN_MASK(channel->channel),
			// 		mcp3914_GAIN_VAL(channel->channel, i),
			// 		1);
			// }
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		if (val2 != 0) {
			ret = -EINVAL;
			goto out;
		}

		/* Write offset */
		// ret = mcp3914_write(adc, mcp3914_OFFCAL(channel->channel), val,
		// 		    3);
		// if (ret)
		// 	goto out;

		/* Enable offset*/
		// ret = mcp3914_update(adc, mcp3914_REG_STATUSCOM,
		// 		     mcp3914_STATUSCOM_EN_OFFCAL,
		// 		     mcp3914_STATUSCOM_EN_OFFCAL, 2);
		break;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		for (int i = 0; i < ARRAY_SIZE(mcp3914_osr_table); i++) {
		}
		break;
	}

out:
	mutex_unlock(&adc->lock);
	return ret;
}

static int mcp3914_calc_scale_table(struct mcp3914 *adc)
{
	u32 ref = MCP3914_INT_VREF_MV;
	u32 div;
	int ret;
	u64 tmp;

	/*
	 * For 24-bit Conversion
	 * Raw = ((Voltage)/(Vref) * 2^23 * Gain * 1.5
	 * Voltage = Raw * (Vref)/(2^23 * Gain * 1.5)
	 *
	 * ref = Reference voltage
	 * div = (2^23 * 1.5 * gain) = 12582912 * gain
	 */
	for (int i = 0; i < MCP3914_NUM_SCALES; i++) {
		div = 12582912 * BIT(i);
		tmp = div_s64((s64)ref * 1000000000LL, div);

		mcp3914_scale_table[i][0] = 0;
		mcp3914_scale_table[i][1] = tmp;
	}

	return 0;
}

#define MCP3914_CHAN(idx)                                                 \
	{                                                                 \
		.type = IIO_VOLTAGE, .indexed = 1, .channel = idx,        \
		.scan_index = idx,                                        \
		.info_mask_shared_by_type =                               \
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),            \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |            \
				      BIT(IIO_CHAN_INFO_OFFSET) |         \
				      BIT(IIO_CHAN_INFO_SCALE),           \
		.info_mask_shared_by_type_available =                     \
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),            \
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE), \
		.scan_type = {                                            \
			.sign = 's',                                      \
			.realbits = 16,                                   \
			.storagebits = 16,                                \
			.endianness = IIO_BE,                             \
		},                                                        \
	}

static const struct iio_chan_spec mcp3914_channels[] = {
	MCP3914_CHAN(0),
	MCP3914_CHAN(1),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static irqreturn_t mcp3914_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct mcp3914 *adc = iio_priv(indio_dev);
	struct spi_transfer xfer[] = {
		{
			.tx_buf = &adc->tx_buf,
			.len = 1,
		},
		{
			.rx_buf = adc->rx_buf,
			.len = sizeof(adc->rx_buf),
		},
	};
	int scan_index;
	int i = 0;
	int ret;
	int read_len;

	mutex_lock(&adc->lock);
	adc->tx_buf = MCP3914_REG_READ(MCP3914_REG_CHANNEL(0), adc->dev_addr);

	ret = spi_sync_transfer(adc->spi, xfer, ARRAY_SIZE(xfer));
	if (ret < 0) {
		dev_warn(&adc->spi->dev, "failed to get conversion data\n");
		goto out;
	}

	for_each_set_bit(scan_index, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		const struct iio_chan_spec *scan_chan =
			&indio_dev->channels[scan_index];

		adc->scan.channels[i] = get_unaligned_be16(
			&adc->rx_buf[scan_chan->channel * 2]);
		i++;
	}
	iio_push_to_buffers_with_timestamp(indio_dev, &adc->scan,
					   iio_get_time_ns(indio_dev));
out:
	mutex_unlock(&adc->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_info mcp3914_info = {
	.read_raw = mcp3914_read_raw,
	.write_raw = mcp3914_write_raw,
	.read_avail = mcp3914_read_avail,
	.write_raw_get_fmt = mcp3914_write_raw_get_fmt,
};

static int mcp3914_config(struct mcp3914 *adc)
{
	struct device *dev = &adc->spi->dev;
	u32 regval;
	int ret;

	ret = device_property_read_u32(dev, "microchip,device-addr",
				       &adc->dev_addr);

	/*
	 * Fallback to "device-addr" due to historical mismatch between
	 * dt-bindings and implementation
	 */
	if (ret)
		device_property_read_u32(dev, "device-addr", &adc->dev_addr);
	if (adc->dev_addr > 3) {
		dev_err(&adc->spi->dev,
			"invalid device address (%i). Must be in range 0-3.\n",
			adc->dev_addr);
		return -EINVAL;
	}
	dev_dbg(&adc->spi->dev, "use device address %i\n", adc->dev_addr);

	ret = mcp3914_read(adc, MCP3914_REG_STATUSCOM, &regval, 3);
	if (ret)
		return ret;

	regval |= MCP3914_STATUSCOM_EN_CRCCOM; // Enable CRC
	regval &= ~MCP3914_STATUSCOM_WIDTH_CRC; // 16bit with rounding

	return mcp3914_write(adc, MCP3914_REG_STATUSCOM, regval, 3);
}

static int mcp3914_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	struct mcp3914 *adc = iio_trigger_get_drvdata(trig);

	if (enable)
		enable_irq(adc->spi->irq);
	else
		disable_irq(adc->spi->irq);

	return 0;
}

static const struct iio_trigger_ops mcp3914_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
	.set_trigger_state = mcp3914_set_trigger_state,
};

static int mcp3914_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct mcp3914 *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	ret = mcp3914_config(adc);
	if (ret)
		return ret;

	ret = mcp3914_calc_scale_table(adc);
	if (ret)
		return ret;

	/* Set gain to 1 for all channels */
	for (int i = 0; i < MCP3914_NUM_CHANNELS; i++) {
		adc->gain[i] = 1;
	}

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mcp3914_info;
	spi_set_drvdata(spi, indio_dev);

	indio_dev->channels = mcp3914_channels;
	indio_dev->num_channels = ARRAY_SIZE(mcp3914_channels);

	mutex_init(&adc->lock);

	if (spi->irq > 0) {
		adc->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
						   indio_dev->name,
						   iio_device_id(indio_dev));
		if (!adc->trig)
			return -ENOMEM;

		adc->trig->ops = &mcp3914_trigger_ops;
		iio_trigger_set_drvdata(adc->trig, adc);
		ret = devm_iio_trigger_register(&spi->dev, adc->trig);
		if (ret)
			return ret;

		/*
		 * The device generates interrupts as long as it is powered up.
		 * Some platforms might not allow the option to power it down so
		 * don't enable the interrupt to avoid extra load on the system.
		 */
		ret = devm_request_irq(&spi->dev, spi->irq,
				       &iio_trigger_generic_data_rdy_poll,
				       IRQF_NO_AUTOEN | IRQF_ONESHOT,
				       indio_dev->name, adc->trig);
		if (ret)
			return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL,
					      mcp3914_trigger_handler, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(&adc->spi->dev, indio_dev);
}

static const struct of_device_id mcp3914_dt_ids[] = {
	{ .compatible = "microchip,mcp3914" },
	{}
};
MODULE_DEVICE_TABLE(of, mcp3914_dt_ids);

static const struct spi_device_id mcp3914_id[] = { { "mcp3914", 0 }, {} };
MODULE_DEVICE_TABLE(spi, mcp3914_id);

static struct spi_driver mcp3914_driver = {
	.driver = {
		.name = "mcp3914",
		.of_match_table = mcp3914_dt_ids,
	},
	.probe = mcp3914_probe,
	.id_table = mcp3914_id,
};
module_spi_driver(mcp3914_driver);

MODULE_AUTHOR("Tyler Watkins <tjwato@gmail.com>");
MODULE_DESCRIPTION("Microchip Technology MCP3914");
MODULE_LICENSE("GPL v2");
