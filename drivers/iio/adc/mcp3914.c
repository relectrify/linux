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

#define MCP3914_NUM_CHANNELS 2
#define MCP3914_NUM_SCALES 6

static const int mcp3914_osr_table[] = {
	32, 64, 128, 256, 512, 1024, 2048, 4096
};
static u32 mcp3914_scale_table[MCP3914_NUM_SCALES][2];

static const u16 mcp3914_crc16_table[] = {
	0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
	0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
	0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
	0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
	0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
	0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
	0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
	0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
	0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
	0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
	0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
	0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
	0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
	0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
	0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
	0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
	0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
	0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
	0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
	0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
	0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
	0x0208, 0x820D, 0x8207, 0x0202
};

static inline u16 mcp3914_crc16_byte(u16 crc, const u8 data)
{
	u8 index = (crc >> 8) ^ data;
	return (crc << 8) ^ mcp3914_crc16_table[index];
}

static u16 mcp3914_crc16_calc(u16 crc, u8 const *buffer, size_t len)
{
	while (len--)
		crc = mcp3914_crc16_byte(crc, *buffer++);
	return crc;
}

#define MCP3914_CRC_INIT 0

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

	if (mcp3914_crc16_calc(MCP3914_CRC_INIT, adc->rx_buf,
			       sizeof(adc->rx_buf)) != 0) {
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
