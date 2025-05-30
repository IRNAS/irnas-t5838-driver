/** @file t5838_aad_mode.c
 *
 * @brief A description of the module's purpose.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2023 Irnas. All rights reserved.
 */

#include "t5838.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(t5838_aad, CONFIG_T5838_LOG_LEVEL);

#define DT_DRV_COMPAT invensense_t5838_nrf_pdm

/* We use any instance macro here, despite driver only supports single PDM microphone */
#define T5838_INST_HAS_MICEN_OR(inst) DT_INST_NODE_HAS_PROP(inst, micen_gpios) ||
#define T5838_ANY_INST_HAS_MICEN      DT_INST_FOREACH_STATUS_OKAY(T5838_INST_HAS_MICEN_OR) 0

/* Although datasheet mentions that t5838 uses One Wire Protocol for configuration of AAD
 * functionality, it is really using two wires (THSEL and PDMCLK) for this, similar to the I2C
 * protocol. That's why FAKE2C names was used to describe this protocol. */
#define T5838_FAKE2C_START_PILOT_CLKS 10
#define T5838_FAKE2C_ZERO	      1 * T5838_FAKE2C_START_PILOT_CLKS
#define T5838_FAKE2C_ONE	      3 * T5838_FAKE2C_START_PILOT_CLKS
#define T5838_FAKE2C_STOP	      130 /* according to datasheet >128clk cycles */
#define T5838_FAKE2C_SPACE	      1 * T5838_FAKE2C_START_PILOT_CLKS

#define T5838_FAKE2C_POST_WRITE_CYCLES 60 /* According to datasheet >50clk cycles */
#define T5838_FAKE2C_PRE_WRITE_CYCLES  60 /* According to datasheet >50clk cycles */
#define T5838_FAKE2C_DEVICE_ADDRESS    0x53

#define T5838_FAKE2C_CLK_PERIOD_US 10 /* approx 100kHz */

/* >2ms of clock is required before entering sleep with AAD  */
#define T5838_ENTER_SLEEP_MODE_CLOCKING_TIME_US 2500
#define T5838_ENTER_SLEEP_MODE_CLK_PERIOD_US	10 /* approx 100kHz */

#define T5838_RESET_TIME_MS 10 /* Time required for device to reset when power is disabled*/

struct t5838_address_data_pair {
	uint8_t address;
	uint8_t data;
};

/**
 * @brief drive clock pin in bitbang fashion.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] cycles Number of cycles to clock.
 * @param[in] period Period of clock in microseconds.
 */
void prv_clock_bitbang(const struct device *dev, uint16_t cycles, uint16_t period)
{
	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	for (int i = 0; i < cycles; i++) {
		gpio_pin_set_dt(&drv_cfg->pdmclk, 1);
		k_busy_wait(period / 2);
		gpio_pin_set_dt(&drv_cfg->pdmclk, 0);
		k_busy_wait(period / 2);
	}
}

/**
 * @brief Function for writing to T5838 registers using their proprietary "one wire" that isn't
 * really one wire. We refer to this protocol as fake2c to avoid confusion with actual one wire
 * protocol.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write to register.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
int prv_reg_write(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	struct t5838_drv_data *pdm_data = drv_cfg->pdm_dev->data;
	if (pdm_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}

	LOG_DBG("prv_reg_write, reg: 0x%x, data: 0x%x", reg, data);

	/**prepare data */
	uint8_t wr_buf[] = {T5838_FAKE2C_DEVICE_ADDRESS << 1, reg, data};
	/* put into wr_buf since it gets written first */
	uint8_t cyc_buf;

	/** start with thsel low */
	gpio_pin_set_dt(&drv_cfg->thsel, 0);
	/** Clock device before writing to prepare device for communication */
	prv_clock_bitbang(dev, T5838_FAKE2C_PRE_WRITE_CYCLES, T5838_FAKE2C_CLK_PERIOD_US);
	/** write start condition*/
	gpio_pin_set_dt(&drv_cfg->thsel, 1);
	prv_clock_bitbang(dev, T5838_FAKE2C_START_PILOT_CLKS, T5838_FAKE2C_CLK_PERIOD_US);
	/** write first space before writing data */
	gpio_pin_set_dt(&drv_cfg->thsel, 0);
	prv_clock_bitbang(dev, T5838_FAKE2C_SPACE, T5838_FAKE2C_CLK_PERIOD_US);
	/** write data */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 8; j++) {

			if (wr_buf[i] & BIT(7 - j)) {
				/* sending one */
				cyc_buf = T5838_FAKE2C_ONE;
			} else {
				/* sending 0 */
				cyc_buf = T5838_FAKE2C_ZERO;
			}
			/** Send data bit */
			gpio_pin_set_dt(&drv_cfg->thsel, 1);
			prv_clock_bitbang(dev, cyc_buf, T5838_FAKE2C_CLK_PERIOD_US);
			/** Send space */
			gpio_pin_set_dt(&drv_cfg->thsel, 0);
			prv_clock_bitbang(dev, T5838_FAKE2C_SPACE, T5838_FAKE2C_CLK_PERIOD_US);
		}
	}
	/**write stop condition */
	gpio_pin_set_dt(&drv_cfg->thsel, 1);
	prv_clock_bitbang(dev, T5838_FAKE2C_STOP, T5838_FAKE2C_CLK_PERIOD_US);
	gpio_pin_set_dt(&drv_cfg->thsel, 0);

	/**keep clock to apply */
	prv_clock_bitbang(dev, T5838_FAKE2C_POST_WRITE_CYCLES, T5838_FAKE2C_CLK_PERIOD_US);
	return 0;
}

/**
 * @brief Function for writing to multiple T5838 registers from provided array of address data
 * pairs.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] data Array of address data pairs with sequence to write to device.
 * @param[in] num Number of address data pairs in data array.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
int prv_multi_reg_write(const struct device *dev, struct t5838_address_data_pair *data, uint8_t num)
{
	int err;
	for (int i = 0; i < num; i++) {
		err = prv_reg_write(dev, data[i].address, data[i].data);
		if (err < 0) {
			LOG_ERR("t5838 reg write err: %d, addr: 0x%x, data: 0x%x", err,
				data[i].address, data[i].data);
			return err;
		}
	}
	return 0;
}

/**
 * @brief Function helper for sending predefined sequence that unlocks aad mode.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
int prv_aad_unlock_sequence(const struct device *dev)
{
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;

	/** This is unlock sequence for AAD modes provided in datasheet. */
	struct t5838_address_data_pair write_data[] = {
		{0x5C, 0x00}, {0x3E, 0x00}, {0x6F, 0x00}, {0x3B, 0x00}, {0x4C, 0x00},
	};

	err = prv_multi_reg_write(dev, write_data, ARRAY_SIZE(write_data));
	if (err < 0) {
		LOG_ERR("prv_t5838_multi_reg_write, err: %d", err);
		return err;
	}
	drv_data->aad_unlocked = true;

	return 0;
}

void t5838_aad_sleep(const struct device *dev)
{
	prv_clock_bitbang(
		dev, T5838_ENTER_SLEEP_MODE_CLOCKING_TIME_US / T5838_ENTER_SLEEP_MODE_CLK_PERIOD_US,
		T5838_ENTER_SLEEP_MODE_CLK_PERIOD_US);
}

static void prv_wake_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct t5838_aad_drv_data *drv_data = CONTAINER_OF(cb, struct t5838_aad_drv_data, wake_cb);
	const struct t5838_aad_drv_cfg *drv_cfg = drv_data->aad_cfg;
	if (drv_data->int_handled) {
		return;
	}
	drv_data->int_handled = true;
	int err = 0;
	err = gpio_pin_interrupt_configure_dt(&drv_cfg->wake, GPIO_INT_DISABLE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure, err: %d", err);
		return;
	}
	LOG_INF("WAKE PIN INTERRUPT");

	if (drv_data->wake_handler) {
		drv_data->wake_handler(dev);
	}
}

int t5838_reset(const struct device *dev)
{

	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	struct t5838_aad_drv_data *drv_data = dev->data;
	if (!drv_cfg->micen_available) {
		return -ENOTSUP;
	}
	gpio_pin_set_dt(&drv_cfg->micen, 0);

	drv_data->aad_unlocked = false;
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_NONE;

	/* Wait for device to power down and reapply power to reset it */
	k_sleep(K_MSEC(T5838_RESET_TIME_MS));
	gpio_pin_set_dt(&drv_cfg->micen, 1);

	return 0;
}

int t5838_aad_wake_clear(const struct device *dev)
{
	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;
	if (drv_data->cb_configured) {
		err = gpio_pin_interrupt_configure_dt(&drv_cfg->wake, GPIO_INT_LEVEL_ACTIVE);
		if (err) {
			LOG_ERR("gpio_pin_interrupt_configure, err: %d", err);
			return err;
		}
	}
	drv_data->int_handled = false;
	return 0;
}

void t5838_aad_wake_handler_set(const struct device *dev, t5838_wake_handler_t handler)
{
	struct t5838_aad_drv_data *drv_data = dev->data;
	drv_data->wake_handler = handler;
}

/**
 * @brief Function for setting AAD mode from provided array of address data pairs.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] write_array Array of address data pairs with setup sequence to write to device.
 * @param[in] write_array_size Size of write_array.
 *
 * @retval 0 if successful.
 * @retval negative errno code if otherwise.
 */
int prv_aad_mode_set(const struct device *dev, struct t5838_address_data_pair *write_array,
		     uint8_t write_array_size)
{
	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;

	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	struct t5838_drv_data *pdm_data = drv_cfg->pdm_dev->data;
	if (pdm_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}
	if (drv_data->aad_unlocked == false) {
		err = prv_aad_unlock_sequence(dev);
		if (err < 0) {
			LOG_ERR("error writing aad unlock sequence, err: %d", err);
			return err;
		}
	}

	err = prv_multi_reg_write(dev, write_array, write_array_size);
	if (err < 0) {
		LOG_ERR("prv_t5838_multi_reg_write, err: %d", err);
		return err;
	}

	t5838_aad_sleep(dev);

	pdm_data->aad_child_dev = dev;
	/**Configure interrupts */
	if (drv_data->cb_configured == false) {
		drv_data->int_handled = false;
		gpio_init_callback(&drv_data->wake_cb, prv_wake_cb_handler, BIT(drv_cfg->wake.pin));

		err = gpio_add_callback(drv_cfg->wake.port, &drv_data->wake_cb);
		if (err) {
			LOG_ERR("gpio_add_callback, err: %d", err);
			return err;
		}
		drv_data->cb_configured = true;
	}
	err = gpio_pin_interrupt_configure_dt(&drv_cfg->wake, GPIO_INT_LEVEL_ACTIVE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}
	return 0;
}

int t5838_aad_a_mode_set(const struct device *dev, struct t5838_aad_a_conf *aadconf)
{
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;

	struct t5838_address_data_pair write_data[] = {
		{T5838_REG_AAD_MODE, 0x00},
		{T5838_REG_AAD_A_LPF, aadconf->aad_a_lpf},
		{T5838_REG_AAD_A_THR, aadconf->aad_a_thr},
		{T5838_REG_AAD_MODE, T5838_AAD_SELECT_A},
	};
	err = prv_aad_mode_set(dev, write_data, ARRAY_SIZE(write_data));
	if (err < 0) {
		LOG_ERR("prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_A;
	return 0;
}

/** Helper macro to avoid code duplication in D1 and D2 configuration functions */
#define AAD_D_CONFIG_BUILDER(write_data, mode)                                                     \
	struct t5838_address_data_pair write_data[] = {                                            \
		{T5838_REG_AAD_MODE, 0x00},                                                        \
		{T5838_REG_AAD_D_FLOOR_HI, (aadconf->aad_d_floor >> 8) & 0x1F},                    \
		{T5838_REG_AAD_D_FLOOR_LO, aadconf->aad_d_floor & 0xFF},                           \
		{0x2C, 0x32},                                                                      \
		{0x2D, 0xC0},                                                                      \
		{T5838_REG_AAD_D_REL_PULSE_MIN_LO, aadconf->aad_d_rel_pulse_min & 0xFF},           \
		{T5838_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED,                                         \
		 ((aadconf->aad_d_abs_pulse_min >> 4) / 0xF0) |                                    \
			 ((aadconf->aad_d_rel_pulse_min >> 8) & 0x0F)},                            \
		{T5838_REG_AAD_D_ABS_PULSE_MIN_LO, aadconf->aad_d_abs_pulse_min & 0xFF},           \
		{T5838_REG_AAD_D_ABS_THR_LO, aadconf->aad_d_abs_thr & 0xFF},                       \
		{T5838_REG_AAD_D_ABS_THR_HI, ((aadconf->aad_d_abs_thr >> 8) & 0x1F) | 0x40},       \
		{T5838_REG_AAD_D_REL_THR, aadconf->aad_d_rel_thr},                                 \
		{T5838_REG_AAD_MODE, mode},                                                        \
	}

int t5838_aad_d1_mode_set(const struct device *dev, struct t5838_aad_d_conf *aadconf)
{
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;
	AAD_D_CONFIG_BUILDER(write_data, T5838_AAD_SELECT_D1);
	err = prv_aad_mode_set(dev, write_data, ARRAY_SIZE(write_data));
	if (err < 0) {
		LOG_ERR("prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_D1;
	return 0;
}

int t5838_aad_d2_mode_set(const struct device *dev, struct t5838_aad_d_conf *aadconf)
{
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;
	AAD_D_CONFIG_BUILDER(write_data, T5838_AAD_SELECT_D2);
	err = prv_aad_mode_set(dev, write_data, ARRAY_SIZE(write_data));
	if (err < 0) {
		LOG_ERR("prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_D2;
	return 0;
}

int t5838_aad_mode_disable(const struct device *dev)
{

	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	struct t5838_aad_drv_data *drv_data = dev->data;
	struct t5838_drv_data *pdm_data = drv_cfg->pdm_dev->data;
	int err;
	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_reg_write(dev, T5838_REG_AAD_MODE, 0);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	/** Disable interrupts */
	err = gpio_pin_interrupt_configure_dt(&drv_cfg->wake, GPIO_INT_DISABLE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_NONE;
	drv_data->aad_unlocked = false;
	pdm_data->aad_child_dev = NULL;
	return 0;
}

/**
 * @brief Function for initializing T5838 AAD trigger device. Called during device boot.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code if otherwise.
 */
static int t5838_aad_init(const struct device *dev)
{
	const struct t5838_aad_drv_cfg *drv_cfg = dev->config;
	struct t5838_aad_drv_data *drv_data = dev->data;
	int err;
	/* T5838 specific configuration */
	err = gpio_pin_configure_dt(&drv_cfg->wake, GPIO_INPUT);
	if (err < 0) {
		LOG_ERR("gpio_pin_configure_dt, err: %d", err);
		return err;
	}
	err = gpio_pin_configure_dt(&drv_cfg->thsel, GPIO_OUTPUT);
	if (err < 0) {
		LOG_ERR("gpio_pin_configure_dt, err: %d", err);
		return err;
	}
	err = gpio_pin_configure_dt(&drv_cfg->pdmclk, GPIO_OUTPUT);
	if (err < 0) {
		LOG_ERR("gpio_pin_configure_dt, err: %d", err);
		return err;
	}
	/* set micen low to turn of microphone and make sure we start in reset state*/
	if (drv_cfg->micen_available) {
		err = gpio_pin_configure_dt(&drv_cfg->micen, GPIO_OUTPUT);

		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}

		t5838_reset(dev);
	}
	drv_data->aad_cfg = drv_cfg;
	return 0;
}

#define T5838_AAD_DEVICE(idx)                                                                      \
	static struct t5838_aad_drv_data t5838_aad_data_##idx;                                     \
	static const struct t5838_aad_drv_cfg t5838_aad_cfg##idx = {                               \
		.pdm_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0)),                                      \
		.micen = GPIO_DT_SPEC_INST_GET_OR(idx, micen_gpios, {}),                           \
		.micen_available = DT_INST_NODE_HAS_PROP(idx, micen_gpios),                        \
		.wake = GPIO_DT_SPEC_INST_GET_OR(idx, wake_gpios, {}),                             \
		.thsel = GPIO_DT_SPEC_INST_GET_OR(idx, thsel_gpios, {}),                           \
		.pdmclk = GPIO_DT_SPEC_INST_GET_OR(idx, pdmclk_gpios, {}),                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, &t5838_aad_init, NULL, &t5838_aad_data_##idx,                   \
			      &t5838_aad_cfg##idx, POST_KERNEL, CONFIG_T5838_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(T5838_AAD_DEVICE);