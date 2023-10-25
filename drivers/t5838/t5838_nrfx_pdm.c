/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "t5838.h"

#include <zephyr/audio/dmic.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <hal/nrf_gpio.h>
#include <nrfx_pdm.h>
#include <soc.h>

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(t5838, CONFIG_T5838_LOG_LEVEL);

#ifdef CONFIG_AUDIO_DMIC_NRFX_PDM
BUILD_ASSERT(CONFIG_AUDIO_DMIC_NRFX_PDM == 0,
	     "Both T5838 and nrfx_pdm drivers enabled, set "
	     "CONFIG_AUDIO_DMIC_NRFX_PDM to false to use T5838 driver");
#endif

#define DT_DRV_COMPAT tdk_t5838_nrf_pdm

const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* We use any instance macro here, despite driver only supports single PDM microphone */
#define T5838_INST_HAS_MICEN_OR(inst) DT_INST_NODE_HAS_PROP(inst, micen_gpios) ||
#define T5838_ANY_INST_HAS_MICEN      DT_INST_FOREACH_STATUS_OKAY(T5838_INST_HAS_MICEN_OR) 0

#define T5838_INST_HAS_THSEL_OR(inst) DT_INST_NODE_HAS_PROP(inst, thsel_gpios) ||
#define T5838_ANY_INST_HAS_THSEL      DT_INST_FOREACH_STATUS_OKAY(T5838_INST_HAS_THSEL_OR) 0

#define T5838_INST_HAS_WAKE_OR(inst) DT_INST_NODE_HAS_PROP(inst, wake_gpios) ||
#define T5838_ANY_INST_HAS_WAKE	     DT_INST_FOREACH_STATUS_OKAY(T5838_INST_HAS_WAKE_OR) 0

#define T5838_INST_HAS_PDMCLK_OR(inst) DT_INST_NODE_HAS_PROP(inst, pdmclk_gpios) ||
#define T5838_ANY_INST_HAS_PDMCLK      DT_INST_FOREACH_STATUS_OKAY(T5838_INST_HAS_WAKE_OR) 0

#define T5838_ANY_INST_AAD_CAPABLE                                                                 \
	T5838_ANY_INST_HAS_THSEL &&T5838_ANY_INST_HAS_WAKE &&T5838_ANY_INST_HAS_PDMCLK

struct t5838_drv_data {
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	struct k_mem_slab *mem_slab;
	uint32_t block_size;
	struct k_msgq rx_queue;
	bool request_clock : 1;
	bool configured : 1;
	volatile bool active;
	volatile bool stopping;

/* T5838 specific data */
#if T5838_ANY_INST_AAD_CAPABLE
	bool aad_unlocked;
	enum t5838_aad_select aad_enabled_mode;

	struct gpio_callback wake_cb;
	bool cb_configured;
	bool int_handled;
	t5838_wake_handler_t wake_handler;
#endif
};

struct t5838_drv_cfg {
	nrfx_pdm_event_handler_t event_handler;
	nrfx_pdm_config_t nrfx_def_cfg;
	const struct pinctrl_dev_config *pcfg;
	enum clock_source {
		PCLK32M,
		PCLK32M_HFXO,
		ACLK
	} clk_src;

/* T5838 specific config */
#if T5838_ANY_INST_HAS_MICEN
	const struct gpio_dt_spec *micen;
#endif
#if T5838_ANY_INST_HAS_WAKE
	const struct gpio_dt_spec *wake;
#endif
#if T5838_ANY_INST_HAS_THSEL
	const struct gpio_dt_spec *thsel;
#endif
#if T5838_ANY_INST_HAS_PDMCLK
	const struct gpio_dt_spec *pdmclk;
#endif
};

/**
 * @brief Function for writing to T5838 registers using their proprietary "one wire" that isn't
 * really one wire. We refer to this protocol as fake2c to avoid confusion with actual one wire
 * protocol.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param reg Register address to write to.
 * @param data Data to write to register.
 *
 * @retval 0 If successful.
 */
int prv_t5838_reg_write(const struct device *dev, uint8_t reg, uint8_t data)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	struct t5838_drv_data *drv_data = dev->data;
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	int err;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	if (drv_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}

	/**prepare data */
	uint8_t wr_buf[] = {T5838_FAKE2C_DEVICE_ADDRESS << 1, reg, data

	};
	/* put into wr_buf since it gets written first */
	uint8_t cyc_buf;

	/** start with thsel low */
	err = gpio_pin_set_dt(drv_cfg->thsel, 0);
	if (err < 0) {
		LOG_ERR("gpio_pin_set_dt, err: %d", err);
		return err;
	}
	/** Clock device before writing to prepare device for communication */

	for (int i = 0; i < T5838_FAKE2C_PRE_WRITE_CYCLES; i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
	}
	/** write start condition*/
	err = gpio_pin_set_dt(drv_cfg->thsel, 1);
	if (err < 0) {
		LOG_ERR("gpio_pin_set_dt, err: %d", err);
		return err;
	}
	for (int i = 0; i < T5838_FAKE2C_START_PILOT_CLKS; i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
	}
	/** write first space before writing data */
	err = gpio_pin_set_dt(drv_cfg->thsel, 0);
	if (err < 0) {
		LOG_ERR("gpio_pin_set_dt, err: %d", err);
		return err;
	}
	for (int i = 0; i < T5838_FAKE2C_SPACE; i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
	}
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
			err = gpio_pin_set_dt(drv_cfg->thsel, 1);
			if (err < 0) {
				LOG_ERR("gpio_pin_set_dt, err: %d", err);
				return err;
			}
			for (int k = 0; k < cyc_buf; k++) {
				err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
				if (err < 0) {
					LOG_ERR("gpio_pin_set_dt, err: %d", err);
					return err;
				}
				k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
				err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
				if (err < 0) {
					LOG_ERR("gpio_pin_set_dt, err: %d", err);
					return err;
				}
				k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
			}
			/** Send space */
			err = gpio_pin_set_dt(drv_cfg->thsel, 0);
			if (err < 0) {
				LOG_ERR("gpio_pin_set_dt, err: %d", err);
				return err;
			}
			for (int k = 0; k < T5838_FAKE2C_SPACE; k++) {
				err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
				if (err < 0) {
					LOG_ERR("gpio_pin_set_dt, err: %d", err);
					return err;
				}
				k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
				err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
				if (err < 0) {
					LOG_ERR("gpio_pin_set_dt, err: %d", err);
					return err;
				}
				k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
			}
		}
	}
	/**write stop condition */
	err = gpio_pin_set_dt(drv_cfg->thsel, 1);
	if (err < 0) {
		LOG_ERR("gpio_pin_set_dt, err: %d", err);
		return err;
	}
	for (int i = 0; i < T5838_FAKE2C_STOP; i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
	}
	err = gpio_pin_set_dt(drv_cfg->thsel, 0);
	if (err < 0) {
		LOG_ERR("gpio_pin_set_dt, err: %d", err);
		return err;
	}

	/**keep clock to apply */
	for (int i = 0; i < T5838_FAKE2C_POST_WRITE_CYCLES; i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_FAKE2C_CLK_PERIOD_US / 2);
	}
	return 0;
#endif
}

/**
 * @brief Function helper for initialization of device GPIOs.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 */
int prv_t5838_configure_gpios(const struct device *dev)
{
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	int err;
	/* T5838 specific configuration */
#if T5838_ANY_INST_HAS_WAKE
	err = gpio_pin_configure_dt(drv_cfg->wake, GPIO_INPUT);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}
	}
#endif

#if T5838_ANY_INST_HAS_THSEL
	err = gpio_pin_configure_dt(drv_cfg->thsel, GPIO_OUTPUT);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}
	}
#endif

#if T5838_ANY_INST_HAS_PDMCLK
	err = gpio_pin_configure_dt(drv_cfg->pdmclk, GPIO_OUTPUT);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}
	}
#endif

#if T5838_ANY_INST_HAS_MICEN
	/* set micen low to turn of microphone and make sure we start in reset state*/
	err = gpio_pin_configure_dt(drv_cfg->micen, GPIO_OUTPUT);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}
	}
	err = gpio_pin_set_dt(drv_cfg->micen, 1);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
	}
#endif /*T5838_ANY_INST_HAS_MICEN*/
	return 0;
}

/**
 * @brief Function helper for sending predefined sequence that unlocks aad mode.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 */
int prv_t5838_aad_unlock_sequence(const struct device *dev)
{
	struct t5838_drv_data *drv_data = dev->data;
	int err;

	if (drv_data->aad_unlocked == false) {
		/** This is unlock sequence for AAD modes provided in datasheet. */
		err = prv_t5838_reg_write(dev, 0x5C, 0x00);
		if (err < 0) {
			LOG_ERR("prv_t5838_reg_write, err: %d", err);
			return err;
		}
		err = prv_t5838_reg_write(dev, 0x3E, 0x00);
		if (err < 0) {
			LOG_ERR("prv_t5838_reg_write, err: %d", err);
			return err;
		}
		err = prv_t5838_reg_write(dev, 0x6F, 0x00);
		if (err < 0) {
			LOG_ERR("prv_t5838_reg_write, err: %d", err);
			return err;
		}
		err = prv_t5838_reg_write(dev, 0x3B, 0x00);
		if (err < 0) {
			LOG_ERR("prv_t5838_reg_write, err: %d", err);
			return err;
		}
		err = prv_t5838_reg_write(dev, 0x4C, 0x00);
		if (err < 0) {
			LOG_ERR("prv_t5838_reg_write, err: %d", err);
			return err;
		}
		drv_data->aad_unlocked = true;
	}
	return 0;
}

/**
 * @brief Function for putting T5838 into sleep mode with AAD is enabled. Must be called after
 * writing to AAD registers.
 *
 * function clocks device for value set in T5838_ENTER_SLEEP_MODE_CLOCKING_TIME_uS to enable AAD.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 */
int prv_t5838_enter_sleep_with_AAD(const struct device *dev)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	int err;
	for (int i = 0;
	     i < T5838_ENTER_SLEEP_MODE_CLOCKING_TIME_uS / T5838_ENTER_SLEEP_MODE_CLK_PERIOD_uS;
	     i++) {
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 1);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_ENTER_SLEEP_MODE_CLK_PERIOD_uS / 2);
		err = gpio_pin_set_dt(drv_cfg->pdmclk, 0);
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
		k_busy_wait(T5838_ENTER_SLEEP_MODE_CLK_PERIOD_uS / 2);
	}
	return 0;
#endif
}

#if T5838_ANY_INST_AAD_CAPABLE

/** this are here because we need to access device struct from callback handler which uses different
 * device context */
const struct t5838_drv_cfg *g_drv_cfg = NULL;
struct t5838_drv_data *g_drv_data = NULL;
static void prv_wake_cb_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (g_drv_data->int_handled) {
		return;
	}
	g_drv_data->int_handled = true;
	int err = 0;
	err = gpio_pin_interrupt_configure_dt(g_drv_cfg->wake, GPIO_INT_DISABLE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure, err: %d", err);
		return;
	}
	LOG_INF("WAKE PIN INTERRUPT");

	if (g_drv_data->wake_handler) {
		g_drv_data->wake_handler(dev);
	}
}
#endif

int t5838_reset(const struct device *dev)
{
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct t5838_drv_data *drv_data = dev->data;
	int err;
#if T5838_ANY_INST_HAS_MICEN
	/* set micen low to turn of microphone and make sure we start in reset state*/
	err = gpio_pin_configure_dt(drv_cfg->micen, GPIO_OUTPUT);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_configure_dt, err: %d", err);
			return err;
		}
	}
	err = gpio_pin_set_dt(drv_cfg->micen, 0);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
	}
#if T5838_ANY_INST_AAD_CAPABLE
	drv_data->aad_unlocked = false;
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_NONE;
#endif /**T5838_ANY_INST_AAD_CAPABLE */
	/* Wait for device to power down and reapply power to reset it */
	k_sleep(K_MSEC(T5838_RESET_TIME_MS));
	err = gpio_pin_set_dt(drv_cfg->micen, 1);
	if (err < 0) {
		if (err < 0) {
			LOG_ERR("gpio_pin_set_dt, err: %d", err);
			return err;
		}
	}
	return 0;
#else /*T5838_ANY_INST_HAS_MICEN*/
	return -ENOTSUP;
#endif
}

int t5838_wake_clear(const struct device *dev)
{
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct t5838_drv_data *drv_data = dev->data;
	int err;
	if (drv_data->cb_configured) {
		err = gpio_pin_interrupt_configure_dt(drv_cfg->wake, GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			LOG_ERR("gpio_pin_interrupt_configure, err: %d", err);
			return err;
		}
	}
	drv_data->int_handled = false;
	return 0;
}

void t5838_wake_handler_set(const struct device *dev, t5838_wake_handler_t handler)
{
	struct t5838_drv_data *drv_data = dev->data;
	drv_data->wake_handler = handler;
}

int t5838_aad_a_mode_set(const struct device *dev, struct t5838_aad_a_conf *aadconf)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct t5838_drv_data *drv_data = dev->data;
	int err;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	if (drv_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}

	err = prv_t5838_configure_gpios(dev);
	if (err < 0) {
		LOG_ERR("gpio configuration error, err: %d", err);
		return err;
	}

	/** if AAD mode is selected, first write unlock sequence */
	err = prv_t5838_aad_unlock_sequence(dev);
	if (err < 0) {
		LOG_ERR("error writing aad unlock sequence, err: %d", err);
		return err;
	}

	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, 0);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_A_LPF, aadconf->aad_a_lpf);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_A_THR, aadconf->aad_a_thr);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, T5838_AAD_SELECT_A);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T5838_AAD_SELECT_A;

	err = prv_t5838_enter_sleep_with_AAD(dev);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	/**Configure interrupts */

	if (drv_data->cb_configured == false) {
		drv_data->int_handled = false;
		gpio_init_callback(&drv_data->wake_cb, prv_wake_cb_handler,
				   BIT(drv_cfg->wake->pin));

		err = gpio_add_callback(drv_cfg->wake->port, &drv_data->wake_cb);
		if (err) {
			LOG_ERR("gpio_add_callback, err: %d", err);
			return err;
		}
		drv_data->cb_configured = true;
		g_drv_data = drv_data;
		g_drv_cfg = drv_cfg;
	}
	err = gpio_pin_interrupt_configure_dt(drv_cfg->wake, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}

	return 0;
#endif
}

int t5838_aad_d1_mode_set(const struct device *dev, struct t5838_aad_d_conf *aadconf)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct t5838_drv_data *drv_data = dev->data;
	int err;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	if (drv_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}

	err = prv_t5838_configure_gpios(dev);
	if (err < 0) {
		LOG_ERR("gpio configuration error, err: %d", err);
		return err;
	}

	/** if AAD mode is selected, first write unlock sequence */
	err = prv_t5838_aad_unlock_sequence(dev);
	if (err < 0) {
		LOG_ERR("error writing aad unlock sequence, err: %d", err);
		return err;
	}
	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, 0);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_THR_LO, aadconf->aad_d_abs_thr & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_THR_HI,
				  (aadconf->aad_d_abs_thr >> 8) & 0x1F);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_FLOOR_LO, aadconf->aad_d_floor & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_FLOOR_HI,
				  (aadconf->aad_d_floor >> 8) & 0x1F);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_REL_PULSE_MIN_LO,
				  aadconf->aad_d_rel_pulse_min & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED,
				  ((aadconf->aad_d_rel_pulse_min >> 4) / 0xF0) |
					  (aadconf->aad_d_rel_pulse_min & 0x0F));
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_PULSE_MIN_LO,
				  aadconf->aad_d_abs_pulse_min & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_REL_THR, aadconf->aad_d_rel_thr);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, T5838_AAD_SELECT_D1);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_enter_sleep_with_AAD(dev);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	/**Configure interrupts */

	if (drv_data->cb_configured == false) {
		drv_data->int_handled = false;
		gpio_init_callback(&drv_data->wake_cb, prv_wake_cb_handler,
				   BIT(drv_cfg->wake->pin));

		err = gpio_add_callback(drv_cfg->wake->port, &drv_data->wake_cb);
		if (err) {
			LOG_ERR("gpio_add_callback, err: %d", err);
			return err;
		}
		drv_data->cb_configured = true;
		g_drv_data = drv_data;
		g_drv_cfg = drv_cfg;
	}
	err = gpio_pin_interrupt_configure_dt(drv_cfg->wake, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}
	return 0;
#endif
}

int t5838_aad_d2_mode_set(const struct device *dev, struct t5838_aad_d_conf *aadconf)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct t5838_drv_data *drv_data = dev->data;
	int err;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	if (drv_data->active) {
		LOG_ERR("Cannot write to device while pdm is active");
		return -EBUSY;
	}

	err = prv_t5838_configure_gpios(dev);
	if (err < 0) {
		LOG_ERR("gpio configuration error, err: %d", err);
		return err;
	}

	/** if AAD mode is selected, first write unlock sequence */
	err = prv_t5838_aad_unlock_sequence(dev);
	if (err < 0) {
		LOG_ERR("error writing aad unlock sequence, err: %d", err);
		return err;
	}
	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, 0);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_THR_LO, aadconf->aad_d_abs_thr & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_THR_HI,
				  (aadconf->aad_d_abs_thr >> 8) & 0x1F);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_FLOOR_LO, aadconf->aad_d_floor & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_FLOOR_HI,
				  (aadconf->aad_d_floor >> 8) & 0x1F);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_REL_PULSE_MIN_LO,
				  aadconf->aad_d_rel_pulse_min & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED,
				  ((aadconf->aad_d_rel_pulse_min >> 4) / 0xF0) |
					  (aadconf->aad_d_rel_pulse_min & 0x0F));
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_ABS_PULSE_MIN_LO,
				  aadconf->aad_d_abs_pulse_min & 0xFF);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_D_REL_THR, aadconf->aad_d_rel_thr);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}

	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, T5838_AAD_SELECT_D2);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	err = prv_t5838_enter_sleep_with_AAD(dev);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	/**Configure interrupts */

	if (drv_data->cb_configured == false) {
		drv_data->int_handled = false;
		gpio_init_callback(&drv_data->wake_cb, prv_wake_cb_handler,
				   BIT(drv_cfg->wake->pin));

		err = gpio_add_callback(drv_cfg->wake->port, &drv_data->wake_cb);
		if (err) {
			LOG_ERR("gpio_add_callback, err: %d", err);
			return err;
		}
		drv_data->cb_configured = true;
		g_drv_data = drv_data;
		g_drv_cfg = drv_cfg;
	}
	err = gpio_pin_interrupt_configure_dt(drv_cfg->wake, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}
	return 0;
#endif
}

int t5838_aad_mode_disable(const struct device *dev)
{
#if !T5838_ANY_INST_AAD_CAPABLE
	return -ENOTSUP;
#else
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	int err;
	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_t5838_reg_write(dev, T5838_REG_AAD_MODE, 0);
	if (err < 0) {
		LOG_ERR("prv_t5838_reg_write, err: %d", err);
		return err;
	}
	/** Disable interrupts */
	err = gpio_pin_interrupt_configure_dt(drv_cfg->wake, GPIO_INT_DISABLE);
	if (err) {
		LOG_ERR("gpio_pin_interrupt_configure_dt, err: %d", err);
		return err;
	}
	return 0;
#endif
}

/** From here bellow this is mostly modified copy of zephyr nrf pdm driver */

static void free_buffer(struct t5838_drv_data *drv_data, void *buffer)
{
	k_mem_slab_free(drv_data->mem_slab, &buffer);
	LOG_DBG("Freed buffer %p", buffer);
}

static void event_handler(const struct device *dev, const nrfx_pdm_evt_t *evt)
{
	struct t5838_drv_data *drv_data = dev->data;
	int ret;
	bool stop = false;

	if (evt->buffer_requested) {
		void *buffer;
		nrfx_err_t err;

		ret = k_mem_slab_alloc(drv_data->mem_slab, &buffer, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate buffer: %d", ret);
			stop = true;
		} else {
			err = nrfx_pdm_buffer_set(buffer, drv_data->block_size / 2);
			if (err != NRFX_SUCCESS) {
				LOG_ERR("Failed to set buffer: 0x%08x", err);
				stop = true;
			}
		}
	}

	if (drv_data->stopping) {
		if (evt->buffer_released) {
			free_buffer(drv_data, evt->buffer_released);
		}

		if (drv_data->active) {
			drv_data->active = false;
			if (drv_data->request_clock) {
				(void)onoff_release(drv_data->clk_mgr);
			}
		}
	} else if (evt->buffer_released) {
		ret = k_msgq_put(&drv_data->rx_queue, &evt->buffer_released, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("No room in RX queue");
			stop = true;

			free_buffer(drv_data, evt->buffer_released);
		} else {
			LOG_DBG("Queued buffer %p", evt->buffer_released);
		}
	}

	if (stop) {
		drv_data->stopping = true;
		nrfx_pdm_stop();
	}
}

static bool is_better(uint32_t freq, uint8_t ratio, uint32_t req_rate, uint32_t *best_diff,
		      uint32_t *best_rate, uint32_t *best_freq)
{
	uint32_t act_rate = freq / ratio;
	uint32_t diff = act_rate >= req_rate ? (act_rate - req_rate) : (req_rate - act_rate);

	LOG_DBG("Freq %u, ratio %u, act_rate %u", freq, ratio, act_rate);

	if (diff < *best_diff) {
		*best_diff = diff;
		*best_rate = act_rate;
		*best_freq = freq;
		return true;
	}

	return false;
}

/** This function was modified to no longer check if we are using nrf52 and locking pdm clock to
 * discrete enumerated values */
static bool check_pdm_frequencies(const struct t5838_drv_cfg *drv_cfg, nrfx_pdm_config_t *config,
				  const struct dmic_cfg *pdm_cfg, uint8_t ratio,
				  uint32_t *best_diff, uint32_t *best_rate, uint32_t *best_freq)
{
	uint32_t req_rate = pdm_cfg->streams[0].pcm_rate;
	bool better_found = false;

	const uint32_t src_freq = (NRF_PDM_HAS_MCLKCONFIG && drv_cfg->clk_src == ACLK)
					  /* The DMIC_NRFX_PDM_DEVICE() macro contains build
					   * assertions that make sure that the ACLK clock
					   * source is only used when it is available and only
					   * with the "hfclkaudio-frequency" property defined,
					   * but the default value of 0 here needs to be used
					   * to prevent compilation errors when the property is
					   * not defined (this expression will be eventually
					   * optimized away then).
					   */
					  ? DT_PROP_OR(DT_NODELABEL(clock), hfclkaudio_frequency, 0)
					  : 32 * 1000 * 1000UL;
	uint32_t req_freq = req_rate * ratio;
	/* As specified in the nRF5340 PS:
	 *
	 * PDMCLKCTRL = 4096 * floor(f_pdm * 1048576 /
	 *                           (f_source + f_pdm / 2))
	 * f_actual = f_source / floor(1048576 * 4096 / PDMCLKCTRL)
	 */
	uint32_t clk_factor = (uint32_t)((req_freq * 1048576ULL) / (src_freq + req_freq / 2));
	uint32_t act_freq = src_freq / (1048576 / clk_factor);

	if (act_freq >= pdm_cfg->io.min_pdm_clk_freq && act_freq <= pdm_cfg->io.max_pdm_clk_freq &&
	    is_better(act_freq, ratio, req_rate, best_diff, best_rate, best_freq)) {
		config->clock_freq = clk_factor * 4096;

		better_found = true;
	}

	return better_found;
}

/* Finds clock settings that give the PCM output rate closest to that requested,
 * taking into account the hardware limitations.
 */
static bool find_suitable_clock(const struct t5838_drv_cfg *drv_cfg, nrfx_pdm_config_t *config,
				const struct dmic_cfg *pdm_cfg)
{
	uint32_t best_diff = UINT32_MAX;
	uint32_t best_rate;
	uint32_t best_freq;

#if NRF_PDM_HAS_RATIO_CONFIG
	static const struct {
		uint8_t ratio_val;
		nrf_pdm_ratio_t ratio_enum;
	} ratios[] = {{64, NRF_PDM_RATIO_64X}, {80, NRF_PDM_RATIO_80X}};

	for (int r = 0; best_diff != 0 && r < ARRAY_SIZE(ratios); ++r) {
		uint8_t ratio = ratios[r].ratio_val;

		if (check_pdm_frequencies(drv_cfg, config, pdm_cfg, ratio, &best_diff, &best_rate,
					  &best_freq)) {
			config->ratio = ratios[r].ratio_enum;

			/* Look no further if a configuration giving the exact
			 * PCM rate is found.
			 */
			if (best_diff == 0) {
				break;
			}
		}
	}
#else
	uint8_t ratio = 64;

	(void)check_pdm_frequencies(drv_cfg, config, pdm_cfg, ratio, &best_diff, &best_rate,
				    &best_freq);
#endif

	if (best_diff == UINT32_MAX) {
		return false;
	}

	LOG_INF("PDM clock frequency: %u, actual PCM rate: %u", best_freq, best_rate);
	return true;
}

static int t5838_configure(const struct device *dev, struct dmic_cfg *config)
{
	struct t5838_drv_data *drv_data = dev->data;
	const struct t5838_drv_cfg *drv_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
	struct pcm_stream_cfg *stream = &config->streams[0];
	uint32_t def_map, alt_map;
	nrfx_pdm_config_t nrfx_cfg;
	nrfx_err_t err;

	prv_t5838_configure_gpios(dev);

	if (drv_data->active) {
		LOG_ERR("Cannot configure device while it is active");
		return -EBUSY;
	}

	/*
	 * This device supports only one stream and can be configured to return
	 * 16-bit samples for two channels (Left+Right samples) or one channel
	 * (only Left samples). Left and Right samples can be optionally swapped
	 * by changing the PDM_CLK edge on which the sampling is done
	 * Provide the valid channel maps for both the above configurations
	 * (to inform the requester what is available) and check if what is
	 * requested can be actually configured.
	 */
	if (channel->req_num_chan == 1) {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT);

		channel->act_num_chan = 1;
	} else {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
			  dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT) |
			  dmic_build_channel_map(1, 0, PDM_CHAN_LEFT);

		channel->act_num_chan = 2;
	}

	channel->act_num_streams = 1;
	channel->act_chan_map_hi = 0;
	channel->act_chan_map_lo = def_map;

	if (channel->req_num_streams != 1 || channel->req_num_chan > 2 ||
	    channel->req_num_chan < 1 ||
	    (channel->req_chan_map_lo != def_map && channel->req_chan_map_lo != alt_map) ||
	    channel->req_chan_map_hi != channel->act_chan_map_hi) {
		LOG_ERR("Requested configuration is not supported");
		return -EINVAL;
	}

	/* If either rate or width is 0, the stream is to be disabled. */
	if (stream->pcm_rate == 0 || stream->pcm_width == 0) {
		if (drv_data->configured) {
			nrfx_pdm_uninit();
			drv_data->configured = false;
		}

		return 0;
	}

	if (stream->pcm_width != 16) {
		LOG_ERR("Only 16-bit samples are supported");
		return -EINVAL;
	}

	nrfx_cfg = drv_cfg->nrfx_def_cfg;
	nrfx_cfg.mode = channel->req_num_chan == 1 ? NRF_PDM_MODE_MONO : NRF_PDM_MODE_STEREO;
	nrfx_cfg.edge = channel->req_chan_map_lo == def_map ? NRF_PDM_EDGE_LEFTFALLING
							    : NRF_PDM_EDGE_LEFTRISING;
#if NRF_PDM_HAS_MCLKCONFIG
	nrfx_cfg.mclksrc =
		drv_cfg->clk_src == ACLK ? NRF_PDM_MCLKSRC_ACLK : NRF_PDM_MCLKSRC_PCLK32M;
#endif
	if (!find_suitable_clock(drv_cfg, &nrfx_cfg, config)) {
		LOG_ERR("Cannot find suitable PDM clock configuration.");
		return -EINVAL;
	}

	if (drv_data->configured) {
		nrfx_pdm_uninit();
		drv_data->configured = false;
	}

	err = nrfx_pdm_init(&nrfx_cfg, drv_cfg->event_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize PDM: 0x%08x", err);
		return -EIO;
	}

	drv_data->block_size = stream->block_size;
	drv_data->mem_slab = stream->mem_slab;

	/* Unless the PCLK32M source is used with the HFINT oscillator
	 * (which is always available without any additional actions),
	 * it is required to request the proper clock to be running
	 * before starting the transfer itself.
	 */
	drv_data->request_clock = (drv_cfg->clk_src != PCLK32M);
	drv_data->configured = true;
	return 0;
}

static int start_transfer(struct t5838_drv_data *drv_data)
{
	nrfx_err_t err;
	int ret;

	err = nrfx_pdm_start();
	if (err == NRFX_SUCCESS) {
		return 0;
	}

	LOG_ERR("Failed to start PDM: 0x%08x", err);
	ret = -EIO;

	if (drv_data->request_clock) {
		(void)onoff_release(drv_data->clk_mgr);
	}

	drv_data->active = false;
	return ret;
}

static void clock_started_callback(struct onoff_manager *mgr, struct onoff_client *cli,
				   uint32_t state, int res)
{
	struct t5838_drv_data *drv_data = CONTAINER_OF(cli, struct t5838_drv_data, clk_cli);

	/* The driver can turn out to be inactive at this point if the STOP
	 * command was triggered before the clock has started. Do not start
	 * the actual transfer in such case.
	 */
	if (!drv_data->active) {
		(void)onoff_release(drv_data->clk_mgr);
	} else {
		(void)start_transfer(drv_data);
	}
}

static int trigger_start(const struct device *dev)
{
	struct t5838_drv_data *drv_data = dev->data;
	int ret;

	drv_data->active = true;

	/* If it is required to use certain HF clock, request it to be running
	 * first. If not, start the transfer directly.
	 */
	if (drv_data->request_clock) {
		sys_notify_init_callback(&drv_data->clk_cli.notify, clock_started_callback);
		ret = onoff_request(drv_data->clk_mgr, &drv_data->clk_cli);
		if (ret < 0) {
			drv_data->active = false;

			LOG_ERR("Failed to request clock: %d", ret);
			return -EIO;
		}
	} else {
		ret = start_transfer(drv_data);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int t5838_trigger(const struct device *dev, enum dmic_trigger cmd)
{
	struct t5838_drv_data *drv_data = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		if (drv_data->active) {
			drv_data->stopping = true;
			nrfx_pdm_stop();
		}
		if (drv_data->aad_enabled_mode != T5838_AAD_SELECT_NONE) {
			prv_t5838_enter_sleep_with_AAD(dev);
		}
		break;

	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		if (!drv_data->configured) {
			LOG_ERR("Device is not configured");
			return -EIO;
		} else if (!drv_data->active) {
			drv_data->stopping = false;
			return trigger_start(dev);
		}
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int t5838_pdm_read(const struct device *dev, uint8_t stream, void **buffer, size_t *size,
			  int32_t timeout)
{
	struct t5838_drv_data *drv_data = dev->data;
	int ret;

	ARG_UNUSED(stream);

	if (!drv_data->configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	ret = k_msgq_get(&drv_data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if (ret != 0) {
		LOG_ERR("No audio data to be read");
	} else {
		LOG_DBG("Released buffer %p", *buffer);

		*size = drv_data->block_size;
	}

	return ret;
}

static void init_clock_manager(const struct device *dev)
{
	struct t5838_drv_data *drv_data = dev->data;
	clock_control_subsys_t subsys;

#if NRF_CLOCK_HAS_HFCLKAUDIO
	const struct t5838_drv_cfg *drv_cfg = dev->config;

	if (drv_cfg->clk_src == ACLK) {
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HFAUDIO;
	} else
#endif
	{
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HF;
	}

	drv_data->clk_mgr = z_nrf_clock_control_get_onoff(subsys);
	__ASSERT_NO_MSG(drv_data->clk_mgr != NULL);
}

static const struct _dmic_ops t5838_dmic_ops = {
	.configure = t5838_configure,
	.trigger = t5838_trigger,
	.read = t5838_pdm_read,
};

#define T5838_NODE_ID(idx)     DT_NODELABEL(t5838)
#define T5838_PDM(idx)	       DT_NODELABEL(pdm##idx)
#define T5838_PDM_CLK_SRC(idx) DT_STRING_TOKEN(T5838_PDM(idx), clock_source)

#define T5838_HAS_THSEL_GPIO(idx) DT_NODE_HAS_PROP(DT_DRV_INST(idx), thsel_gpios)
#define T5838_THSEL_GPIO_SPEC(idx)                                                                 \
	IF_ENABLED(T5838_HAS_THSEL_GPIO(idx),                                                      \
		   (static const struct gpio_dt_spec thsel_##idx =                                 \
			    GPIO_DT_SPEC_GET(DT_DRV_INST(idx), thsel_gpios);))

#define T5838_HAS_MICEN_GPIO(idx) DT_NODE_HAS_PROP(DT_DRV_INST(idx), micen_gpios)
#define T5838_MICEN_GPIO_SPEC(idx)                                                                 \
	IF_ENABLED(T5838_HAS_MICEN_GPIO(idx),                                                      \
		   (static const struct gpio_dt_spec micen_##idx =                                 \
			    GPIO_DT_SPEC_GET(DT_DRV_INST(idx), micen_gpios);))

#define T5838_HAS_WAKE_GPIO(idx) DT_NODE_HAS_PROP(DT_DRV_INST(idx), wake_gpios)
#define T5838_WAKE_GPIO_SPEC(idx)                                                                  \
	IF_ENABLED(T5838_HAS_WAKE_GPIO(idx),                                                       \
		   (static const struct gpio_dt_spec wake_##idx =                                  \
			    GPIO_DT_SPEC_GET(DT_DRV_INST(idx), wake_gpios);))

#define T5838_HAS_PDMCLK_GPIO(idx) DT_NODE_HAS_PROP(DT_DRV_INST(idx), pdmclk_gpios)
#define T5838_PDMCLK_GPIO_SPEC(idx)                                                                \
	IF_ENABLED(T5838_HAS_PDMCLK_GPIO(idx),                                                     \
		   (static const struct gpio_dt_spec pdmclk_##idx =                                \
			    GPIO_DT_SPEC_GET(DT_DRV_INST(idx), pdmclk_gpios);))

#define T5838_PDM_NRFX_DEVICE(idx)                                                                 \
	static void *t5838_rx_msgs##idx[DT_PROP(T5838_PDM(idx), queue_size)];                      \
	static struct t5838_drv_data t5838_data##idx;                                              \
	T5838_THSEL_GPIO_SPEC(idx)                                                                 \
	T5838_MICEN_GPIO_SPEC(idx)                                                                 \
	T5838_WAKE_GPIO_SPEC(idx)                                                                  \
	T5838_PDMCLK_GPIO_SPEC(idx)                                                                \
	static int t5838_pdm_nrfx_init##idx(const struct device *dev)                              \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(T5838_PDM(idx)), DT_IRQ(T5838_PDM(idx), priority), nrfx_isr,   \
			    nrfx_pdm_irq_handler, 0);                                              \
		const struct t5838_drv_cfg *drv_cfg = dev->config;                                 \
		int err = pinctrl_apply_state(drv_cfg->pcfg, PINCTRL_STATE_DEFAULT);               \
		if (err < 0) {                                                                     \
			return err;                                                                \
		}                                                                                  \
		k_msgq_init(&t5838_data##idx.rx_queue, (char *)t5838_rx_msgs##idx, sizeof(void *), \
			    ARRAY_SIZE(t5838_rx_msgs##idx));                                       \
		init_clock_manager(dev);                                                           \
		return 0;                                                                          \
	}                                                                                          \
	static void event_handler##idx(const nrfx_pdm_evt_t *evt)                                  \
	{                                                                                          \
		event_handler(DEVICE_DT_GET(T5838_NODE_ID(idx)), evt);                             \
	}                                                                                          \
	PINCTRL_DT_DEFINE(T5838_PDM(idx));                                                         \
	static const struct t5838_drv_cfg t5838_cfg##idx = {                                       \
		.event_handler = event_handler##idx,                                               \
		.nrfx_def_cfg = NRFX_PDM_DEFAULT_CONFIG(0, 0),                                     \
		.nrfx_def_cfg.skip_gpio_cfg = true,                                                \
		.nrfx_def_cfg.skip_psel_cfg = true,                                                \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(T5838_PDM(idx)),                                 \
		.clk_src = T5838_PDM_CLK_SRC(idx),                                                 \
		IF_ENABLED(T5838_HAS_MICEN_GPIO(idx), (.micen = &micen_##idx, ))                   \
			IF_ENABLED(T5838_HAS_THSEL_GPIO(idx), (.thsel = &thsel_##idx, ))           \
				IF_ENABLED(T5838_HAS_WAKE_GPIO(idx), (.wake = &wake_##idx, ))      \
					IF_ENABLED(T5838_HAS_PDMCLK_GPIO(idx),                     \
						   (.pdmclk = &pdmclk_##idx, ))};                  \
	BUILD_ASSERT(T5838_PDM_CLK_SRC(idx) != ACLK || NRF_PDM_HAS_MCLKCONFIG,                     \
		     "Clock source ACLK is not available.");                                       \
	BUILD_ASSERT(T5838_PDM_CLK_SRC(idx) != ACLK ||                                             \
			     DT_NODE_HAS_PROP(DT_NODELABEL(clock), hfclkaudio_frequency),          \
		     "Clock source ACLK requires the hfclkaudio-frequency "                        \
		     "property to be defined in the nordic,nrf-clock node.");                      \
	DEVICE_DT_DEFINE(T5838_NODE_ID(idx), t5838_pdm_nrfx_init##idx, NULL, &t5838_data##idx,     \
			 &t5838_cfg##idx, POST_KERNEL, CONFIG_T5838_INIT_PRIORITY,                 \
			 &t5838_dmic_ops);

/* Existing SoCs only have one PDM instance - that is why we support only one instance of mic */
T5838_PDM_NRFX_DEVICE(0);
