/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <t5838.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel_version.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_sample);

#define MAX_SAMPLE_RATE	 16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT	 1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels)                                              \
	(BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT    6
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 6);

K_SEM_DEFINE(mic_sem, 0, 1);
void t5838_cb(const struct device *dev)
{
	LOG_INF("T5838 interrupt triggered");
	k_sem_give(&mic_sem);
}

int main(void)
{
	const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(pdm0));
	const struct device *const dmic_aad_dev = DEVICE_DT_GET(DT_NODELABEL(t5838));
	int ret;

	LOG_INF("DMIC triggered sample");
	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io =
			{
				/* These fields can be used to limit the PDM clock
				 * configurations that the driver is allowed to use
				 * to those supported by the microphone.
				 */
				.min_pdm_clk_freq = 1000000,
				.max_pdm_clk_freq = 3500000,
				.min_pdm_clk_dc = 40,
				.max_pdm_clk_dc = 60,
			},
		.streams = &stream,
		.channel =
			{
				.req_num_streams = 1,
			},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	LOG_INF("PCM output rate: %u, channels: %u", cfg.streams[0].pcm_rate,
		cfg.channel.req_num_chan);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}
	/*AAD A CONFIGURATION */
	struct t5838_aad_a_conf aadcfg = {
		.aad_a_lpf = T5838_AAD_A_LPF_2_0kHz,
		.aad_a_thr = T5838_AAD_A_THR_85dB,
	};
	t5838_aad_a_mode_set(dmic_aad_dev, &aadcfg);
	t5838_aad_wake_handler_set(dmic_aad_dev, t5838_cb);
	while (1) {

		LOG_INF("Waiting on dmic trigger");
		k_sem_take(&mic_sem, K_FOREVER);
		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
		if (ret < 0) {
			LOG_ERR("START trigger failed: %d", ret);
			return ret;
		}

		for (int i = 0; i < 2; ++i) {
			void *buffer;
			uint32_t size;
			int ret;

			ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
			if (ret < 0) {
				LOG_ERR("%d - read failed: %d", i, ret);
				return ret;
			}

			LOG_INF("%d - got buffer %p of %u bytes", i, buffer, size);

			k_mem_slab_free(&mem_slab, &buffer);
		}

		ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
		if (ret < 0) {
			LOG_ERR("STOP trigger failed: %d", ret);
			return ret;
		}
		t5838_aad_wake_clear(dmic_aad_dev);
	}

	LOG_INF("Exiting");
	return 0;
}
