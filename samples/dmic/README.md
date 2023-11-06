# DMIC sample

This sample is almost direct copy of zephyr `samples/dmic` sample. The only difference is that it uses t5838 driver instead of zephyr pdm driver.

Refer to original sample documentation for more information. See modified dts overlay for added child node configuration and modified `prj.conf` for added `CONFIG_AUDIO_DMIC_NRFX_PDM=n` kconfig needed to use our driver.
