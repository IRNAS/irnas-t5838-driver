# DMIC-triggered sample

This sample is almost direct copy of zephyr `samples/dmic` sample. The only difference is that it uses t5838 driver instead of zephyr pdm driver and that sampling starts after receiving trigger signal from microphone.

Refer to original sample documentation for more information. See modified dts overlay for added child node configuration and modified `prj.conf` for added `CONFIG_AUDIO_DMIC_NRFX_PDM=n` kconfig needed to use our driver, and `CONFIG_T5838_AAD_TRIGGER=y` to use interrupt based trigger functionality.
