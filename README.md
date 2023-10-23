# Irnas's T5838 PDM microphone driver

This repository contains a driver for the T5838 PDM microphone. It is based on, and replaces the Zephyr PDM driver for nrf, but it has been modified to support the T5838 microphone.

Reason we are replacing the driver is that it doesn't allow us to set PDM frequency that is required by the microphone (see datasheet) and we need to use PDM CLK pin as a GPIO for setting AAD modes.

T5838 datasheet for more information can be found [here](https://invensense.tdk.com/wp-content/uploads/2023/06/DS-000383-T5838-Datasheet-v1.1.pdf).

## Setup

1. To your `west.yml` add the IRNAS remote to the `remotes` section:

   ```yaml
   - name: irnas
     url-base: https://github.com/irnas
   ```

2. Then in the `projects` section add at the bottom:

   ```yaml
   - name: irnas-t5838-driver
      repo-path: irnas-t5838-driver
      path: irnas/irnas-t5838-driver
      remote: irnas
      revision: feature/t5838-driver
   ```

3. Add the flash DTS entry to your board definition or overlay file. For example:

   ```dts
   dmic_dev: &pdm0 {
      status = "okay";
      pinctrl-0 = <&pdm0_default>;
      pinctrl-1 = <&pdm0_sleep>;
      pinctrl-names = "default","sleep";
      #address-cells = < 0x1 >;
      #size-cells = < 0x0 >;
      t5838: t5838@0 {
         status = "okay";
         compatible = "tdk,t5838-nrf-pdm";
         micen-gpios = <&gpio1 11 GPIO_ACTIVE_HIGH>;
         thsel-gpios = <&gpio1 4 GPIO_ACTIVE_HIGH>;
         wake-gpios = <&gpio1 10 GPIO_ACTIVE_HIGH>;
         pdmclk-gpios = <&gpio1 13 GPIO_ACTIVE_HIGH>;
         reg = <0x0>;
      };
   };
   ```

   Note that since we are using dts entry pdm0 as a bus, we need to add the following to its definition:

   ```dts
   #address-cells = < 0x1 >;
   #size-cells = < 0x0 >;
   ```

   This tells the kernel that the bus has 1 address cell and 0 size cells. So we don't have to set reg value to full address of the device, but only the address of the device on the bus. Note that reg property is not currently used, as we currently only support one device on the bus and microphone proprietary protocol (I refer to it as fake2c communication) address is hardcoded in the driver header file (0x53 which is the only address microphone supports).
   It is planed reg value will be used in the future to support two microphones on the same bus (stereo configuration).

   Also note it is necessary to define all the gpios used by the microphone. The driver will not work without them. Clock pin must be defined in both pdm0 and t5838 node. Other pins are only defined in t5838 node.

4. Add the following to your `prj.conf` or whatever kconfig file you are using:

   ```conf
   CONFIG_AUDIO=y
   CONFIG_AUDIO_DMIC=y
   CONFIG_AUDIO_DMIC_NRFX_PDM=n
   ```

   This will enable audio subsystem, dmic driver and disable nrfx pdm driver. We must disable nrfx default pdm driver, because it is not compatible with our driver. If you don't disable it, you will get a compile error.

5. Don't forget to run `east clean` and `east update` before trying to build the project you are working on.

## Usage

The driver is used in the same way zephyr pdm driver is used. The only difference is that you need to use your microphones node name instead of `pdm0`. For example:

There are added functions that handle microphones AAD (acoustic activity detection) feature. API was designed to be as similar as possible to the API from vm3011 driver being used in other projects (since t5838 is to replace vm3011 in those projects).

## AAD API

To configure AAD we first create configuration struct and fill it with desired values. Then we call `t5838_configure_AAD` function with pointer to our configuration struct as an argument.

T5838_thconf struct is defined as follows:

```c
struct T5838_thconf {
   /* enum used in all AAD modes */
   enum T5838_aad_select aad_select;

   /* enums used in AAD A mode */
   enum T5838_aad_a_lpf aad_a_lpf;
   enum T5838_aad_a_thr aad_a_thr;

   /* enums used in AAD D modes */
   enum T5838_aad_d_floor aad_d_floor;
   enum T5838_aad_d_rel_pulse_min aad_d_rel_pulse_min;
   enum T5838_aad_d_abs_pulse_min aad_d_abs_pulse_min;
   enum T5838_aad_d_abs_thr aad_d_abs_thr;
   enum T5838_aad_d_rel_thr aad_d_rel_thr;
};
```

aad_select enumerator is used to select AAD mode. It can be set to one of the following values:
| AAD mode              | description                                                                                                                      |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------|
| T5838_AAD_SELECT_NONE | Disable AAD modes. This is lowest power mode but provides us no trigger on sound level                                           |
| T5838_AAD_SELECT_D1   | AAD D1 mode is digital mode with PDM output disabled (triggers with PDM clock off)                                               |
| T5838_AAD_SELECT_D2   | AAD D2 mode is digital mode with PDM output disabled (triggers in low power mode, PDM clock 400-800kHz)                          |
| T5838_AAD_SELECT_A    | AAD A mode is analog mode, microphone digital logic is off (PDM clock off), this is lowest power mode that allows triggering AAD |

### AAD A configuration

To use AAD A mode we need to set aad_select to T5838_AAD_SELECT_A and set aad_a_lpf and aad_a_thr to desired values. aad_a_lpf is used to set low pass filter frequency. It can be set to one of the following values:

|    aad_a_lpf value     |
|------------------------|
| T5838_AAD_A_LPF_4_4kHz |
| T5838_AAD_A_LPF_2_0kHz |
| T5838_AAD_A_LPF_1_9kHz |
| T5838_AAD_A_LPF_1_8kHz |
| T5838_AAD_A_LPF_1_6kHz |
| T5838_AAD_A_LPF_1_3kHz |
| T5838_AAD_A_LPF_1_1kHz |

aad_a_thr is used to set threshold for AAD A mode. It can be set to one of the following values:

|    aad_a_thr value     |
|------------------------|
|  T5838_AAD_A_THR_60dB  |
|  T5838_AAD_A_THR_65dB  |
|  T5838_AAD_A_THR_70dB  |
|  T5838_AAD_A_THR_75dB  |
|  T5838_AAD_A_THR_80dB  |
|  T5838_AAD_A_THR_85dB  |
|  T5838_AAD_A_THR_90dB  |
|  T5838_AAD_A_THR_95dB  |
| T5838_AAD_A_THR_97_5dB |

### AAD D configuration

Both D1 and D2 modes use same configuration registers, only difference in configuration is setting aad_select to T5838_AAD_SELECT_D1 or T5838_AAD_SELECT_D2.

To use AAD D mode we need to set aad_select to T5838_AAD_SELECT_D1 or T5838_AAD_SELECT_D2 and set aad_d_floor, aad_d_rel_pulse_min, aad_d_abs_pulse_min, aad_d_abs_thr and aad_d_rel_thr to desired values.

AAD D modes are not yet tested with this driver, so please refer to t5838.h file to see what values can be set for each of the registers. Please also refer to [t5838 datasheet](https://invensense.tdk.com/wp-content/uploads/2023/06/DS-000383-T5838-Datasheet-v1.1.pdf) for more information on AAD D modes.

### API functions

Configuration function is defined as follows:

```c
int t5838_configure_AAD(const struct device *dev, struct T5838_thconf *thconf);
```

we pass pointer to our configuration struct as an argument. Function will write configuration to microphone registers and put microphone in desired AAD mode. This function must be called after calling `dmic_configure` pdm API function.

Function for setting interrupt handler is defined as follows:

```c
void t5838_wake_set_handler(const struct device *dev, T5838_wake_handler_t handler);
```

This function sets handler to be called when wake interrupt is triggered. Note that interrupt will be called once and then it will be disabled. If you want to use wake interrupt again, you must call `t5838_wake_clear();` function.

Function for clearing interrupt is defined as follows:

```c
int t5838_wake_clear(const struct device *dev);
```

This function clears wake interrupt and reenables it. It is used to reenable wake interrupt after it has been triggered.

Function for getting wake pin state is defined as follows:

```c
int t5838_wake_get(const struct device *dev);
```

This function directly pools wake pin. Note that this is not recommended since T5838 wake pin goes high when AAD threshold is reached and goes low as soon as the level falls bellow it. This can give us very erratic results if we pool it at the wrong time. It is recommended to use interrupt instead.
