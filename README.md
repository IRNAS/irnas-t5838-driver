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

2. Then in the `projects` section add at the bottom (select the revision you want to use):

   ```yaml
   - name: irnas-t5838-driver
      repo-path: irnas-t5838-driver
      path: irnas/irnas-t5838-driver
      remote: irnas
      revision: <revision>
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
         compatible = "invensense,t5838-nrf-pdm";
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

The driver is used in the same way zephyr pdm driver is used.

There are added functions that handle microphones AAD (acoustic activity detection) feature. API was designed to be as similar as possible to the API from vm3011 driver being used in other projects (since t5838 is to replace vm3011 in those projects).
