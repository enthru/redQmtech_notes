# redQmtech_notes

This is an adaptation of red-pitaya hpsdr receiver/transceiver code by Pavel Demin (https://github.com/pavel-demin/red-pitaya-notes/) for QMTech 7z020 board. It's in fact original code with some disabled features/peripherals and changed pinout.
<br />
<br />
**This is for experiments only! With this hardware you may be unable to achieve datasheet parameters for LTC2208. Main branch is for 61.44MHz clock**
<br />
<br />
**Board pins:**
![alt text](https://enthru.net/wp-content/uploads/2024/09/qmtech_pins.jpg)
<br />
![alt text](https://enthru.net/wp-content/uploads/2026/11/cpinout.png)
<br />
Pins assignment can be found in this file - cfg/ports.xdc
<br /><br />
**Boot image (boot.bin):**
<br />
The ADS1115 on JP5 pins 5/6 is read through PS I2C0 over EMIO, so Linux needs `i2c0` enabled in the device tree, which a bitstream loaded later cannot add. Copy `dts/common.dts` over the same file in https://github.com/pavel-demin/qmtech-xc7z020-notes and rebuild boot.bin there:
```
source /opt/AMD/2025.2/Vivado/settings64.sh
source /opt/AMD/2025.2/Vitis/settings64.sh
make NAME=led_blinker_77_76 XSCT="xsct -nodisp" boot.bin
```
`-nodisp` keeps xsct 2025.2 from requiring xlsclients/Xvfb; it also wants `gmake` in PATH (on distributions without it: `ln -s /usr/bin/make ~/bin/gmake`).
On a host with OpenSSL 4 (no ENGINE API) the kernel's `certs/extract-cert` fails to link; after the kernel is unpacked into `tmp/linux-6.6`, apply `patches/extract-cert-openssl4.patch` there with `patch -d tmp -p0` and rerun make.
<br /><br />
**Some links:**
<br />
<br />
Some history of the project - https://enthru.net/?tag=qmtech
<br />
EasyEda project -  https://oshwlab.com/neuroworker/ltc2208_lvds
