# redQmtech_notes

This is an adaptation of red-pitaya hpsdr receiver/transceiver code by Pavel Demin (https://github.com/pavel-demin/red-pitaya-notes/) for QMTech 7z020 board. It's in fact original code with some disabled features/peripherals and changed pinout.
<br />

**Branching:**

main - one ADC (technicaly not - but pinout is a mess for two adc) non-dedicated pin for input clock, pinout for board from my github
2adc - the same ad in main, but pinout for easyeda project board with two ADCs
ad9226_61M - pinout for ad9226 module, 61.44M main enerator, DAC clocking via PLL - 122.88M

**Board pins:**
![alt text](https://enthru.net/wp-content/uploads/2024/09/qmtech_pins.jpg)
<br />
Pins assignment can be found in this file - cfg/ports.xdc
<br /><br />
**Some links:**
<br />
<br />
EasyEDA project of the example ADC board with two LTC2208 - https://oshwlab.com/neuroworker/redqmtechadc
<br />
Some history of the project - https://enthru.net/?tag=qmtech
<br />
Github project with the gerbers and some pics for single ADC module - https://github.com/enthru/redQmtech_notes_adc
