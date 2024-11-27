# redQmtech_notes

This is an adaptation of red-pitaya hpsdr transceiver code by Pavel Demin for QMTech 7z020 board. It's in fact original code with some disabled features/peripherals and changed pinout.
<br />
Main branch is using 122.88MHz single ended clock input.
<br />
At this moment I'm using it with DAC904E DAC and AD6645 ADC. Replacing AD6645 to 2xLTC2208 is in plans for future, so I'm keeping 2x16 bit ADC pinout.
<br /><br />

**Note:**
With my LTC2208 board you can use bitstream created from oneADC branch (see releases and commentaries in the ports.xdc file in oneADC branch).
Or just solder clock pin from adc board to FPGA board by wire.

**Board pins:**
![alt text](https://enthru.net/wp-content/uploads/2024/09/qmtech_pins.jpg)
<br />
Pins assignment can be found in this file - cfg/ports.xdc
<br /><br />
**Some links:**
<br />
<br />
EasyEDA project of the example ADC board - https://oshwlab.com/neuroworker/redqmtechadc
<br />
Some history of the project - https://enthru.net/?tag=qmtech
<br />
Github project with the gerbers and some pics - https://github.com/enthru/redQmtech_notes_adc
