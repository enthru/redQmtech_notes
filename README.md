# redQmtech_notes

This is an adaptation of red-pitaya hpsdr transceiver for QMTech 7z020 board. It's in fact original code with some disabled features/peripherals and changed pinout.
<br />
Main branch is using 122.88MHz single ended clock input.
<br />
At this moment I'm using it with DAC904E DAC and AD6645 ADC. Replacing AD6645 to 2xLTC2208 is in plans for future, so I'm keeping 2x16 bit ADC pinout.
<br /><br />

**Board pins:**
![alt text](https://enthru.net/wp-content/uploads/2024/09/qmtech_pins.jpg)
<br />
Pins assignment can be found in this file - cfg/ports.xdc
<br /><br />
**TODO:**
<br />
- Enable i2c using EMIO
