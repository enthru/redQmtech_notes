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
Pins assignment can be found in this file - cfg/ports.xdc
<br /><br />
**Some links:**
<br />
<br />
Some history of the project - https://enthru.net/?tag=qmtech
<br />
EasyEda project -  https://oshwlab.com/neuroworker/ltc2208_lvds
<br /><br />

## Protocol 2

`projects/sdr_transceiver_hpsdr_61_44` speaks the openHPSDR **protocol 2**
(UDP 1024-1029 for the register blocks, 1035+ for DDC I/Q), not Metis.  A
Metis-only client will not discover it.

What the port needed, and what it costs:

* **TX I/Q is 192 kHz** in protocol 2 instead of 48 kHz, so the DUC chain in
  `tx.tcl` moved with it: the interpolating FIR now runs at 0.192 MHz and the
  CIC interpolates by 320 from 0.384 MHz.  Coefficients are unchanged - the
  CIC compensation is insensitive to the rate over this range.
* **DDC rates now reach 1536 kHz.**  The RX CIC decimation range in `rx.tcl`
  is 20 to 640 (1536 kHz down to 48 kHz) and the FIR after it runs at up to
  3.072 MHz, which costs roughly four times the DSP slices it used to.
* **RX samples reach the CPU by DMA**, not through the AXI hub FIFO.  Five
  channels at 1536 kHz are 49 MB/s, well past what single word reads over
  M_AXI_GP sustain, so `rx.tcl` ends in an `axis_ram_writer` on `S_AXI_ACP`
  writing a 4 MiB ring in DDR allocated through `/dev/cma`, exactly as
  `sdr_receiver_wide` does.  `cfg` grew to 384 bits to carry the ring's base
  address and wrap count, and the first status word is now the writer's burst
  index rather than a FIFO fill level.
* **Three DDCs are advertised**, one per LO.  They all run at the rate the
  host asks for on the lowest numbered enabled DDC, because the gateware has
  a single decimation register.
* The board answers discovery as a **Hermes**.
* Not implemented: the wideband ADC stream, PureSignal (the two feedback
  channels locked to the TX LO are still in the gateware but protocol 2 has
  no way to reach them yet), and host-chosen UDP port numbers.
