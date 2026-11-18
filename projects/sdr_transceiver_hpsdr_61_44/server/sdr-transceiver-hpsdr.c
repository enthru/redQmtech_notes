/*
openHPSDR Protocol 2 server.

Derived from Pavel Demin's protocol 1 server for the same gateware.  The I2C
peripheral support (PENELOPE, ALEX, ATU, DS1803, MCP4725, G8NJJ Arduino,
NUCLEO, IC-735 CAT) and the CW keyer are carried over unchanged; the network
side is new.

Differences from the protocol 1 server that follow from the hardware:

  * The DDCs share one CIC decimation register, so every DDC runs at the rate
    the host asked for on the lowest numbered enabled DDC.  A host that asks
    for different rates on different DDCs gets that one rate on all of them.
  * Two DDCs are advertised, one per LO in the gateware.  The board this runs
    on has a single ADC, so neither diversity nor PureSignal can work here and
    the channels those used to occupy were given back to the sample rate: two
    DDCs fit under a CIC decimation of 5, which is what puts 6144 kHz within
    reach.  See the RATE_MAX comment below.
  * RX samples reach the CPU by DMA into a ring buffer in DDR (S_AXI_ACP),
    not through the AXI hub FIFO: at 6144 kHz the two channels produce
    74 MB/s, which is well past what single word reads over M_AXI_GP sustain.
  * The wideband ADC stream (general register bytes 23..28) is not implemented.
*/

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <termios.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#define I2C_SLAVE       0x0703 /* Use this slave address */
#define I2C_SLAVE_FORCE 0x0706 /* Use this slave address, even if it
                                  is already in use by a driver! */

#define CMA_ALLOC _IOWR('Z', 0, uint32_t)

#define ADDR_PENE 0x20 /* PCA9555 address 0 */
#define ADDR_ALEX 0x21 /* PCA9555 address 1 */
#define ADDR_LEVEL 0x22 /* PCA9555 address 2 */
#define ADDR_MISC 0x23 /* PCA9555 address 3 */
#define ADDR_DRIVE 0x28 /* DS1803 address 0 */
#define ADDR_CODEC 0x1A /* WM8731 or TLV320AIC23B address 0 */
#define ADDR_DAC0 0x60 /* MCP4725 address 0 */
#define ADDR_DAC1 0x61 /* MCP4725 address 1 */
#define ADDR_ARDUINO 0x40 /* G8NJJ Arduino sketch */
#define ADDR_NUCLEO 0x55 /* NUCLEO-G071RB */
#define ADDR_ADS1115 0x48 /* ADS1115 with ADDR to GND */

/* ports the host sends to, and that we answer from */
#define PORT_GENERAL  1024 /* general registers in, command response out */
#define PORT_RXSPEC   1025 /* DDC specific in, high priority status out */
#define PORT_TXSPEC   1026 /* DUC specific in, mic/line samples out */
#define PORT_HIGHPRIO 1027 /* high priority in, wideband samples out */
#define PORT_AUDIO    1028 /* RX audio from host */
#define PORT_TXIQ     1029 /* TX I/Q from host */
#define PORT_IQ_BASE  1035 /* DDC n I/Q out on PORT_IQ_BASE + n */

#define DDC_COUNT 2 /* one per LO in the gateware */

#define IQ_SAMPLES 238
#define IQ_PKT_BYTES (16 + IQ_SAMPLES * 6)
#define MIC_SAMPLES 64
#define MIC_PKT_BYTES (4 + MIC_SAMPLES * 2)
#define AUDIO_SAMPLES 64
#define AUDIO_PKT_BYTES (4 + AUDIO_SAMPLES * 4)
#define TXIQ_SAMPLES 240
#define TXIQ_PKT_BYTES (4 + TXIQ_SAMPLES * 6)
#define REGISTER_PKT_BYTES 1444

#define BOARD_HERMES 1
#define GATEWARE_VERSION 39

/* DDC sample rates the gateware can produce: 61.44e6 / (2 * cic_rate).

   The ceiling is set by the one wire between the CICs and the FIR, not by the
   ADC.  The CICs run in parallel but the FIR is a single time multiplexed
   core, so the four streams (two DDCs, I and Q each) are serialised onto one
   32 bit bus at 61.44 MHz: 4 * 61.44 / cic <= 61.44, that is cic >= 4.  The
   rate ladder needs cic = 5 for 6144 kHz, which fits with 20 percent to
   spare; 12288 kHz would need cic = 2.5 and simply does not exist at this
   clock.  So 6144 kHz is the top of the ladder for any channel count.

   This is also why there are two DDCs and not three: six streams would force
   cic >= 6, and 6144 kHz would be out of reach again. */
#define RATE_MIN 48000
#define RATE_MAX 6144000

/* DMA ring: 32768 bursts of 128 bytes = 4 MiB, eight 16 byte sample sets per
   burst.  Both counts are powers of two so the wrap is a mask.  A sample set
   is two channels of 24 bit I and Q, 12 bytes, padded to 16 so it divides the
   burst and the writer's 64 bit beat. */
#define RX_BURSTS 32768
#define RX_RAM_SIZE (RX_BURSTS * 128)
#define RX_SET_BYTES 16
#define RX_SETS (RX_BURSTS * (128 / RX_SET_BYTES))

/* TX I/Q FIFO holds 4096 32 bit words, two per sample: 5.3 ms at 192 kHz.
   Kept between a quarter and three quarters full to ride out host jitter
   without letting the transmit latency grow. */
#define TX_FIFO_LOW 1024
#define TX_FIFO_HIGH 3072
#define DAC_FIFO_LOW 512
#define DAC_FIFO_HIGH 1536

volatile uint32_t *rx_freq, *tx_freq, *alex, *dac_freq, *rx_ram;
volatile uint16_t *rx_rate, *rx_wrap, *rx_cntr, *tx_cntr, *dac_cntr, *adc_cntr;
volatile int16_t *tx_level, *dac_level;
volatile uint8_t *gpio_in, *gpio_out, *rx_rst, *tx_rst, *lo_rst;
volatile uint32_t *fifo, *codec, *xadc;
uint8_t *ram;

const uint32_t freq_min = 0;
const uint32_t freq_max = 30720000;

/* variables to handle I2C devices */
int i2c_fd;
int i2c_pene = 0;
int i2c_alex = 0;
int i2c_level = 0;
int i2c_misc = 0;
int i2c_drive = 0;
int i2c_codec = 0;
int i2c_dac0 = 0;
int i2c_dac1 = 0;
int i2c_arduino = 0;
int i2c_nucleo = 0;

uint16_t i2c_pene_data = 0;
uint16_t i2c_alex_data = 0;
uint16_t i2c_level_data = 0;
uint16_t i2c_misc_data = 0;
uint16_t i2c_drive_data = 0;
uint16_t i2c_dac0_data = 0xfff;
uint16_t i2c_dac1_data = 0xfff;
uint32_t i2c_nucleo_data[2] = {0, 0};

uint16_t i2c_ard_frx1_data = 0; /* rx 1 freq in kHz */
uint16_t i2c_ard_frx2_data = 0; /* rx 2 freq in kHz */
uint16_t i2c_ard_ftx_data = 0; /* tx freq in kHz */
uint32_t i2c_ard_ocant_data = 0; /* oc output and ant */
uint32_t i2c_ard_txatt_data = 0; /* tx attenuation and oddments  */
uint16_t i2c_ard_rxatt_data = 0; /* rx attenuation */

uint8_t log_table_lookup[256]; /* lookup table from linear scale to
                                  6 bit / 0.5 dB attenuation */

uint8_t i2c_boost_data = 0;

/* The ADS1115 has a descriptor of its own: the slave address belongs to the
   descriptor, and the other one is retargeted from several threads. */
int ads_fd = -1;
volatile uint16_t ads_data[3] = {0, 0, 0}; /* forward, reverse, supply */

uint8_t dac_level_data = 0;

uint8_t cw_int_data = 0;
uint8_t rx_att_data = 0;
uint8_t tx_mux_data = 0;

uint16_t cw_hang = 0;
uint8_t cw_reversed = 0;
uint8_t cw_speed = 25;
uint8_t cw_mode = 0;
uint8_t cw_weight = 50;
uint8_t cw_spacing = 0;
uint8_t cw_delay = 0;
uint8_t cw_ptt = 0;

int cw_memory[2] = {0, 0};
int cw_ptt_delay = 0;

void *handler_keyer(void *arg);
ssize_t i2c_write_addr_data8(int fd, uint8_t addr, uint8_t data)
{
  uint8_t buffer[2];
  buffer[0] = addr;
  buffer[1] = data;
  return write(fd, buffer, 2);
}

ssize_t i2c_write_addr_data16(int fd, uint8_t addr, uint16_t data)
{
  uint8_t buffer[3];
  buffer[0] = addr;
  buffer[1] = data;
  buffer[2] = data >> 8;
  return write(fd, buffer, 3);
}

ssize_t i2c_write_addr_data24(int fd, uint8_t addr, uint32_t data)
{
  uint8_t buffer[4];
  buffer[0] = addr;
  buffer[1] = data;
  buffer[2] = data >> 8;
  buffer[3] = data >> 16;
  return write(fd, buffer, 4);
}

ssize_t i2c_write_data16(int fd, uint16_t data)
{
  uint8_t buffer[2];
  buffer[0] = data >> 8;
  buffer[1] = data;
  return write(fd, buffer, 2);
}

ssize_t i2c_write_data40(int fd, uint32_t low, uint32_t high)
{
  uint32_t buffer[2];
  buffer[0] = low;
  buffer[1] = high;
  return write(fd, buffer, 5);
}

uint16_t alex_data_rx = 0;
uint16_t alex_data_tx = 0;
uint16_t alex_data_0 = 0;
uint16_t alex_data_1 = 0;
uint16_t alex_update = 0;

uint32_t freq_data[3] = {0, 0, 0};

/* calculate lookup table from drive scale value to 0.5 dB attenuation units */
void calc_log_lookup()
{
  int index;
  float value;
  uint8_t att;

  log_table_lookup[0] = 63; /* max att if no drive */
  for(index = 1; index < 256; ++index)
  {
    value = -40.0 * log10((float)index / 255.0);
    if(value > 63.0)
    {
      att = 63;
    }
    else
    {
      att = (uint8_t)value;
    }
    log_table_lookup[index] = att;
  }
}

void alex_write()
{
  uint32_t max = freq_data[1] > freq_data[2] ? freq_data[1] : freq_data[2];
  uint16_t manual = (alex_data_1 >> 15) & 0x01;
  uint16_t preamp = manual ? (alex_data_1 >> 6) & 0x01 : max > 50000000;
  uint16_t ptt = alex_data_0 & 0x01;
  uint32_t freq = 0;
  uint16_t hpf = 0, lpf = 0, data = 0;

  if(i2c_codec) return;

  freq = freq_data[1] < freq_data[2] ? freq_data[1] : freq_data[2];

  if(preamp) hpf = 0;
  else if(manual) hpf = alex_data_1 & 0x3f;
  else if(freq < 1416000) hpf = 0x20; /* bypass */
  else if(freq < 6500000) hpf = 0x10; /* 1.5 MHz HPF */
  else if(freq < 9500000) hpf = 0x08; /* 6.5 MHz HPF */
  else if(freq < 13000000) hpf = 0x04; /* 9.5 MHz HPF */
  else if(freq < 20000000) hpf = 0x01; /* 13 MHz HPF */
  else hpf = 0x02; /* 20 MHz HPF */

  data =
    ptt << 15 |
    ((alex_data_0 >> 1) & 0x01) << 14 |
    ((alex_data_0 >> 2) & 0x01) << 13 |
    ((hpf >> 5) & 0x01) << 12 |
    ((alex_data_0 >> 7) & 0x01) << 11 |
    (((alex_data_0 >> 5) & 0x03) == 0x01) << 10 |
    (((alex_data_0 >> 5) & 0x03) == 0x02) << 9 |
    (((alex_data_0 >> 5) & 0x03) == 0x03) << 8 |
    ((hpf >> 2) & 0x07) << 4 |
    preamp << 3 |
    (hpf & 0x03) << 1 |
    1;

  if(alex_data_rx != data)
  {
    alex_data_rx = data;
    *alex = 1 << 16 | data;
  }

  freq = ptt ? freq_data[0] : max;

  if(manual) lpf = (alex_data_1 >> 8) & 0x7f;
  else if(freq > 32000000) lpf = 0x10; /* bypass */
  else if(freq > 22000000) lpf = 0x20; /* 12/10 meters */
  else if(freq > 15000000) lpf = 0x40; /* 17/15 meters */
  else if(freq > 8000000) lpf = 0x01; /* 30/20 meters */
  else if(freq > 4500000) lpf = 0x02; /* 60/40 meters */
  else if(freq > 2400000) lpf = 0x04; /* 80 meters */
  else lpf = 0x08; /* 160 meters */

  data =
    ((lpf >> 4) & 0x07) << 13 |
    ptt << 12 |
    (~(alex_data_1 >> 7) & ptt) << 11 |
    (((alex_data_0 >> 8) & 0x03) == 0x02) << 10 |
    (((alex_data_0 >> 8) & 0x03) == 0x01) << 9 |
    (((alex_data_0 >> 8) & 0x03) == 0x00) << 8 |
    (lpf & 0x0f) << 4 |
    1 << 3;

  if(alex_data_tx != data)
  {
    alex_data_tx = data;
    *alex = 1 << 17 | data;
  }
}

static inline int lower_bound(int *array, int size, int value)
{
  int i = 0, j = size, k;
  while(i < j)
  {
    k = i + (j - i) / 2;
    if(value > array[k]) i = k + 1;
    else j = k;
  }
  return i;
}

uint16_t misc_data_0 = 0;
uint16_t misc_data_1 = 0;
uint16_t misc_data_2 = 0;
uint16_t misc_update = 0;

void misc_write()
{
  uint16_t code[3], data = 0;
  int i, freqs[20] =
  {
     1700000,  2100000,
     3400000,  3900000,
     6900000,  7350000,
     9900000, 10250000,
    13900000, 14450000,
    17950000, 18250000,
    20900000, 21550000,
    24800000, 25100000,
    26900000, 30000000,
    49000000, 55000000
  };

  for(i = 0; i < 3; ++i)
  {
    code[i] = lower_bound(freqs, 20, freq_data[i]);
    code[i] = code[i] % 2 ? code[i] / 2 + 1 : 0;
  }

  data |= (code[0] != code[1]) << 8 | code[2] << 4 | code[1];
  data |= (misc_data_1 & 0x18) << 8 | (misc_data_0 & 0x18) << 6;
  data |= (misc_data_2 & 0x03) << 13;

  if(i2c_misc_data != data)
  {
    i2c_misc_data = data;
    ioctl(i2c_fd, I2C_SLAVE, ADDR_MISC);
    i2c_write_addr_data16(i2c_fd, 0x02, data);
  }
}

uint32_t nucleo_data_0 = 0;
uint32_t nucleo_data_1 = 0;
uint32_t nucleo_data_2 = 0;
uint32_t nucleo_data_3 = 0;
uint32_t nucleo_update = 0;

void nucleo_write()
{
  uint32_t data[2] = {0};
  uint32_t code[3];
  int i, freqs[22] =
  {
     1700000,  2100000,
     3400000,  3900000,
     5250000,  5450000,
     6900000,  7350000,
     9900000, 10250000,
    13900000, 14450000,
    17950000, 18250000,
    20900000, 21550000,
    24800000, 25100000,
    26900000, 30000000,
    49000000, 55000000
  };

  for(i = 0; i < 3; ++i)
  {
    code[i] = lower_bound(freqs, 22, freq_data[i]);
    code[i] = code[i] % 2 ? code[i] / 2 + 1 : 0;
  }

  data[0] = nucleo_data_3 << 25 | nucleo_data_2 << 20 | nucleo_data_1 << 17 | nucleo_data_0 << 1 | (code[0] != code[1]);
  data[1] = code[2] << 4 | code[1];

  if(i2c_nucleo_data[0] != data[0] || i2c_nucleo_data[1] != data[1])
  {
    i2c_nucleo_data[0] = data[0];
    i2c_nucleo_data[1] = data[1];
    ioctl(i2c_fd, I2C_SLAVE, ADDR_NUCLEO);
    i2c_write_data40(i2c_fd, data[0], data[1]);
  }
}

int uart_fd;
uint8_t icom_band_data = 0;

void icom_write()
{
  uint32_t freq = freq_data[0];
  uint8_t band;
  uint8_t buffer[10] = {0xfe, 0xfe, 0x04, 0xe0, 0x05, 0x00, 0x00, 0x08, 0x01, 0xfd};

  if(freq < 2000000) band = 0x01;       /* 160m */
  else if(freq < 4000000) band = 0x03;  /*  80m */
  else if(freq < 8000000) band = 0x07;  /*  40m */
  else if(freq < 11000000) band = 0x10; /*  30m */
  else if(freq < 15000000) band = 0x14; /*  20m */
  else if(freq < 20000000) band = 0x18; /*  17m */
  else if(freq < 22000000) band = 0x21; /*  15m */
  else if(freq < 26000000) band = 0x24; /*  12m */
  else band = 0x28;                     /*  10m */

  switch(band)
  {
    case 0x01:
      buffer[7] = 0x80;
      break;
    case 0x03:
      buffer[7] = 0x50;
      break;
    default:
      buffer[7] = 0x00;
  }

  buffer[8] = band;

  if(icom_band_data != band)
  {
    icom_band_data = band;
    if(uart_fd >= 0) write(uart_fd, buffer, 10);
  }
}
/* ---------------------------------------------------------------- state */

/* Everything the host tells us that outlives one packet.  Plain globals with
   no locking, as in the protocol 1 server: the control thread writes them and
   the streaming threads read them, and every one of them is a single word
   that is meaningful whichever side of an update it is read on. */

static volatile int running = 0;    /* high priority byte 4 bit 0 */
static volatile int ptt = 0;        /* high priority byte 4 bit 1 */
static volatile int phase_words = 1;/* general byte 37 bit 3 */

static struct sockaddr_in peer;
static volatile int have_peer = 0;

static int ddc_enable = 0;          /* DDC specific byte 7 */
static int ddc_rate[DDC_COUNT];     /* Hz, per DDC as asked for */
static int rate_hz = 48000;         /* the one rate the gateware runs at */

static uint32_t rx_phase[DDC_COUNT];
static uint32_t tx_phase = 0;
static uint8_t tx_drive = 0;
static uint8_t oc_data = 0;
static uint8_t step_att[2] = {0, 0};

static int sock_general, sock_rxspec, sock_txspec, sock_highprio;
static int sock_audio, sock_txiq;
static int sock_iq[DDC_COUNT];

static uint8_t mac[6];

static long long last_command_us = 0;

static long long now_us(void)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (long long)tv.tv_sec * 1000000LL + tv.tv_usec;
}

static void put_be32(uint8_t *p, uint32_t v)
{
  p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

static uint32_t get_be32(const uint8_t *p)
{
  return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static uint16_t get_be16(const uint8_t *p)
{
  return (uint16_t)p[0] << 8 | p[1];
}

/* ------------------------------------------------------- tuning */

/* Protocol 2 carries tuning as a 32 bit phase word referenced to 122.88 MHz
   when general byte 37 bit 3 is set, which every current host does, and as
   plain Hz otherwise.

   Both LOs take the same increment, phase / 2, which is the number the
   protocol 1 server wrote (f / 61.44e6 * 2^30) and which is correct on air.
   The transmit side runs in the 122.88 MHz domain, reached through the CDC
   FIFOs in tx.tcl, and this increment already accounts for it - do not
   "fix" it against the transmit clock. */

static uint32_t phase_to_hz(uint32_t v)
{
  if(!phase_words) return v;
  return (uint32_t)floor((double)v * 122.88e6 / 4294967296.0 + 0.5);
}

static uint32_t phase_to_inc(uint32_t v)
{
  if(phase_words) return v >> 1;
  return (uint32_t)floor((double)v / 61.44e6 * (1 << 30) + 0.5);
}

/* ------------------------------------------------------- RX data path */

/* The read pointer belongs to the streaming thread alone.  A restart is
   announced by bumping the epoch instead of writing the pointer from here,
   which would race with a read already in progress. */
static volatile uint32_t rx_epoch = 0;

static void rx_restart(void)
{
  /* Bump the epoch first: from here on the streaming thread's read pointer
     means nothing, and the counter it is compared against is about to be
     zeroed. */
  ++rx_epoch;
  *rx_rst &= ~1;
  usleep(1000);
  *rx_rst |= 1;
}

static void set_rate(int hz)
{
  uint16_t cic;

  switch(hz)
  {
    case 48000:   cic = 640; break;
    case 96000:   cic = 320; break;
    case 192000:  cic = 160; break;
    case 384000:  cic = 80; break;
    case 768000:  cic = 40; break;
    case 1536000: cic = 20; break;
    case 3072000: cic = 10; break;
    case 6144000: cic = 5; break;
    default: return;
  }

  if(rate_hz == hz) return;

  rate_hz = hz;
  *rx_rate = cic;
  rx_restart();
}

/* The gateware has one decimation register for all channels, so the rate of
   the lowest numbered enabled DDC wins. */
static void update_rate(void)
{
  int i;
  for(i = 0; i < DDC_COUNT; ++i)
  {
    if((ddc_enable & (1 << i)) && ddc_rate[i] >= RATE_MIN && ddc_rate[i] <= RATE_MAX)
    {
      set_rate(ddc_rate[i]);
      return;
    }
  }
}

/* ------------------------------------------------------- register blocks */

static void process_general(uint8_t *buffer)
{
  static int reported_wideband = 0;

  static int reported_ports = 0;
  int i;
  static const int fixed[8] = {PORT_RXSPEC, PORT_TXSPEC, PORT_HIGHPRIO, PORT_RXSPEC,
    PORT_AUDIO, PORT_TXIQ, PORT_IQ_BASE, PORT_TXSPEC};

  phase_words = (buffer[37] & 0x08) ? 1 : 0;

  for(i = 0; i < 8; ++i)
  {
    int port = get_be16(buffer + 5 + 2 * i);
    if(port == 0 || port == fixed[i] || reported_ports) continue;
    reported_ports = 1;
    fprintf(stderr, "hpsdr2: host asked to move a port off its default, not supported\n");
  }

  if((buffer[23] & 0x03) && !reported_wideband)
  {
    reported_wideband = 1;
    fprintf(stderr, "hpsdr2: host asked for the wideband ADC stream, not implemented\n");
  }
}

static void process_ddc_specific(uint8_t *buffer)
{
  int i, rate, enable;

  enable = buffer[7];

  for(i = 0; i < DDC_COUNT; ++i)
  {
    rate = get_be16(buffer + 18 + 6 * i) * 1000;
    if(rate >= RATE_MIN && rate <= RATE_MAX) ddc_rate[i] = rate;
  }

  ddc_enable = enable;
  update_rate();
}

static void process_duc_specific(uint8_t *buffer)
{
  static uint32_t sidetone_freq = 0;
  uint32_t freq;
  uint8_t data, boost;

  /* CW keyer.  Bit 1 says the host is in a CW mode at all, which is what
     arms the keyer here; without it the paddle inputs are ignored exactly as
     they are under protocol 1 with the internal keyer switched off. */
  cw_int_data = (buffer[5] >> 1) & 0x01;
  cw_reversed = (buffer[5] >> 2) & 0x01;
  if((buffer[5] & 0x28) == 0x28) cw_mode = 2;      /* iambic B */
  else if(buffer[5] & 0x08) cw_mode = 1;           /* iambic A */
  else cw_mode = 0;                                /* straight key */
  cw_spacing = (buffer[5] >> 6) & 0x01;
  cw_speed = buffer[9] ? buffer[9] : 25;
  cw_weight = buffer[10] ? buffer[10] : 50;
  cw_hang = get_be16(buffer + 11);
  cw_delay = buffer[13];

  if(i2c_codec)
  {
    /* sidetone level and frequency drive the codec DAC */
    data = buffer[6];
    if(dac_level_data != data)
    {
      dac_level_data = data;
      *dac_level = (int16_t)data * 64;
    }
    freq = get_be16(buffer + 7);
    if(sidetone_freq != freq)
    {
      sidetone_freq = freq;
      *dac_freq = (uint32_t)floor(freq / 48.0e3 * (1 << 30) + 0.5);
    }

    boost = (buffer[50] >> 1) & 0x01;
    if(i2c_boost_data != boost)
    {
      i2c_boost_data = boost;
      ioctl(i2c_fd, I2C_SLAVE, ADDR_CODEC);
      i2c_write_addr_data8(i2c_fd, 0x08, 0x14 + boost);
    }
  }
}

static void process_high_priority(uint8_t *buffer)
{
  uint32_t value, freq;
  uint16_t data;
  uint8_t att, preamp, ext_ptt, rx_ant, tx_ant;
  int i, run, was;

  run = buffer[4] & 0x01;
  ptt = (buffer[4] >> 1) & 0x01;

  /* DDC and DUC tuning */
  for(i = 0; i < DDC_COUNT; ++i)
  {
    value = get_be32(buffer + 9 + 4 * i);
    if(rx_phase[i] == value) continue;
    freq = phase_to_hz(value);
    if(freq > freq_max) continue;
    rx_phase[i] = value;
    rx_freq[i] = phase_to_inc(value);
    if(i > 1) continue;
    /* freq_data[1] and [2] are the two receive frequencies the filter and
       band switching code works from, as under protocol 1. */
    if(freq_data[i + 1] != freq)
    {
      freq_data[i + 1] = freq;
      alex_update = 1;
      if(i2c_misc) misc_update = 1;
      if(i2c_nucleo) nucleo_update = 1;
      if(i2c_arduino)
      {
        data = freq / 1000;
        if(i == 0 && data != i2c_ard_frx1_data)
        {
          i2c_ard_frx1_data = data;
          ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO);
          i2c_write_addr_data16(i2c_fd, 0x01, data);
        }
        if(i == 1 && data != i2c_ard_frx2_data)
        {
          i2c_ard_frx2_data = data;
          ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO);
          i2c_write_addr_data16(i2c_fd, 0x02, data);
        }
      }
    }
  }

  value = get_be32(buffer + 329);
  if(tx_phase != value)
  {
    freq = phase_to_hz(value);
    if(freq <= freq_max)
    {
      tx_phase = value;
      *tx_freq = phase_to_inc(value);
      if(freq_data[0] != freq)
      {
        freq_data[0] = freq;
        icom_write();
        alex_update = 1;
        if(i2c_misc) misc_update = 1;
        if(i2c_nucleo) nucleo_update = 1;
        if(i2c_arduino)
        {
          data = freq / 1000;
          if(data != i2c_ard_ftx_data)
          {
            i2c_ard_ftx_data = data;
            ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO);
            i2c_write_addr_data16(i2c_fd, 0x03, data);
          }
        }
      }
    }
  }

  /* drive level */
  if(tx_drive != buffer[345])
  {
    tx_drive = buffer[345];
    data = tx_drive;
    if(i2c_level)
    {
      if(i2c_level_data != data)
      {
        i2c_level_data = data;
        ioctl(i2c_fd, I2C_SLAVE, ADDR_LEVEL);
        i2c_write_addr_data16(i2c_fd, 0x02, data);
      }
    }
    else if(i2c_drive)
    {
      if(i2c_drive_data != data)
      {
        i2c_drive_data = data;
        ioctl(i2c_fd, I2C_SLAVE, ADDR_DRIVE);
        i2c_write_addr_data16(i2c_fd, 0xa9, data << 8 | data);
      }
    }
    else if(i2c_arduino)
    {
      value = i2c_ard_txatt_data & 0x00001f00;
      value |= log_table_lookup[data];
      if(value != i2c_ard_txatt_data)
      {
        i2c_ard_txatt_data = value;
        ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO);
        i2c_write_addr_data24(i2c_fd, 0x05, value);
      }
    }
    else
    {
      *tx_level = (int16_t)floor(data * 85.920 + 0.5);
    }
  }

  /* step attenuators: byte 1443 is ADC0, byte 1442 ADC1 */
  step_att[0] = buffer[1443] & 0x1f;
  step_att[1] = buffer[1442] & 0x1f;
  rx_att_data = step_att[0];

  if(i2c_misc && misc_data_0 != step_att[0])
  {
    misc_data_0 = step_att[0];
    misc_update = 1;
  }
  if(i2c_misc && misc_data_1 != step_att[1])
  {
    misc_data_1 = step_att[1];
    misc_update = 1;
  }
  if(i2c_nucleo && nucleo_data_2 != step_att[0])
  {
    nucleo_data_2 = step_att[0];
    nucleo_update = 1;
  }
  if(i2c_nucleo && nucleo_data_3 != step_att[1])
  {
    nucleo_data_3 = step_att[1];
    nucleo_update = 1;
  }
  if(i2c_arduino)
  {
    data = step_att[0] | (uint16_t)step_att[1] << 8;
    if(data != i2c_ard_rxatt_data)
    {
      i2c_ard_rxatt_data = data;
      ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO);
      i2c_write_addr_data16(i2c_fd, 0x06, data);
    }
  }

  /* open collector outputs, byte 1401 bits 1..7 as on the wire */
  oc_data = buffer[1401];

  /* The ALEX word the host computes carries the antenna choice, the ALEX
     attenuator and the filter selection.  The filters are recomputed here
     from the frequencies, as under protocol 1, because that path is what
     drives this board's I2C filter hardware; the antenna and attenuator bits
     are taken from the host. */
  value = get_be32(buffer + 1432);
  ext_ptt = *gpio_in & 1;

  /* ALEX attenuator, bits 13 and 14 */
  att = 0;
  if(value & 0x00004000) att |= 1;
  if(value & 0x00002000) att |= 2;

  /* RX antenna, in the numbering the shift register wants:
     0 = none, 1 = RX1 (EXT1), 2 = RX2 (EXT2), 3 = XVTR */
  rx_ant = 0;
  if(value & 0x00000200) rx_ant = 1;
  else if(value & 0x00000400) rx_ant = 2;
  else if(value & 0x00000100) rx_ant = 3;

  /* TX antenna: 0 = ANT1, 1 = ANT2, 2 = ANT3 */
  tx_ant = 0;
  if(value & 0x02000000) tx_ant = 1;
  else if(value & 0x04000000) tx_ant = 2;

  data = (ptt | cw_ptt);
  data |= (uint16_t)att << 1;
  data |= (uint16_t)rx_ant << 5;
  data |= (uint16_t)((value >> 11) & 1) << 7; /* ALEX RX output enabled */
  data |= (uint16_t)tx_ant << 8;

  if(alex_data_0 != data)
  {
    alex_data_0 = data;
    alex_update = 1;
  }

  /* output pins: PTT, preamp, ALEX attenuator, open collectors */
  preamp = (ptt || cw_ptt || ext_ptt) ? 0 : (rx_att_data == 0);
  *gpio_out = (oc_data & 0x1e) << 3 | att << 2 | preamp << 1 | (ptt | cw_ptt);

  if(i2c_pene)
  {
    data = (oc_data >> 1) | (uint16_t)att << 7 |
      (uint16_t)rx_ant << 9 | (uint16_t)tx_ant << 11;
    if(i2c_pene_data != data)
    {
      i2c_pene_data = data;
      ioctl(i2c_fd, I2C_SLAVE, ADDR_PENE);
      i2c_write_addr_data16(i2c_fd, 0x02, data);
    }
  }

  if(i2c_dac0)
  {
    data = rx_att_data + 10 * att;
    data = 4065 - (uint32_t)data * 4095 / 61;
    if(i2c_dac0_data != data)
    {
      i2c_dac0_data = data;
      ioctl(i2c_fd, I2C_SLAVE, ADDR_DAC0);
      i2c_write_data16(i2c_fd, data);
    }
  }

  was = running;
  if(run != was)
  {
    running = run;
    if(run)
    {
      *lo_rst &= ~3;
      *lo_rst |= 3;
      rx_restart();
      if(i2c_codec)
      {
        /* resync the microphone stream with the packets we are about to send */
        *rx_rst &= ~2;
        *rx_rst |= 2;
      }
      fprintf(stderr, "hpsdr2: start, DDC enable 0x%02x at %d Hz\n", ddc_enable, rate_hz);
    }
    else
    {
      fprintf(stderr, "hpsdr2: stop\n");
    }
  }
}

/* ------------------------------------------------------- sample streams */

/* TX I/Q arrives 24 bit at 192 kHz; the DUC takes the top 16 bits of each.
   The gateware reads four 16 bit channels per sample: the first pair is the
   envelope path, which protocol 2 has no use for, and the second is I and Q
   in the same byte order protocol 1 delivered them. */
static void process_tx_iq(uint8_t *buffer)
{
  int i;
  uint32_t word;
  uint8_t *p;

  if(tx_mux_data) return; /* the CW keyer owns the transmit path */

  if(*tx_cntr > TX_FIFO_HIGH) return; /* host is running fast, drop a packet */

  if(*tx_cntr < 32)
  {
    /* underrun: prime the FIFO so the next packets are not chasing it */
    for(i = 0; i < TX_FIFO_LOW; ++i) *fifo = 0;
  }

  if(!(ptt || cw_ptt || (*gpio_in & 1)))
  {
    for(i = 0; i < TXIQ_SAMPLES; ++i)
    {
      *fifo = 0;
      *fifo = 0;
    }
    return;
  }

  for(i = 0; i < TXIQ_SAMPLES; ++i)
  {
    p = buffer + 4 + i * 6;
    /* I and Q, most significant 16 bits of each, in the byte order the
       gateware's byte swapper expects */
    word = (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[3] << 16 | (uint32_t)p[4] << 24;
    *fifo = 0; /* envelope channels, unused by protocol 2 */
    *fifo = word;
  }
}

/* RX audio: 64 stereo 16 bit samples at 48 kHz, the same layout the codec
   FIFO takes. */
static void process_audio(uint8_t *buffer)
{
  int i;

  if(!i2c_codec) return;

  if(*dac_cntr > DAC_FIFO_HIGH) return;

  if(*dac_cntr < 16)
  {
    for(i = 0; i < DAC_FIFO_LOW; ++i) *codec = 0;
  }

  for(i = 0; i < AUDIO_SAMPLES; ++i)
  {
    *codec = *(uint32_t *)(buffer + 4 + i * 4);
  }
}

static void build_iq_packet(uint8_t *pkt, int ddc, uint32_t seq, uint64_t stamp,
  int nsamples, uint32_t first)
{
  int i;
  uint8_t *p = pkt + 16;
  uint32_t set = first;

  put_be32(pkt, seq);
  for(i = 0; i < 8; ++i) pkt[4 + i] = (stamp >> (56 - 8 * i)) & 0xff;
  pkt[12] = 0;
  pkt[13] = 24;
  pkt[14] = (nsamples >> 8) & 0xff;
  pkt[15] = nsamples & 0xff;

  for(i = 0; i < nsamples; ++i)
  {
    memcpy(p, ram + (size_t)(set & (RX_SETS - 1)) * RX_SET_BYTES + ddc * 6, 6);
    p += 6;
    ++set;
  }
}

/* One thread for all DDCs: they share the decimation rate, so their samples
   arrive in the same 16 byte sample set and their packets are due at the same
   moment. */
static void *handler_ddc(void *arg)
{
  int d, i, n, batch;
  uint32_t avail, cur, epoch = 0, rd_set = 0;
  int resync = 1;
  long long last_overrun = 0;
  uint32_t seq[DDC_COUNT];
  uint64_t stamp[DDC_COUNT];
  static uint8_t buffer[DDC_COUNT][8][IQ_PKT_BYTES];
  struct iovec iovec[DDC_COUNT][8][1];
  struct mmsghdr datagram[DDC_COUNT][8];

  memset(seq, 0, sizeof(seq));
  memset(stamp, 0, sizeof(stamp));

  while(1)
  {
    if(!running || !have_peer)
    {
      memset(seq, 0, sizeof(seq));
      memset(stamp, 0, sizeof(stamp));
      epoch = rx_epoch;
      resync = 1;
      usleep(5000);
      continue;
    }

    if(epoch != rx_epoch)
    {
      epoch = rx_epoch;
      resync = 1;
    }

    if(!(*rx_rst & 1))
    {
      /* the writer is held in reset, its counter means nothing */
      resync = 1;
      usleep(500);
      continue;
    }

    /* one packet is 238 samples; batch enough of them to keep the wakeup rate
       near 500 Hz at any sample rate, and no more than 8 */
    batch = rate_hz / (IQ_SAMPLES * 500);
    if(batch < 1) batch = 1;
    if(batch > 8) batch = 8;

    cur = (uint32_t)*rx_cntr * (128 / RX_SET_BYTES);

    if(resync)
    {
      /* Start from wherever the writer is now: samples from before the
         restart are of no use. */
      resync = 0;
      rd_set = cur;
      continue;
    }

    avail = (cur - rd_set) & (RX_SETS - 1);

    if(avail > RX_SETS - RX_SETS / 4)
    {
      /* We fell far enough behind that the writer is about to lap us. */
      if(now_us() - last_overrun > 1000000)
      {
        last_overrun = now_us();
        fprintf(stderr, "hpsdr2: RX ring overrun at %d Hz, resynchronising\n", rate_hz);
      }
      rd_set = cur;
      continue;
    }

    if(avail < (uint32_t)(IQ_SAMPLES * batch))
    {
      usleep(500);
      continue;
    }

    n = avail / IQ_SAMPLES;
    if(n > 8) n = 8;

    for(d = 0; d < DDC_COUNT; ++d)
    {
      if(!(ddc_enable & (1 << d))) continue;
      memset(iovec[d], 0, sizeof(iovec[d]));
      memset(datagram[d], 0, sizeof(datagram[d]));
      for(i = 0; i < n; ++i)
      {
        build_iq_packet(buffer[d][i], d, seq[d]++, stamp[d], IQ_SAMPLES,
          rd_set + (uint32_t)i * IQ_SAMPLES);
        stamp[d] += IQ_SAMPLES;
        iovec[d][i][0].iov_base = buffer[d][i];
        iovec[d][i][0].iov_len = IQ_PKT_BYTES;
        datagram[d][i].msg_hdr.msg_iov = iovec[d][i];
        datagram[d][i].msg_hdr.msg_iovlen = 1;
        datagram[d][i].msg_hdr.msg_name = &peer;
        datagram[d][i].msg_hdr.msg_namelen = sizeof(peer);
      }
      sendmmsg(sock_iq[d], datagram[d], n, 0);
    }

    rd_set = (rd_set + (uint32_t)n * IQ_SAMPLES) & (RX_SETS - 1);
  }

  return NULL;
}

/* Microphone samples, 64 per packet at 48 kHz.  Sent whether or not a codec
   is fitted: this stream is what clocks the host's transmit chain. */
static void *handler_mic(void *arg)
{
  int i;
  uint32_t seq = 0;
  uint32_t audio[MIC_SAMPLES];
  uint8_t buffer[MIC_PKT_BYTES];
  struct timespec ts;

  memset(buffer, 0, sizeof(buffer));
  clock_gettime(CLOCK_MONOTONIC, &ts);

  while(1)
  {
    if(!running || !have_peer)
    {
      seq = 0;
      usleep(5000);
      clock_gettime(CLOCK_MONOTONIC, &ts);
      continue;
    }

    if(i2c_codec)
    {
      if(*adc_cntr < MIC_SAMPLES)
      {
        usleep(500);
        continue;
      }
      memcpy(audio, (void *)codec, MIC_SAMPLES * 4);
      for(i = 0; i < MIC_SAMPLES; ++i) memcpy(buffer + 4 + i * 2, &audio[i], 2);
    }
    else
    {
      /* 64 samples at 48 kHz */
      ts.tv_nsec += 1333333;
      while(ts.tv_nsec >= 1000000000)
      {
        ts.tv_nsec -= 1000000000;
        ++ts.tv_sec;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
    }

    put_be32(buffer, seq++);
    sendto(sock_txspec, buffer, MIC_PKT_BYTES, 0, (struct sockaddr *)&peer, sizeof(peer));
  }

  return NULL;
}

/* ADS1115 in place of the XADC: AIN0 forward, AIN1 reverse, AIN2 supply.
   Single-shot conversions at 860 SPS taken in turn, so each input is at most
   about 4 ms old.  Results are rescaled to what the Hermes 12-bit converter
   with its 3.3 V reference would report, which is what the host's power
   formula assumes. */
static void *handler_ads(void *arg)
{
  uint8_t buffer[3];
  int16_t raw;
  int32_t value;
  int i = 0, j;

  while(1)
  {
    /* OS, AINi to GND, +-4.096 V, single shot, 860 SPS, comparator off */
    buffer[0] = 0x01;
    buffer[1] = 0xc3 | (i << 4);
    buffer[2] = 0xe3;
    if(write(ads_fd, buffer, 3) != 3)
    {
      usleep(100000);
      continue;
    }

    /* the conversion register keeps the previous input's result until this
       one is done, so wait for OS to come back rather than trust a delay */
    usleep(1200);
    for(j = 0; j < 10; ++j)
    {
      if(write(ads_fd, buffer, 1) != 1 || read(ads_fd, buffer + 1, 2) != 2)
      {
        buffer[1] = 0;
        break;
      }
      if(buffer[1] & 0x80) break;
      usleep(100);
    }
    if(!(buffer[1] & 0x80))
    {
      usleep(100000);
      continue;
    }

    buffer[0] = 0x00;
    if(write(ads_fd, buffer, 1) != 1 || read(ads_fd, buffer, 2) != 2)
    {
      usleep(100000);
      continue;
    }

    /* 125 uV per count, so 3.3 V is 26400 */
    raw = buffer[0] << 8 | buffer[1];
    value = raw * 4095 / 26400;
    ads_data[i] = value < 0 ? 0 : value > 4095 ? 4095 : value;

    if(++i == 3) i = 0;
  }

  return NULL;
}

/* High priority status, 20 per second while receiving and every millisecond
   while transmitting, where the host wants the power readings promptly. */
static void *handler_status(void *arg)
{
  uint32_t seq = 0;
  uint16_t value;
  uint8_t buffer[60];
  uint8_t in;

  while(1)
  {
    if(!have_peer)
    {
      usleep(50000);
      continue;
    }

    memset(buffer, 0, sizeof(buffer));
    put_be32(buffer, seq++);

    in = *gpio_in;
    buffer[4] = (in & 1) | cw_ptt;      /* PTT */
    buffer[4] |= ((in >> 1) & 1) << 1;  /* dot */
    buffer[4] |= ((in >> 2) & 1) << 2;  /* dash */
    buffer[4] |= 0x08;                  /* PLL locked */

    /* The ADS1115 if one answered, otherwise the same XADC channels the
       protocol 1 server reported. */
    value = ads_fd >= 0 ? ads_data[0] : xadc[30] >> 3;
    buffer[14] = value >> 8;
    buffer[15] = value;
    value = ads_fd >= 0 ? ads_data[1] : xadc[20] >> 3;
    buffer[22] = value >> 8;
    buffer[23] = value;
    value = ads_fd >= 0 ? ads_data[2] : xadc[31] >> 3;
    buffer[49] = value >> 8;
    buffer[50] = value;

    buffer[59] = (in >> 3) & 0x1f;      /* digital user inputs */

    sendto(sock_rxspec, buffer, sizeof(buffer), 0, (struct sockaddr *)&peer, sizeof(peer));

    usleep((ptt || cw_ptt) ? 1000 : 50000);
  }

  return NULL;
}

/* ------------------------------------------------------- discovery */

static void send_discovery_reply(int sock, struct sockaddr_in *from, socklen_t len)
{
  uint8_t buffer[60];

  memset(buffer, 0, sizeof(buffer));
  buffer[4] = 2 + (running ? 1 : 0);
  memcpy(buffer + 5, mac, 6);
  buffer[11] = BOARD_HERMES;
  buffer[12] = 38;                  /* protocol version 3.8 */
  buffer[13] = GATEWARE_VERSION;
  buffer[20] = DDC_COUNT;           /* DDCs this board offers */
  buffer[21] = 1;                   /* DUCs */
  buffer[22] = 3;                   /* DDC sample width in bytes */

  sendto(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)from, len);
}

/* ------------------------------------------------------- main */

static int bind_udp(int port)
{
  int sock, yes = 1;
  struct sockaddr_in addr;

  if((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket");
    return -1;
  }

  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));

  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);

  if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    fprintf(stderr, "cannot bind UDP %d: %s\n", port, strerror(errno));
    close(sock);
    return -1;
  }

  return sock;
}

int main(int argc, char *argv[])
{
  int fd, i, size;
  uint16_t data16;
  uint32_t data32;
  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  volatile void *cfg, *sts;
  volatile int32_t *tx_ramp;
  volatile uint16_t *tx_size, *dac_size;
  volatile int16_t *dac_ramp;
  volatile uint8_t *rx_sel, *tx_sel;
  float scale, ramp[1024], a[4] = {0.35875, 0.48829, 0.14128, 0.01168};
  uint32_t buffer_size;
  struct termios tty;
  struct ifreq hwaddr;
  struct sockaddr_in from;
  socklen_t fromlen;
  uint8_t buffer[2048];
  struct pollfd pfd[6];
  ssize_t bytes;
  char *end;
  uint8_t chan = 0;
  long number;

  /* One ADC per argument: two DDC channels then the two TX channels.  This
     board wires only ADC A, so the DDC arguments have nothing to choose
     between and exist to keep the selectors in the gateware driven. */
  for(i = 0; i < 4; ++i)
  {
    errno = 0;
    number = (argc == 5) ? strtol(argv[i + 1], &end, 10) : -1;
    if(errno != 0 || end == argv[i + 1] || number < 1 || number > 2)
    {
      fprintf(stderr, "Usage: sdr-transceiver-hpsdr 1|2 1|2 1|2 1|2\n");
      return EXIT_FAILURE;
    }
    chan |= (number - 1) << i;
  }

  if((fd = open("/dev/mem", O_RDWR)) < 0)
  {
    perror("open");
    return EXIT_FAILURE;
  }

  if((uart_fd = open("/dev/ttyPS1", O_RDWR | O_NOCTTY | O_NDELAY)) >= 0)
  {
    tcgetattr(uart_fd, &tty);
    cfsetspeed(&tty, (speed_t)B1200);
    cfmakeraw(&tty);
    tty.c_cflag &= ~(CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &tty);
  }

  if((i2c_fd = open("/dev/i2c-0", O_RDWR)) >= 0)
  {
    if(ioctl(i2c_fd, I2C_SLAVE_FORCE, ADDR_PENE) >= 0)
    {
      /* set all pins to low */
      if(i2c_write_addr_data16(i2c_fd, 0x02, 0x0000) > 0)
      {
        i2c_pene = 1;
        /* configure all pins as output */
        i2c_write_addr_data16(i2c_fd, 0x06, 0x0000);
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_ALEX) >= 0)
    {
      if(i2c_write_addr_data16(i2c_fd, 0x02, 0x0000) > 0)
      {
        i2c_alex = 1;
        i2c_write_addr_data16(i2c_fd, 0x06, 0x0000);
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_LEVEL) >= 0)
    {
      if(i2c_write_addr_data16(i2c_fd, 0x02, 0x0000) > 0)
      {
        i2c_level = 1;
        i2c_write_addr_data16(i2c_fd, 0x06, 0x0000);
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_MISC) >= 0)
    {
      if(i2c_write_addr_data16(i2c_fd, 0x02, 0x0000) > 0)
      {
        i2c_misc = 1;
        i2c_write_addr_data16(i2c_fd, 0x06, 0x0000);
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_DRIVE) >= 0)
    {
      /* set both potentiometers to 0 */
      if(i2c_write_addr_data16(i2c_fd, 0xa9, 0x0000) > 0)
      {
        i2c_drive = 1;
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_DAC0) >= 0)
    {
      if(i2c_write_data16(i2c_fd, i2c_dac0_data) > 0)
      {
        i2c_dac0 = 1;
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_DAC1) >= 0)
    {
      if(i2c_write_data16(i2c_fd, i2c_dac1_data) > 0)
      {
        i2c_dac1 = 1;
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_ARDUINO) >= 0)
    {
      if(i2c_write_addr_data16(i2c_fd, 0x1, i2c_ard_frx1_data) > 0)
      {
        i2c_arduino = 1;
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_NUCLEO) >= 0)
    {
      if(i2c_write_data40(i2c_fd, 0x02100000, 0) > 0)
      {
        i2c_nucleo = 1;
      }
    }
    if(ioctl(i2c_fd, I2C_SLAVE, ADDR_CODEC) >= 0)
    {
      /* reset */
      if(i2c_write_addr_data8(i2c_fd, 0x1e, 0x00) > 0)
      {
        i2c_codec = 1;
        /* set power down register */
        i2c_write_addr_data8(i2c_fd, 0x0c, 0x51);
        /* reset activate register */
        i2c_write_addr_data8(i2c_fd, 0x12, 0x00);
        /* set volume to -10 dB */
        i2c_write_addr_data8(i2c_fd, 0x04, 0x6f);
        i2c_write_addr_data8(i2c_fd, 0x06, 0x6f);
        /* set analog audio path register */
        i2c_write_addr_data8(i2c_fd, 0x08, 0x14);
        /* set digital audio path register */
        i2c_write_addr_data8(i2c_fd, 0x0a, 0x00);
        /* set format register */
        i2c_write_addr_data8(i2c_fd, 0x0e, 0x42);
        /* set activate register */
        i2c_write_addr_data8(i2c_fd, 0x12, 0x01);
        /* set power down register */
        i2c_write_addr_data8(i2c_fd, 0x0c, 0x41);
      }
    }
  }

  if((ads_fd = open("/dev/i2c-0", O_RDWR)) >= 0)
  {
    /* a bare pointer write is acknowledged only if the chip is there */
    buffer[0] = 0x01;
    if(ioctl(ads_fd, I2C_SLAVE, ADDR_ADS1115) < 0 || write(ads_fd, buffer, 1) != 1)
    {
      close(ads_fd);
      ads_fd = -1;
    }
  }

  cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40000000);
  sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x41000000);
  fifo = mmap(NULL, 8*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x42000000);
  codec = mmap(NULL, 2*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x43000000);
  xadc = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x44000000);
  alex = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x45000000);
  tx_ramp = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x46000000);
  dac_ramp = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x47000000);

  close(fd);

  /* The receive stream is written straight into DDR over the ACP port, so the
     ring buffer has to be physically contiguous and cache coherent. */
  if((fd = open("/dev/cma", O_RDWR)) < 0)
  {
    perror("open /dev/cma");
    return EXIT_FAILURE;
  }

  buffer_size = RX_RAM_SIZE;

  if(ioctl(fd, CMA_ALLOC, &buffer_size) < 0)
  {
    perror("ioctl CMA_ALLOC");
    return EXIT_FAILURE;
  }

  ram = mmap(NULL, RX_RAM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

  if(ram == MAP_FAILED)
  {
    perror("mmap /dev/cma");
    return EXIT_FAILURE;
  }

  rx_rst = (uint8_t *)(cfg + 0);
  lo_rst = (uint8_t *)(cfg + 1);
  tx_rst = (uint8_t *)(cfg + 2);
  gpio_out = (uint8_t *)(cfg + 3);

  rx_rate = (uint16_t *)(cfg + 4);

  rx_sel = (uint8_t *)(cfg + 6);

  rx_freq = (uint32_t *)(cfg + 8);

  tx_freq = (uint32_t *)(cfg + 20);
  tx_size = (uint16_t *)(cfg + 24);
  tx_level = (int16_t *)(cfg + 26);

  tx_sel = (uint8_t *)(cfg + 30);

  dac_freq = (uint32_t *)(cfg + 32);
  dac_size = (uint16_t *)(cfg + 36);
  dac_level = (int16_t *)(cfg + 38);

  rx_ram = (uint32_t *)(cfg + 40);
  rx_wrap = (uint16_t *)(cfg + 44);

  rx_cntr = (uint16_t *)(sts + 0);
  tx_cntr = (uint16_t *)(sts + 2);
  dac_cntr = (uint16_t *)(sts + 4);
  adc_cntr = (uint16_t *)(sts + 6);
  gpio_in = (uint8_t *)(sts + 8);

  /* hold the DMA writer in reset until it has somewhere to write */
  *rx_rst &= ~1;
  *rx_ram = buffer_size;
  *rx_wrap = RX_BURSTS - 1;

  /* set rx and tx selectors */
  *rx_sel = chan & 3;
  *tx_sel = (chan >> 2) & 3;

  /* set all GPIO pins to low */
  *gpio_out = 0;

  /* set default rx phase increment */
  rx_freq[0] = (uint32_t)floor(600000 / 61.44e6 * (1 << 30) + 0.5);
  rx_freq[1] = (uint32_t)floor(600000 / 61.44e6 * (1 << 30) + 0.5);

  /* set default rx sample rate */
  *rx_rate = 640;

  /* set default tx phase increment */
  *tx_freq = (uint32_t)floor(600000 / 61.44e6 * (1 << 30) + 0.5);

  /* set tx ramp */
  size = 1001;
  ramp[0] = 0.0;
  for(i = 1; i <= size; ++i)
  {
    ramp[i] = ramp[i - 1] + a[0] - a[1] * cos(2.0 * M_PI * i / size) + a[2] * cos(4.0 * M_PI * i / size) - a[3] * cos(6.0 * M_PI * i / size);
  }
  scale = 4.1e6 / ramp[size];
  for(i = 0; i <= size; ++i)
  {
    tx_ramp[i] = (int32_t)floor(ramp[i] * scale + 0.5);
  }
  *tx_size = size;

  /* set default tx level */
  *tx_level = 21910;

  /* set default tx mux channel */
  *tx_rst &= ~16;

  /* reset tx and codec DAC fifo */
  *tx_rst &= ~3;
  *tx_rst |= 3;

  /* reset tx lo */
  *lo_rst &= ~4;
  *lo_rst |= 4;

  if(i2c_codec)
  {
    /* reset codec ADC fifo */
    *rx_rst &= ~2;
    *rx_rst |= 2;
    /* enable I2S interface */
    *rx_rst &= ~4;

    /* set default dac phase increment */
    *dac_freq = (uint32_t)floor(600 / 48.0e3 * (1 << 30) + 0.5);

    /* set dac ramp */
    size = 481;
    ramp[0] = 0.0;
    for(i = 1; i <= size; ++i)
    {
      ramp[i] = ramp[i - 1] + a[0] - a[1] * cos(2.0 * M_PI * i / size) + a[2] * cos(4.0 * M_PI * i / size) - a[3] * cos(6.0 * M_PI * i / size);
    }
    scale = 3.2e4 / ramp[size];
    for(i = 0; i <= size; ++i)
    {
      dac_ramp[i] = (int16_t)floor(ramp[i] * scale + 0.5);
    }
    *dac_size = size;

    /* set default dac level */
    *dac_level = 3200;
  }
  else
  {
    /* enable ALEX interface */
    *rx_rst |= 4;
  }

  calc_log_lookup();

  for(i = 0; i < DDC_COUNT; ++i) ddc_rate[i] = 48000;

  if((sock_general = bind_udp(PORT_GENERAL)) < 0) return EXIT_FAILURE;
  if((sock_rxspec = bind_udp(PORT_RXSPEC)) < 0) return EXIT_FAILURE;
  if((sock_txspec = bind_udp(PORT_TXSPEC)) < 0) return EXIT_FAILURE;
  if((sock_highprio = bind_udp(PORT_HIGHPRIO)) < 0) return EXIT_FAILURE;
  if((sock_audio = bind_udp(PORT_AUDIO)) < 0) return EXIT_FAILURE;
  if((sock_txiq = bind_udp(PORT_TXIQ)) < 0) return EXIT_FAILURE;
  for(i = 0; i < DDC_COUNT; ++i)
  {
    if((sock_iq[i] = bind_udp(PORT_IQ_BASE + i)) < 0) return EXIT_FAILURE;
  }

  memset(&hwaddr, 0, sizeof(hwaddr));
  strncpy(hwaddr.ifr_name, "eth0", IFNAMSIZ - 1);
  ioctl(sock_general, SIOCGIFHWADDR, &hwaddr);
  for(i = 0; i < 6; ++i) mac[i] = hwaddr.ifr_addr.sa_data[i];

  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  param.sched_priority = 99;
  pthread_attr_setschedparam(&attr, &param);
  if(pthread_create(&thread, &attr, handler_keyer, NULL) != 0)
  {
    /* Without the real time policy the keyer's timing suffers but everything
       else works, so this is a warning and not a failure. */
    fprintf(stderr, "hpsdr2: cannot start the keyer thread with SCHED_FIFO\n");
    if(pthread_create(&thread, NULL, handler_keyer, NULL) != 0)
    {
      perror("pthread_create");
      return EXIT_FAILURE;
    }
  }
  pthread_detach(thread);

  if(pthread_create(&thread, NULL, handler_ddc, NULL) != 0)
  {
    perror("pthread_create");
    return EXIT_FAILURE;
  }
  pthread_detach(thread);

  if(pthread_create(&thread, NULL, handler_mic, NULL) != 0)
  {
    perror("pthread_create");
    return EXIT_FAILURE;
  }
  pthread_detach(thread);

  if(ads_fd >= 0)
  {
    if(pthread_create(&thread, NULL, handler_ads, NULL) != 0)
    {
      perror("pthread_create");
      return EXIT_FAILURE;
    }
    pthread_detach(thread);
  }

  if(pthread_create(&thread, NULL, handler_status, NULL) != 0)
  {
    perror("pthread_create");
    return EXIT_FAILURE;
  }
  pthread_detach(thread);

  fprintf(stderr, "hpsdr2: %d DDCs, ring %d MiB at 0x%08x\n",
    DDC_COUNT, RX_RAM_SIZE / (1024 * 1024), buffer_size);
  fprintf(stderr, "hpsdr2: power and supply from the %s\n", ads_fd >= 0 ? "ADS1115" : "XADC");

  pfd[0].fd = sock_general;
  pfd[1].fd = sock_rxspec;
  pfd[2].fd = sock_txspec;
  pfd[3].fd = sock_highprio;
  pfd[4].fd = sock_audio;
  pfd[5].fd = sock_txiq;
  for(i = 0; i < 6; ++i) pfd[i].events = POLLIN;

  while(1)
  {
    poll(pfd, 6, 100);

    for(i = 0; i < 6; ++i)
    {
      if(!(pfd[i].revents & POLLIN)) continue;

      while(1)
      {
        fromlen = sizeof(from);
        bytes = recvfrom(pfd[i].fd, buffer, sizeof(buffer), MSG_DONTWAIT,
          (struct sockaddr *)&from, &fromlen);
        if(bytes < 0) break;

        /* Discovery arrives on the general register port and has to be told
           apart from a general register block, which is also 60 bytes with a
           zero sequence number on its first send: byte 4 is the command. */
        if(pfd[i].fd == sock_general && bytes == 60 && get_be32(buffer) == 0 &&
          (buffer[4] == 0x02 || buffer[4] == 0x06))
        {
          send_discovery_reply(pfd[i].fd, &from, fromlen);
          continue;
        }

        /* A protocol 1 host on a protocol 2 board. */
        if(bytes >= 2 && buffer[0] == 0xef && buffer[1] == 0xfe) continue;

        peer = from;
        have_peer = 1;
        last_command_us = now_us();

        if(pfd[i].fd == sock_general && bytes == 60) process_general(buffer);
        else if(pfd[i].fd == sock_rxspec && bytes == REGISTER_PKT_BYTES) process_ddc_specific(buffer);
        else if(pfd[i].fd == sock_txspec && bytes == 60) process_duc_specific(buffer);
        else if(pfd[i].fd == sock_highprio && bytes == REGISTER_PKT_BYTES) process_high_priority(buffer);
        else if(pfd[i].fd == sock_audio && bytes == AUDIO_PKT_BYTES) process_audio(buffer);
        else if(pfd[i].fd == sock_txiq && bytes == TXIQ_PKT_BYTES) process_tx_iq(buffer);
      }
    }

    /* The host stops sending when it goes away without saying so. */
    if(running && now_us() - last_command_us > 2000000)
    {
      fprintf(stderr, "hpsdr2: host went quiet, stopping\n");
      running = 0;
      ptt = 0;
      *gpio_out = 0;
    }

    data16 = alex_data_0 | cw_ptt;
    if(alex_data_0 != data16)
    {
      alex_data_0 = data16;
      alex_update = 1;
    }

    if(alex_update)
    {
      alex_update = 0;
      alex_write();
    }

    if(misc_update)
    {
      misc_update = 0;
      misc_write();
    }

    data32 = nucleo_data_0 | cw_ptt;
    if(nucleo_data_0 != data32)
    {
      nucleo_data_0 = data32;
      nucleo_update = 1;
    }

    if(nucleo_update)
    {
      nucleo_update = 0;
      nucleo_write();
    }
  }

  return EXIT_SUCCESS;
}
static inline int cw_input()
{
  int input;
  if(!cw_int_data) return 0;
  input = (*gpio_in >> 1) & 3;
  if(cw_reversed) input = (input & 1) << 1 | input >> 1;
  return input;
}

static inline void cw_on()
{
  int delay = 1200 / cw_speed;
  if(cw_delay < delay) delay = cw_delay;
  /* PTT on */
  *tx_rst |= 32;
  cw_ptt = 1;
  *tx_rst |= 16;
  tx_mux_data = 1;
  if(i2c_codec && dac_level_data > 0)
  {
    *tx_rst |= 8; /* sidetone on */
  }
  while(delay--)
  {
    usleep(1000);
    cw_memory[0] = cw_input();
    if(cw_mode == 1 && !cw_memory[0]) cw_memory[1] = 0;
    else cw_memory[1] |= cw_memory[0];
  }
  *tx_rst |= 4; /* RF on */
}

static inline void cw_off()
{
  int delay = 1200 / cw_speed;
  if(cw_delay < delay) delay = cw_delay;
  if(i2c_codec)
  {
    *tx_rst &= ~8; /* sidetone off */
  }
  while(delay--)
  {
    usleep(1000);
    cw_memory[0] = cw_input();
    cw_memory[1] |= cw_memory[0];
  }
  *tx_rst &= ~4; /* RF off */
  cw_ptt_delay = cw_hang > 0 ? cw_hang : 10;
}

static inline void cw_ptt_off()
{
  if(--cw_ptt_delay > 0) return;
  /* PTT off */
  *tx_rst &= ~32;
  cw_ptt = 0;
  /* reset tx fifo */
  *tx_rst &= ~1;
  *tx_rst |= 1;
  *tx_rst &= ~16;
  tx_mux_data = 0;
}

static inline void cw_signal_delay(int code)
{
  int delay = code ? 1200 / cw_speed : 3600 * cw_weight / (50 * cw_speed);
  delay -= cw_delay;
  if(delay < 0) delay = 0;
  while(delay--)
  {
    usleep(1000);
    cw_memory[0] = cw_input();
    if(cw_mode == 1 && !cw_memory[0]) cw_memory[1] = 0;
    else cw_memory[1] |= cw_memory[0];
  }
}

static inline void cw_space_delay(int code)
{
  int delay = code ? 1200 / cw_speed - cw_delay : 2400 / cw_speed;
  if(delay < 0) delay = 0;
  while(delay--)
  {
    usleep(1000);
    if(cw_ptt) cw_ptt_off();
    cw_memory[0] = cw_input();
    cw_memory[1] |= cw_memory[0];
  }
}

void *handler_keyer(void *arg)
{
  int state, delay;

  while(1)
  {
    usleep(1000);
    if(cw_ptt) cw_ptt_off();
    if(!(cw_memory[0] = cw_input())) continue;

    if(cw_mode == 0)
    {
      if(cw_memory[0] & 1)
      {
        cw_on();
        while(cw_memory[0] & 1)
        {
          usleep(1000);
          cw_memory[0] = cw_input();
        }
        cw_off();
      }
      else
      {
        cw_on();
        delay = 1200 / cw_speed - cw_delay;
        if(delay > 0) usleep(delay * 1000);
        cw_off();
        cw_space_delay(1);
      }
    }
    else
    {
      state = 1;
      cw_memory[1] = cw_memory[0];
      while(1)
      {
        if(cw_memory[1] & (1 << state))
        {
          cw_memory[1] = 0;
          cw_on();
          cw_signal_delay(state);
          cw_off();
          cw_space_delay(1);
          cw_memory[1] &= ~(1 << state);
          cw_memory[1] |= cw_memory[0];
        }
        if(cw_memory[1])
        {
          state ^= 1;
        }
        else
        {
          if(cw_spacing)
          {
            state = 1;
            cw_space_delay(0);
            if(cw_memory[1]) continue;
          }
          break;
        }
      }
    }
  }

  return NULL;
}
