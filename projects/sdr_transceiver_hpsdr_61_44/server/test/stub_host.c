/*
Runs the protocol 2 server on a development machine with the gateware
replaced by ordinary memory, so the network side can be exercised against a
test client.  The server source is included verbatim: nothing here is
compiled into the board's binary.

The fake hardware models the parts the protocol code actually reads back:
the DMA ring and its burst counter (filled with a per-channel tone), the
transmit and codec FIFO fill levels, and the GPIO inputs.
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <termios.h>
#include <poll.h>

/* Linux bits macOS does not have */
struct mmsghdr { struct msghdr msg_hdr; unsigned int msg_len; };
static int sendmmsg(int fd, struct mmsghdr *msgs, unsigned int n, int flags)
{
  unsigned int i;
  for(i = 0; i < n; ++i)
  {
    ssize_t rc = sendmsg(fd, &msgs[i].msg_hdr, flags);
    if(rc < 0) return i ? (int)i : -1;
    msgs[i].msg_len = rc;
  }
  return (int)n;
}
#ifndef TIMER_ABSTIME
#define TIMER_ABSTIME 1
#endif
static int clock_nanosleep(clockid_t id, int flags, const struct timespec *req,
  struct timespec *rem)
{
  struct timespec now, delta;
  clock_gettime(id, &now);
  delta.tv_sec = req->tv_sec - now.tv_sec;
  delta.tv_nsec = req->tv_nsec - now.tv_nsec;
  if(delta.tv_nsec < 0) { delta.tv_nsec += 1000000000; --delta.tv_sec; }
  if(delta.tv_sec < 0) return 0;
  return nanosleep(&delta, rem);
}
#ifndef SIOCGIFHWADDR
#define SIOCGIFHWADDR 0x8927
#endif

#define FAKE_MEM_FD 900
#define FAKE_CMA_FD 901

static void *fake_cma;
static uint32_t fake_cma_phys = 0x0e000000;

static int stub_open(const char *path, int flags, ...)
{
  if(!strcmp(path, "/dev/mem")) return FAKE_MEM_FD;
  if(!strcmp(path, "/dev/cma")) return FAKE_CMA_FD;
  errno = ENOENT;
  return -1; /* no i2c, no uart */
}

static int stub_close(int fd)
{
  if(fd == FAKE_MEM_FD || fd == FAKE_CMA_FD) return 0;
  return close(fd);
}

static void *stub_mmap(void *addr, size_t len, int prot, int flags, int fd, off_t off)
{
  if(fd == FAKE_CMA_FD) return fake_cma;
  if(fd == FAKE_MEM_FD) return calloc(1, len < 65536 ? 65536 : len);
  return mmap(addr, len, prot, flags, fd, off);
}

static int stub_ioctl(int fd, unsigned long request, ...)
{
  va_list ap;
  void *arg;
  va_start(ap, request);
  arg = va_arg(ap, void *);
  va_end(ap);

  if(fd == FAKE_CMA_FD && (request & 0xff) == 0)
  {
    *(uint32_t *)arg = fake_cma_phys;
    return 0;
  }
  if(request == SIOCGIFHWADDR)
  {
    struct ifreq *r = arg;
    static const char m[6] = {0x00, 0x1c, 0xc0, 0xa2, 0x20, 0x02};
    memcpy(r->ifr_addr.sa_data, m, 6);
    return 0;
  }
  return -1;
}

#define open stub_open
#define close stub_close
#define mmap stub_mmap
#define ioctl stub_ioctl
#define main server_main
#include "sdr-transceiver-hpsdr.c"
#undef main
#undef open
#undef close
#undef mmap
#undef ioctl

/* The gateware: fills the DMA ring at the configured rate and keeps the
   transmit FIFOs at a plausible level.  Channel n carries a tone at
   (n + 1) * 1000 Hz so a packet delivered to the wrong DDC is visible. */
static void *fake_gateware(void *arg)
{
  double phase[DDC_COUNT] = {0};
  int rate_seen = 0;
  long long t0 = 0;
  uint64_t bursts = 0, due;
  uint32_t set = 0;
  int c, i;
  uint8_t *p;
  struct timeval tv;
  long long now;

  while(!ram) usleep(1000);

  while(1)
  {
    gettimeofday(&tv, NULL);
    now = (long long)tv.tv_sec * 1000000LL + tv.tv_usec;

    if(!(*rx_rst & 1) || rate_seen != rate_hz)
    {
      set = 0;
      bursts = 0;
      t0 = now;
      rate_seen = rate_hz;
      *rx_cntr = 0;
      if(!(*rx_rst & 1)) { usleep(200); continue; }
    }

    /* one burst is 128 bytes worth of sample sets */
    due = (uint64_t)((double)(now - t0) * 1e-6 * (double)rate_hz / (double)(128 / RX_SET_BYTES));

    if(bursts >= due) { usleep(200); continue; }

    while(bursts < due)
    {
      for(i = 0; i < 128 / RX_SET_BYTES; ++i)
      {
        p = ram + (size_t)(set & (RX_SETS - 1)) * RX_SET_BYTES;
        memset(p, 0, RX_SET_BYTES);
        for(c = 0; c < DDC_COUNT; ++c)
        {
          int32_t si = (int32_t)(8388607.0 * 0.4 * sin(phase[c]));
          int32_t sq = (int32_t)(8388607.0 * 0.4 * cos(phase[c]));
          phase[c] += 2.0 * M_PI * (c + 1) * 1000.0 / (double)rate_hz;
          if(phase[c] > M_PI) phase[c] -= 2.0 * M_PI;
          p[c * 6 + 0] = si >> 16; p[c * 6 + 1] = si >> 8; p[c * 6 + 2] = si;
          p[c * 6 + 3] = sq >> 16; p[c * 6 + 4] = sq >> 8; p[c * 6 + 5] = sq;
        }
        ++set;
      }
      ++bursts;
      *rx_cntr = (uint16_t)((set / (128 / RX_SET_BYTES)) & (RX_BURSTS - 1));
    }

    *tx_cntr = 2048;
    *dac_cntr = 1024;
    *adc_cntr = 0;
  }

  return NULL;
}

int main(int argc, char *argv[])
{
  pthread_t t;
  fake_cma = calloc(1, RX_RAM_SIZE);
  pthread_create(&t, NULL, fake_gateware, NULL);
  return server_main(argc, argv);
}
