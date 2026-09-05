#!/usr/bin/env python3
"""Exercises the protocol 2 server the way machpsdr's protocol2.c does."""
import socket, struct, time, math, sys, collections

HOST = "127.0.0.1"
P_GEN, P_RXSPEC, P_TXSPEC, P_HP, P_AUDIO, P_TXIQ, P_IQ0 = 1024,1025,1026,1027,1028,1029,1035
fails, checks = [], []

def check(name, ok, detail=""):
    checks.append((name, ok, detail))
    if not ok: fails.append(name)
    print(("  ok   " if ok else "  FAIL ") + name + (("  -- " + detail) if detail else ""))

def phase_word(hz):
    return int(4294967296.0 * hz / 122880000.0) & 0xffffffff

# ---- discovery ------------------------------------------------------------
d = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
d.settimeout(2.0)
d.sendto(b"\x00\x00\x00\x00\x02" + bytes(55), (HOST, P_GEN))
try:
    rep, _ = d.recvfrom(2048)
except socket.timeout:
    print("no discovery reply"); sys.exit(1)
check("discovery reply is 60 bytes", len(rep) == 60, str(len(rep)))
check("discovery status is idle", rep[4] == 2, hex(rep[4]))
check("discovery board is Hermes", rep[11] == 1, str(rep[11]))
check("discovery advertises 2 DDCs", rep[20] == 2, str(rep[20]))
check("discovery reports a MAC", rep[5:11] != bytes(6), rep[5:11].hex())
d.close()

# ---- one socket for everything, as the client does ------------------------
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind(("127.0.0.1", 0))
seq = collections.Counter()

def send(port, payload):
    b = bytearray(payload)
    b[0:4] = struct.pack(">I", seq[port]); seq[port] += 1
    s.sendto(bytes(b), (HOST, port))

def general():
    b = bytearray(60)
    b[37] = 0x08   # phase words, not Hz
    b[38] = 0x01
    b[58] = 0x01
    b[59] = 0x01
    send(P_GEN, b)

def rxspec(enable, rate, adc=0):
    b = bytearray(1444)
    b[4] = 2
    b[7] = enable
    for ddc in range(8):
        b[17 + 6*ddc] = adc
        b[18 + 6*ddc], b[19 + 6*ddc] = divmod(rate // 1000, 256)
        b[22 + 6*ddc] = 24
    send(P_RXSPEC, b)

def txspec():
    b = bytearray(60)
    b[4] = 1
    b[9] = 20      # keyer speed
    b[10] = 50
    send(P_TXSPEC, b)

def highprio(run, ptt=0, rxhz=(7100000, 14100000, 21100000), txhz=7100000, drive=128):
    b = bytearray(1444)
    b[4] = (1 if run else 0) | (2 if ptt else 0)
    for i, hz in enumerate(rxhz):
        b[9 + 4*i:13 + 4*i] = struct.pack(">I", phase_word(hz))
    b[329:333] = struct.pack(">I", phase_word(txhz))
    b[345] = drive
    b[1432:1436] = struct.pack(">I", 0x01000040)  # TX ANT1, 1.5 MHz HPF
    b[1443] = 0
    send(P_HP, b)

def collect(seconds, feed_tx=False):
    """Drain everything the server sends for `seconds`."""
    got = collections.defaultdict(list)
    end = time.time() + seconds
    s.settimeout(0.05)
    txseq = 0
    audioseq = 0
    next_tx = time.time()
    while time.time() < end:
        if feed_tx:
            now = time.time()
            while next_tx < now:
                iq = bytearray(1444)
                iq[0:4] = struct.pack(">I", txseq); txseq += 1
                s.sendto(bytes(iq), (HOST, P_TXIQ))
                au = bytearray(260)
                au[0:4] = struct.pack(">I", audioseq); audioseq += 1
                s.sendto(bytes(au), (HOST, P_AUDIO))
                next_tx += 240 / 192000.0
        try:
            pkt, addr = s.recvfrom(4096)
        except socket.timeout:
            continue
        got[addr[1]].append(pkt)
    return got

def sample24(b, o):
    v = int.from_bytes(b[o:o+3], "big", signed=True)
    return v / 8388607.0

def tone_hz(pkt, rate):
    """Angle advance per sample of the I/Q pair carried in this packet."""
    n = struct.unpack(">H", pkt[14:16])[0]
    acc, cnt = 0.0, 0
    prev = None
    for i in range(min(n, 64)):
        o = 16 + 6*i
        # the wire order is (I, Q) exactly as protocol 1 delivers it
        z = complex(sample24(pkt, o+3), sample24(pkt, o))
        if prev is not None:
            d = z * prev.conjugate()
            acc += math.atan2(d.imag, d.real); cnt += 1
        prev = z
    return acc / cnt * rate / (2*math.pi) if cnt else 0.0

# ---- start, 192 kHz, DDC0 and DDC1 ---------------------------------------
print("\n192 kHz, DDC0 + DDC1")
general(); rxspec(0b011, 192000); txspec(); highprio(True)
time.sleep(0.3)
collect(0.2)                       # discard the start transient
got = collect(1.0)

for ddc in (0, 1):
    port = P_IQ0 + ddc
    pkts = got.get(port, [])
    rate = len(pkts) * 238
    check(f"DDC{ddc} streams on port {port}", len(pkts) > 0, f"{len(pkts)} packets")
    if not pkts: continue
    check(f"DDC{ddc} packets are 1444 bytes", all(len(p) == 1444 for p in pkts))
    check(f"DDC{ddc} header says 24 bits, 238 samples",
          all(p[12:16] == b"\x00\x18\x00\xee" for p in pkts), pkts[0][12:16].hex())
    s0 = struct.unpack(">I", pkts[0][0:4])[0]
    contiguous = all(struct.unpack(">I", p[0:4])[0] == s0 + i for i, p in enumerate(pkts))
    check(f"DDC{ddc} sequence is contiguous", contiguous)
    check(f"DDC{ddc} sample rate is 192 kHz", abs(rate - 192000) < 192000*0.10,
          f"{rate} samples/s")
    f = tone_hz(pkts[len(pkts)//2], 192000)
    want = (ddc + 1) * 1000.0
    check(f"DDC{ddc} carries its own channel ({want:.0f} Hz)", abs(f - want) < 50, f"{f:.1f} Hz")

check("nothing on the port of a DDC the board does not have",
      len(got.get(P_IQ0+2, [])) == 0, f"{len(got.get(P_IQ0+2, []))} packets")

mic = got.get(P_TXSPEC, [])
check("mic stream runs at 48 kHz", abs(len(mic)*64 - 48000) < 48000*0.15, f"{len(mic)*64} samples/s")
check("mic packets are 132 bytes", all(len(p) == 132 for p in mic) and len(mic) > 0)

hp = got.get(P_RXSPEC, [])
check("high priority status arrives", len(hp) >= 15, f"{len(hp)} packets/s")
check("high priority status is 60 bytes", all(len(p) == 60 for p in hp) and len(hp) > 0)
if hp:
    check("status reports PLL locked, PTT down", hp[0][4] == 0x08, hex(hp[0][4]))

# ---- rate change ----------------------------------------------------------
for rate in (48000, 768000, 1536000, 3072000, 6144000):
    print(f"\n{rate//1000} kHz, DDC0 only")
    rxspec(0b001, rate); highprio(True)
    time.sleep(0.4)
    collect(0.3)
    got = collect(1.0)
    pkts = got.get(P_IQ0, [])
    measured = len(pkts) * 238
    check(f"{rate//1000} kHz sample rate", abs(measured - rate) < rate*0.12,
          f"{measured} samples/s")
    if pkts:
        f = tone_hz(pkts[len(pkts)//2], rate)
        check(f"{rate//1000} kHz tone still on channel 0", abs(f - 1000) < 100, f"{f:.1f} Hz")
    check(f"{rate//1000} kHz DDC1 silent while disabled", len(got.get(P_IQ0+1, [])) == 0,
          f"{len(got.get(P_IQ0+1, []))} packets")

# ---- transmit and audio ---------------------------------------------------
print("\ntransmit: PTT with TX I/Q and RX audio")
rxspec(0b001, 192000); highprio(True, ptt=1)
time.sleep(0.3)
got = collect(1.0, feed_tx=True)
check("still streaming while transmitting", len(got.get(P_IQ0, [])) > 100,
      f"{len(got.get(P_IQ0, []))} packets")
hp = got.get(P_RXSPEC, [])
check("status speeds up under PTT", len(hp) > 100, f"{len(hp)} packets/s")

# ---- stop -----------------------------------------------------------------
print("\nstop")
highprio(False)
collect(0.3)   # drain what was already in flight
got = collect(0.5)
check("DDC stops on run=0", len(got.get(P_IQ0, [])) == 0, f"{len(got.get(P_IQ0, []))} packets")

print()
print(f"{len(checks)-len(fails)}/{len(checks)} checks passed")
sys.exit(1 if fails else 0)
