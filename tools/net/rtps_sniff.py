#!/usr/bin/env python3
"""Who sends what to a host, by RTPS submessage.  sudo python3 rtps_sniff.py IFACE DST_IP [seconds]
Counts outgoing UDP packets on IFACE to DST_IP: per source port (with its process), per
destination port, per RTPS submessage kind and writer/reader entity id. Read-only."""
import collections, socket, struct, subprocess, sys, time
iface, dst, secs = sys.argv[1], socket.inet_aton(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 5
KIND = {0x06: 'ACKNACK', 0x07: 'HEARTBEAT', 0x08: 'GAP', 0x09: 'INFO_TS', 0x0e: 'INFO_DST', 0x12: 'NACK_FRAG',
        0x13: 'HB_FRAG', 0x15: 'DATA', 0x16: 'DATA_FRAG', 0x0c: 'INFO_SRC', 0x01: 'PAD'}
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003)); s.bind((iface, 0)); s.settimeout(0.5)
n = b = 0
sport, dport, kinds, ent, pair = (collections.Counter() for _ in range(5))
bytes_sport = collections.Counter()
t0 = time.time()
while time.time() - t0 < secs:
    try: pkt, addr = s.recvfrom(65535)
    except socket.timeout: continue
    if addr[2] != 4 or pkt[12:14] != b"\x08\x00": continue                      # PACKET_OUTGOING only
    ip = pkt[14:]
    if len(ip) < 28 or ip[9] != 17 or ip[16:20] != dst: continue
    ihl = (ip[0] & 15) * 4
    if (struct.unpack('!H', ip[6:8])[0] & 0x1fff): continue      # later IP fragments
    sp, dp = struct.unpack('!HH', ip[ihl:ihl + 4]); pl = ip[ihl + 8:]
    n += 1; b += len(pkt); sport[sp] += 1; dport[dp] += 1; bytes_sport[sp] += len(pkt); pair[(sp, dp)] += 1
    if pl[:4] != b'RTPS': kinds['non-RTPS'] += 1; continue
    i = 20
    while i + 4 <= len(pl):
        k, fl, ln = pl[i], pl[i + 1], struct.unpack('<H' if pl[i + 1] & 1 else '>H', pl[i + 2:i + 4])[0]
        name = KIND.get(k, hex(k)); kinds[name] += 1
        body = pl[i + 4:]
        if k in (0x15, 0x16) and len(body) >= 12: ent[(name, sp, 'w=' + body[8:12].hex(), 'r=' + body[4:8].hex())] += 1
        elif k in (0x06, 0x07, 0x08, 0x12, 0x13) and len(body) >= 8: ent[(name, sp, 'w=' + body[4:8].hex(), 'r=' + body[0:4].hex())] += 1
        if ln == 0: break
        i += 4 + ln
own = {}
for line in subprocess.run(['ss', '-uanpH'], capture_output=True, text=True).stdout.splitlines():
    f = line.split()
    try: own[int(f[3].rsplit(':', 1)[1])] = f[-1].split('"')[1] + ' pid=' + f[-1].split('pid=')[1].split(',')[0]
    except Exception: pass
print(f'{n/secs:.0f} pkt/s  {b/secs/1e6:.2f} MB/s  to {len(dport)} ports, from {len(sport)} source ports')
print('by source port:'); [print(f'  {c/secs:8.0f}/s {bytes_sport[p]/secs/1e3:8.0f} kB/s  :{p}  {own.get(p, "?")}') for p, c in sport.most_common(8)]
print('by destination port:'); [print(f'  {c/secs:8.0f}/s  :{p}') for p, c in dport.most_common(8)]
print('by submessage:'); [print(f'  {c/secs:8.0f}/s  {k}') for k, c in kinds.most_common()]
print('by entity:'); [print(f'  {c/secs:8.0f}/s  {k}') for k, c in ent.most_common(14)]
