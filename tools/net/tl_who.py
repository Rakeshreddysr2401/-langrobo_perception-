import collections, socket, struct, subprocess, sys, time
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3
TL = {'000300c3': 'req', '000301c3': 'rep'}
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003)); s.bind(('lo', 0)); s.settimeout(0.5)
src, dst, pair = collections.Counter(), collections.Counter(), collections.Counter()
hb = collections.Counter()
t0 = time.time()
while time.time() - t0 < secs:
    try: pkt, addr = s.recvfrom(65535)
    except socket.timeout: continue
    if addr[2] != 4: continue
    ip = pkt[14:]
    if len(ip) < 28 or ip[9] != 17 or (struct.unpack('!H', ip[6:8])[0] & 0x1fff): continue
    ihl = (ip[0] & 15) * 4; sp, dp = struct.unpack('!HH', ip[ihl:ihl + 4]); pl = ip[ihl + 8:]
    if pl[:4] != b'RTPS': continue
    i = 20
    while i + 4 <= len(pl):
        k, le = pl[i], pl[i + 1] & 1; ln = struct.unpack('<H' if le else '>H', pl[i + 2:i + 4])[0]; body = pl[i + 4:]
        if k in (0x06, 0x07) and len(body) >= 8 and body[4:8].hex() in TL:
            kind = ('HB ' if k == 7 else 'ACK ') + TL[body[4:8].hex()]
            src[sp] += 1; dst[dp] += 1; pair[(kind, sp, dp)] += 1
            if k == 7 and len(body) >= 24:
                f = '<' if le else '>'
                first = struct.unpack(f + 'iI', body[8:16]); last = struct.unpack(f + 'iI', body[16:24])
                hb[(sp, dp, TL[body[4:8].hex()], first[1], last[1])] += 1
            break
        if ln == 0: break
        i += 4 + ln
own = {}
for line in subprocess.run(['ss', '-uanpH'], capture_output=True, text=True).stdout.splitlines():
    f = line.split()
    try: own[int(f[3].rsplit(':', 1)[1])] = f[-1].split('"')[1] + '/' + f[-1].split('pid=')[1].split(',')[0]
    except Exception: pass
cmd = {}
def name(port):
    o = own.get(port, '?(no socket)')
    pid = o.split('/')[-1]
    if pid.isdigit() and pid not in cmd:
        try: a = open(f'/proc/{pid}/cmdline').read().split('\0'); cmd[pid] = next((x.split('/')[-1] for x in a if x.endswith('.py')), a[0].split('/')[-1])
        except Exception: cmd[pid] = '?'
    return f'{port} {cmd.get(pid, o)}'
tot = sum(src.values())
print(f'{tot/secs:.0f} TypeLookup HB/ACK pkt/s; {len(src)} source ports, {len(dst)} destination ports')
print('top senders:'); [print(f'  {c/secs:7.0f}/s  {name(p)}') for p, c in src.most_common(12)]
print('top destinations:'); [print(f'  {c/secs:7.0f}/s  {name(p)}') for p, c in dst.most_common(12)]
print('top pairs:'); [print(f'  {c/secs:7.1f}/s  {k[0]:8s} {name(k[1])} -> {name(k[2])}') for k, c in pair.most_common(8)]
print('heartbeat seq ranges (src,dst,kind,first,last):'); [print(f'  {c/secs:6.1f}/s {k}') for k, c in hb.most_common(6)]
