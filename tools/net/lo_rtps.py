import collections, socket, struct, sys, time
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 2
LO, HI = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (0, 65535)
KIND = {0x06: 'ACKNACK', 0x07: 'HEARTBEAT', 0x08: 'GAP', 0x09: 'INFO_TS', 0x0e: 'INFO_DST', 0x12: 'NACK_FRAG', 0x13: 'HB_FRAG', 0x15: 'DATA', 0x16: 'DATA_FRAG'}
BUILTIN = {'000100c2': 'PDP', '000003c2': 'EDP pubs', '000004c2': 'EDP subs', '000200c2': 'liveliness', '000300c3': 'TypeLookup request', '000301c3': 'TypeLookup reply'}
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003)); s.bind(('lo', 0)); s.settimeout(0.5)
n = b = 0; cls, clsb = collections.Counter(), collections.Counter()
t0 = time.time()
while time.time() - t0 < secs:
    try: pkt, addr = s.recvfrom(65535)
    except socket.timeout: continue
    if addr[2] != 4: continue
    ip = pkt[14:]
    if len(ip) < 28 or ip[9] != 17: continue
    if struct.unpack('!H', ip[6:8])[0] & 0x1fff: cls['(later IP fragment of a big message)'] += 1; clsb['(later IP fragment of a big message)'] += len(ip); n += 1; b += len(ip); continue
    ihl = (ip[0] & 15) * 4; pl = ip[ihl + 8:]
    dp = struct.unpack("!H", ip[ihl + 2:ihl + 4])[0]
    if not LO <= dp <= HI: continue
    n += 1; b += len(ip)
    if pl[:4] != b'RTPS': cls['non-RTPS'] += 1; continue
    i = 20; label = None
    while i + 4 <= len(pl):
        k, ln = pl[i], struct.unpack('<H' if pl[i + 1] & 1 else '>H', pl[i + 2:i + 4])[0]
        body = pl[i + 4:]
        if k in (0x15, 0x16) and len(body) >= 12: w = body[8:12].hex()
        elif k in (0x06, 0x07, 0x08, 0x12, 0x13) and len(body) >= 8: w = body[4:8].hex()
        else: w = None
        if w is not None:
            label = (BUILTIN.get(w, 'user topic') + ' ' + KIND[k]); break
        if ln == 0: break
        i += 4 + ln
    cls[label or 'other'] += 1; clsb[label or 'other'] += len(ip)
print(f'{n/secs:.0f} pkt/s {b/secs/1e6:.1f} MB/s on lo')
for k, c in cls.most_common(16): print(f'{c/secs:9.0f} pkt/s {clsb[k]/secs/1e6:7.2f} MB/s  {k}')
