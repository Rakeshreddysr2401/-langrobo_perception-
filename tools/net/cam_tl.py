import collections, socket, struct, sys, time
secs = 4
NAMES = {'000100c2': 'PDP', '000003c2': 'EDP pubs', '000004c2': 'EDP subs', '000200c2': 'liveliness', '000300c3': 'TypeLookup req', '000301c3': 'TypeLookup rep'}
KIND = {0x06: 'ACKNACK', 0x07: 'HEARTBEAT', 0x08: 'GAP', 0x15: 'DATA', 0x16: 'DATA_FRAG'}
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003)); s.bind(('enP8p1s0', 0)); s.settimeout(0.5)
c = collections.Counter(); srcs = collections.Counter(); t0 = time.time()
while time.time() - t0 < secs:
    try: pkt, addr = s.recvfrom(65535)
    except socket.timeout: continue
    if pkt[12:14] != b'\x08\x00': continue
    ip = pkt[14:]
    if len(ip) < 28 or ip[9] != 17 or (struct.unpack('!H', ip[6:8])[0] & 0x1fff): continue
    d = 'Jetson->camera' if addr[2] == 4 else 'camera->Jetson'
    ihl = (ip[0] & 15) * 4; pl = ip[ihl + 8:]
    if addr[2] == 4: srcs[struct.unpack('!H', ip[ihl:ihl + 2])[0]] += 1
    if pl[:4] != b'RTPS': continue
    i = 20; lab = 'other'
    while i + 4 <= len(pl):
        k = pl[i]; ln = struct.unpack('<H' if pl[i + 1] & 1 else '>H', pl[i + 2:i + 4])[0]; body = pl[i + 4:]
        w = body[8:12].hex() if k in (0x15, 0x16) and len(body) >= 12 else body[4:8].hex() if k in (6, 7, 8) and len(body) >= 8 else None
        if w: lab = NAMES.get(w, 'user') + ' ' + KIND[k]; break
        if ln == 0: break
        i += 4 + ln
    c[(d, lab)] += 1
for k, v in sorted(c.items(), key=lambda x: -x[1])[:18]: print(f'{v/secs:8.0f}/s  {k[0]:15s} {k[1]}')
print('Jetson source ports talking to the camera side:', len(srcs))
