#!/usr/bin/env python3
"""upload_main.py PORT main.py -- put main.py on a MicroPython board, reset it, show what it prints."""
import sys, time, serial


def opened(port):
    s = serial.Serial(); s.port, s.baudrate, s.timeout = port, 115200, 0.1
    s.dtr = False; s.rts = False; s.open()                 # no auto-reset / boot-mode on open
    return s


def until(s, token, secs):
    buf, end = b'', time.time() + secs
    while time.time() < end and token not in buf:
        buf += s.read(4096)
    return buf


def raw_exec(s, code, secs=5):
    """Run code in the raw REPL; -> (stdout, stderr) as the board reports them."""
    s.write(code.encode() + b'\x04')
    got = until(s, b'\x04>', secs)                          # OK <out> \x04 <err> \x04 >
    if not got.startswith(b'OK'):
        raise RuntimeError(f'board did not accept the code: {got[:200]!r}')
    out, err = (got[2:].split(b'\x04') + [b'', b''])[:2]
    return out.decode(errors='replace').strip(), err.decode(errors='replace').strip()


port, src = sys.argv[1], open(sys.argv[2]).read()
s = opened(port)
for _ in range(5):                                          # stop whatever runs, enter raw REPL
    s.write(b'\r\x03\x03'); time.sleep(0.3); s.read(100000)
    s.write(b'\x01')
    if b'raw REPL' in until(s, b'raw REPL; CTRL-B to exit\r\n>', 1.5):
        break
else:
    sys.exit('could not get the raw REPL')
out, err = raw_exec(s, f"f = open('main.py', 'w'); n = f.write({src!r}); f.close(); print(n)")
print('wrote', out, 'bytes', ('ERROR ' + err) if err else '')
out, err = raw_exec(s, "import os; print(os.listdir()); print(open('main.py').read()[:60])")
print('board files:', out.splitlines()[0], '| main.py starts:', out.splitlines()[1] if len(out.splitlines()) > 1 else '', err)
s.write(b'import machine; machine.reset()\x04'); time.sleep(0.3); s.close()
time.sleep(3.0)                                             # it restarts and runs main.py by itself
s = opened(port); buf = until(s, b'never', 2.0); s.close()
print('--- after the reset, printed by itself:')
print('\n'.join(buf.decode(errors='replace').splitlines()[-10:]))
