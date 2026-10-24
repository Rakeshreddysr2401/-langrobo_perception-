#!/usr/bin/env python3
"""Watches the rover's layers, serves one health page, and (only if allowed)
restarts the layer that died.

    python3 ops/rover_watchdog.py            # observe: log + serve, touch nothing
    ROVER_WATCHDOG_ACT=1 python3 ...         # also restart a dead layer

  http://<jetson>:8099/health   JSON: every layer ok/low/down, restarts, moving
  http://<jetson>:8099/         the same as a small page
  logs/watchdog.jsonl           one JSON line per check that changed anything

WHY OBSERVE BY DEFAULT. A watchdog that restarts things fights whoever is
debugging them. It acts only when ROVER_WATCHDOG_ACT=1, never while
~/.rover_watchdog_pause exists (`./rover watchdog pause` / `resume`), never
while the rover is driving (a non-zero /cmd_vel), and at most MAX_RESTARTS per
layer per RESTART_WINDOW_S, so a layer that cannot start stays down and loud
instead of restart-looping. It restarts the LOWEST failed layer in bring-up
order (camera before pose before fused ...), because a dead camera also makes
everything above it look dead.

It does not restart the container, the Pi 5, or Studio: those are `./rover up`.
"""
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LOG = REPO / 'logs' / 'watchdog.jsonl'
PAUSE = Path.home() / '.rover_watchdog_pause'
PORT = int(os.environ.get('ROVER_HEALTH_PORT', '8099'))
ACT = os.environ.get('ROVER_WATCHDOG_ACT') == '1'
PERIOD_S = 30                   # each probe subscribes to the camera streams: not free on a loaded Jetson
DOWN_CHECKS = 3                 # consecutive bad probes before a layer counts as down
MAX_RESTARTS, RESTART_WINDOW_S = 2, 900
PI5 = os.environ.get('PI5_HOST', '192.168.1.16')
CAM = '/camera/camera0'

# bring-up order; (layer, topic, minimum Hz). Same numbers as ./rover status.
CHECKS = [
    ('camera', f'{CAM}/infra1/image_rect_raw', 15),
    ('camera', f'{CAM}/depth/image_rect_raw', 5),      # status wants 10; 5 is "alive and useful"
    ('lidar', '/scan', 8),
    ('pose', '/vo/odom', 10),
    ('pose', '/gyro/base', 50),
    ('wheels', '/wheel_state', 15),
    ('fused', '/odom', 15),
    ('slam', '/map', 0.3),
    ('map', '/ghost_clear/grid', 1),
    ('map', '/depth_gate/passed', 3),
    ('nav', '/global_costmap/costmap', 0.4),
    ('nav', '/local_costmap/costmap', 0.8),
    ('vlm', '/camera/color/image_raw/compressed', 1.5),
]
ORDER = ['camera', 'lidar', 'pose', 'fused', 'slam', 'map', 'nav', 'vlm']   # wheels: the ESP32, not restartable here
ENV = {'pose': {'SLAM': 'false'}}     # `up` brings pose up with SLAM=false: slam_toolbox owns map->odom

state = {'t': 0, 'layers': {}, 'moving': False, 'container': False, 'pi5': {}, 'restarts': {},
         'act': ACT, 'paused': False, 'error': ''}
history = {}       # layer -> [restart times]
bad_streak = {}    # layer -> consecutive bad probes
lock = threading.Lock()


def log(event, **kw):
    LOG.parent.mkdir(exist_ok=True)
    with open(LOG, 'a') as f:
        f.write(json.dumps({'t': round(time.time(), 1), 'event': event, **kw}) + '\n')


def probe():
    cmd = ['docker', 'exec', 'rover', 'bash', '-lc',
           'unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=0; '
           'for f in /opt/ros/*/setup.bash; do source "$f"; done; python3 /opt/rover3/tools/health_probe.py 6']
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return json.loads(out.stdout.strip().splitlines()[-1])


def container_up():
    r = subprocess.run(['docker', 'inspect', '-f', '{{.State.Running}}', 'rover'],
                       capture_output=True, text=True)
    return r.stdout.strip() == 'true'


def pi5_status():
    """The Pi 5's half, cheaply, over HTTP (teleop page); None = unreachable."""
    out = subprocess.run(['curl', '-s', '-m', '4', f'http://{PI5}:8091/mode'],
                         capture_output=True, text=True).stdout.strip()
    try:
        return {'reachable': True, 'manual': bool(json.loads(out).get('manual'))}
    except ValueError:
        return {'reachable': False, 'manual': None}


def judge(hz):
    layers = {}
    for layer, topic, want in CHECKS:
        r = hz.get(topic, 0.0)
        v = 'ok' if r >= want else ('low' if r > 0 else 'down')
        row = layers.setdefault(layer, {'status': 'ok', 'topics': {}})
        row['topics'][topic] = {'hz': r, 'want': want, 'status': v}
        if v == 'down' or (v == 'low' and row['status'] == 'ok'):
            row['status'] = v
    return layers


def restart(layer):
    now = time.time()
    h = [t for t in history.get(layer, []) if now - t < RESTART_WINDOW_S]
    if len(h) >= MAX_RESTARTS:
        log('restart_refused', layer=layer, why=f'{len(h)} restarts in {RESTART_WINDOW_S}s: leaving it down')
        return False
    history[layer] = h + [now]
    env = dict(os.environ, **ENV.get(layer, {}))
    log('restart', layer=layer)
    r = subprocess.run([str(REPO / 'rover'), layer], capture_output=True, text=True, env=env, timeout=300)
    log('restart_done', layer=layer, rc=r.returncode, tail=r.stdout.strip().splitlines()[-3:])
    return r.returncode == 0


def step():
    with lock:
        state['paused'] = PAUSE.exists()
    if not container_up():
        with lock:
            state.update(t=time.time(), container=False, layers={}, error='container "rover" is not running (./rover up)')
        return
    try:
        p = probe()
    except Exception as e:   # noqa: BLE001 - a failed probe is a finding, not a crash
        with lock:
            state.update(t=time.time(), container=True, error=f'probe failed: {e}')
        return
    layers = judge(p['hz'])
    pi5 = pi5_status()
    prev = {k: v['status'] for k, v in state['layers'].items()}
    for layer, row in layers.items():
        bad_streak[layer] = bad_streak.get(layer, 0) + 1 if row['status'] == 'down' else 0
        row['restarts'] = len([t for t in history.get(layer, []) if time.time() - t < RESTART_WINDOW_S])
    with lock:
        state.update(t=time.time(), container=True, layers=layers, moving=p['moving'], pi5=pi5, error='')
        state['restarts'] = {k: len(v) for k, v in history.items()}
    if {k: v['status'] for k, v in layers.items()} != prev:
        log('change', layers={k: v['status'] for k, v in layers.items()}, moving=p['moving'], pi5=pi5)
    if not ACT or PAUSE.exists() or p['moving']:
        return
    for layer in ORDER:
        if bad_streak.get(layer, 0) >= DOWN_CHECKS:
            restart(layer)
            bad_streak[layer] = 0
            break                      # one layer per cycle; the next probe judges the result


PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Kino health</title><body style="font:16px system-ui;margin:16px;max-width:42em">
<h1>Kino health</h1><pre id=o>loading…</pre>
<script>async function f(){try{const r=await (await fetch('/health')).json();
let s=(r.container?'':'CONTAINER DOWN\\n')+(r.error?r.error+'\\n':'')+
'mode: '+(r.act?'acts on failures':'observe only')+(r.paused?' (PAUSED)':'')+(r.moving?'  — driving':'')+
'\\nPi 5: '+(r.pi5.reachable?(r.pi5.manual?'teleop MANUAL':'teleop AUTO'):'unreachable')+'\\n\\n';
for(const [k,v] of Object.entries(r.layers))s+=k.padEnd(8)+v.status.toUpperCase().padEnd(6)+Object.entries(v.topics).map(([t,x])=>t.split('/').pop()+' '+x.hz).join('  ')+'\\n';
s+='\\nage '+Math.round(Date.now()/1000-r.t)+' s';o.textContent=s}catch(e){o.textContent='health service unreachable'}}
f();setInterval(f,5000)</script>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        with lock:
            body = json.dumps(state).encode()
        if self.path.startswith('/health'):
            ctype, data = 'application/json', body
        else:
            ctype, data = 'text/html; charset=utf-8', PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


def main():
    threading.Thread(target=ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever, daemon=True).start()
    log('start', act=ACT, port=PORT)
    while True:
        t0 = time.time()
        try:
            step()
        except Exception as e:   # noqa: BLE001 - the watchdog must outlive its own bugs
            log('error', msg=repr(e))
        time.sleep(max(1, PERIOD_S - (time.time() - t0)))


if __name__ == '__main__':
    main()
