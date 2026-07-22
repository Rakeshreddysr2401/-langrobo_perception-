#!/usr/bin/env python3
"""LangRobo mobile teleop — hold-to-move web control -> /cmd_vel.

Open http://<pi5-ip>:8091 on a phone on the same wifi.

MODE switch on the page:
  * AUTO (default)  -> teleop is hands-off, publishes NOTHING, so nav2 / the
    brain own /cmd_vel and the rover explores/maps on its own. The always-shared
    link is safe to leave open: no button drives until you flip to MANUAL.
  * MANUAL          -> you drive. Switching to MANUAL hard-cancels any active
    nav2 goal (empty CancelGoal on navigate_to_pose / navigate_through_poses) so
    nav2 fully lets go — no /cmd_vel contention. Buttons publish a Twist while held; releasing
    (or losing the connection / backgrounding the page) stops the rover within
    ~0.4 s (dead-man), backed by the ESP32's own 500 ms /cmd_vel watchdog. While
    MANUAL is on the node publishes zeros when idle, holding the rover firmly in
    your control (overrides nav2).

Publishes DIRECTLY to /cmd_vel (bypasses safety_guard) — drive by sight. Speed
constants below are tunable (rover caps: vx<=0.22 m/s, wz<=0.90 rad/s).
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist

PORT = 8091
PUB_HZ = 10.0          # publish rate while MANUAL (feeds the 500 ms ESP32 watchdog)
CMD_TIMEOUT = 0.4      # s since last button ping before we treat it as released (dead-man)

# velocity presets (linear m/s, angular rad/s). +wz turns LEFT (CCW).
VX = 0.20
WZ = 0.70
# "slight" = a TIGHT forward turn: slow forward + strong rotation so the INNER
# wheel counter-rotates (goes backward) and the rover carves through tight spots
# instead of one wheel just creeping. Tune vx down / wz up for a tighter turn.
VX_SLIGHT = 0.10
WZ_SLIGHT = 0.75
MOVES = {
    'forward': (VX, 0.0),
    'back':    (-VX, 0.0),
    'left':    (0.0, WZ),          # pivot in place (both wheels counter-rotate)
    'right':   (0.0, -WZ),
    'slleft':  (VX_SLIGHT, WZ_SLIGHT),   # tight forward-left curve (inner wheel reverses)
    'slright': (VX_SLIGHT, -WZ_SLIGHT),
    'stop':    (0.0, 0.0),
}


class Mode:
    """MANUAL (you drive) vs AUTO (nav2/brain own /cmd_vel). Default AUTO."""
    def __init__(self):
        self.manual = False


class TeleopState:
    def __init__(self):
        self.lock = threading.Lock()
        self.vx = 0.0
        self.wz = 0.0
        self.t = 0.0

    def set(self, vx, wz):
        with self.lock:
            self.vx, self.wz, self.t = vx, wz, time.time()

    def fresh_cmd(self):
        """(vx, wz) — the held command, or zeros if the last ping is stale."""
        with self.lock:
            if time.time() - self.t <= CMD_TIMEOUT:
                return self.vx, self.wz
        return 0.0, 0.0


MODE = Mode()
STATE = TeleopState()
CANCEL_NAV = threading.Event()   # set when switching to MANUAL -> cancel active nav2 goal

# nav2 actions to cancel on takeover (empty CancelGoal request = cancel ALL goals)
NAV_CANCEL_SERVICES = (
    '/navigate_to_pose/_action/cancel_goal',
    '/navigate_through_poses/_action/cancel_goal',
)

HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>LangRobo Teleop</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; -webkit-touch-callout: none; }
  html, body { margin: 0; height: 100%; background: #16181d; color: #e6e6e6;
    font-family: system-ui, -apple-system, sans-serif; overscroll-behavior: none; touch-action: none; }
  .wrap { display: flex; flex-direction: column; align-items: center; padding: 14px; }
  h1 { font-size: 16px; font-weight: 600; margin: 6px 0 4px; color: #9aa4b2; }
  #status { font-size: 13px; color: #6b7280; height: 18px; margin-bottom: 10px; }
  #mode { width: min(94vw, 460px); border: 0; border-radius: 16px; padding: 18px 0;
    font-size: 18px; font-weight: 700; margin-bottom: 14px; color: #fff; }
  #mode.auto { background: #374151; }
  #mode.manual { background: #15803d; }
  .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;
    width: min(94vw, 460px); }
  .grid button { border: 0; border-radius: 16px; background: #262b34; color: #e6e6e6;
    font-size: 30px; padding: 26px 0; touch-action: none; transition: background .05s; }
  .grid button:active, .grid button.act { background: #3b82f6; color: #fff; }
  .stop { background: #7f1d1d !important; color: #fff; font-size: 20px; font-weight: 700; }
  .stop:active, .stop.act { background: #ef4444 !important; }
  .empty { visibility: hidden; }
  .grid.locked button:not(.stop) { opacity: .35; }
</style>
</head>
<body>
<div class="wrap">
  <h1>LangRobo Teleop</h1>
  <button id="mode" class="auto">AUTO &mdash; nav2 driving (tap to take control)</button>
  <div id="status">&nbsp;</div>
  <div class="grid locked" id="pad">
    <button data-dir="slleft">&#8598;</button>
    <button data-dir="forward">&#8593;</button>
    <button data-dir="slright">&#8599;</button>
    <button data-dir="left">&#8634;</button>
    <button class="stop" data-dir="stop">STOP</button>
    <button data-dir="right">&#8635;</button>
    <button class="empty"></button>
    <button data-dir="back">&#8595;</button>
    <button class="empty"></button>
  </div>
</div>
<script>
  const status = document.getElementById('status');
  const modeBtn = document.getElementById('mode');
  const pad = document.getElementById('pad');
  let timer = null, active = null, manual = false;

  function ping(dir) { fetch('/cmd?dir=' + dir, { method: 'POST' }).catch(() => {}); }

  function applyMode(m) {
    manual = m;
    if (manual) {
      modeBtn.className = 'manual';
      modeBtn.innerHTML = 'MANUAL &mdash; you drive (tap for AUTO)';
      pad.classList.remove('locked');
      status.textContent = 'you are in control';
    } else {
      release();
      modeBtn.className = 'auto';
      modeBtn.innerHTML = 'AUTO &mdash; nav2 driving (tap to take control)';
      pad.classList.add('locked');
      status.textContent = 'nav2 in control';
    }
  }
  function setMode(m) {
    fetch('/mode?manual=' + (m ? 'on' : 'off'), { method: 'POST' })
      .then(r => r.json()).then(j => applyMode(j.manual)).catch(() => {});
  }
  modeBtn.addEventListener('click', () => setMode(!manual));

  function press(btn, dir) {
    if (!manual || active === dir) return;
    release();
    active = dir; btn.classList.add('act'); status.textContent = dir;
    ping(dir); timer = setInterval(() => ping(dir), 150);
  }
  function release() {
    if (timer) { clearInterval(timer); timer = null; }
    if (active) {
      pad.querySelectorAll('button').forEach(b => b.classList.remove('act'));
      ping('stop'); active = null;
      if (manual) status.textContent = 'stop';
    }
  }

  pad.querySelectorAll('button[data-dir]').forEach(btn => {
    const dir = btn.dataset.dir;
    if (dir === 'stop') {
      btn.addEventListener('pointerdown', e => { e.preventDefault(); release(); });
      return;
    }
    btn.addEventListener('pointerdown', e => { e.preventDefault(); press(btn, dir); });
    btn.addEventListener('pointerup', e => { e.preventDefault(); release(); });
    btn.addEventListener('pointerleave', () => release());
    btn.addEventListener('pointercancel', () => release());
  });
  document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
  window.addEventListener('blur', release);
  window.addEventListener('pagehide', release);

  // sync mode on load (multiple phones stay consistent)
  fetch('/mode').then(r => r.json()).then(j => applyMode(j.manual)).catch(() => applyMode(false));
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='text/html; charset=utf-8'):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(b)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if self.path == '/' or self.path.startswith('/index'):
            self._send(200, HTML)
        elif self.path.startswith('/mode'):
            self._mode()
        elif self.path.startswith('/cmd'):
            self._cmd()
        else:
            self._send(404, 'not found', 'text/plain')

    def do_POST(self):
        if self.path.startswith('/mode'):
            self._mode()
        elif self.path.startswith('/cmd'):
            self._cmd()
        else:
            self._send(404, 'not found', 'text/plain')

    def _mode(self):
        q = parse_qs(urlparse(self.path).query)
        if 'manual' in q:
            MODE.manual = (q['manual'][0] == 'on')
            if MODE.manual:
                CANCEL_NAV.set()      # taking control -> cancel any active nav2 goal
            else:
                STATE.set(0.0, 0.0)
        self._send(200, json.dumps({'manual': MODE.manual}), 'application/json')

    def _cmd(self):
        q = parse_qs(urlparse(self.path).query)
        d = (q.get('dir') or ['stop'])[0]
        if not MODE.manual:
            self._send(200, json.dumps({'ok': False, 'reason': 'auto'}), 'application/json')
            return
        vx, wz = MOVES.get(d, (0.0, 0.0))
        STATE.set(vx, wz)
        self._send(200, json.dumps({'ok': True, 'dir': d, 'vx': vx, 'wz': wz}),
                   'application/json')

    def log_message(self, *a):
        pass


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_web')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    cancel_clients = [node.create_client(CancelGoal, s) for s in NAV_CANCEL_SERVICES]

    def tick():
        if CANCEL_NAV.is_set():          # taking manual control -> hard-cancel nav2
            CANCEL_NAV.clear()
            sent = [s for c, s in zip(cancel_clients, NAV_CANCEL_SERVICES)
                    if c.service_is_ready() and (c.call_async(CancelGoal.Request()) or True)]
            node.get_logger().info(
                f'MANUAL: nav2 goal-cancel sent to {sent}' if sent
                else 'MANUAL: no nav2 cancel service ready (nav2 down or no goal) — nothing to cancel')
        if not MODE.manual:
            return  # AUTO: nav2 / brain own /cmd_vel
        vx, wz = STATE.fresh_cmd()   # zeros when idle -> firm hold
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        pub.publish(msg)

    node.create_timer(1.0 / PUB_HZ, tick)
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'teleop web up on http://0.0.0.0:{PORT}  (default AUTO; flip to MANUAL to drive)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            pub.publish(Twist())
        except Exception:
            pass
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
