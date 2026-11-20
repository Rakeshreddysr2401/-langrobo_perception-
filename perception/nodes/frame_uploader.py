#!/usr/bin/env python3
"""frame_uploader -- send every kept keyframe (JPEG + metadata, NOT the depth) to the Mac's frame memory.

Frame memory step 2 (Pi 5 ~/ros2_ws docs/plans/FRAME_MEMORY.md). frame_capture writes
/logs/frames/<origin>/<id>.{jpg,depth.png,json}; this posts each complete one (the .json is written last)
to $MEMORY_URL/ingest (the Mac's kino_memory, started there by `kmem`) and marks it <id>.sent.
The depth stays here: pixel_to_goal grounds a box on it.

The Mac may be off or asleep: frames then wait on disk and go when it is back (oldest first),
so nothing the robot saw is lost. No ROS, stdlib only.

    MEMORY_URL   e.g. http://192.168.1.10:8094  (./rover vlm resolves the Mac's mDNS name on the host:
                 the container has no mDNS)
"""
import base64
import glob
import json
import os
import sys
import time
import urllib.request

ROOT = os.environ.get('FRAMES_DIR', '/logs/frames')
URL = os.environ.get('MEMORY_URL', '').rstrip('/')
RETRY_S = 10.0


def pending():
    out = []
    for j in glob.glob(os.path.join(ROOT, '*', '*.json')):
        base = j[:-5]
        if not os.path.exists(base + '.sent'):
            out.append(base)
    out.sort(key=lambda b: int(os.path.basename(b)) if os.path.basename(b).isdigit() else 0)
    return out


def send(base):
    meta = json.load(open(base + '.json'))
    body = {'meta': meta, 'jpeg': base64.b64encode(open(base + '.jpg', 'rb').read()).decode()}
    r = urllib.request.Request(URL + '/ingest', json.dumps(body).encode(), {'Content-Type': 'application/json'})
    res = json.load(urllib.request.urlopen(r, timeout=20))
    if not res.get('ok'):
        raise RuntimeError(res)
    open(base + '.sent', 'w').close()


def main():
    if not URL:
        print('[frame_uploader] MEMORY_URL not set: nothing to send to', flush=True)
        sys.exit(1)
    print(f'[frame_uploader] {ROOT} -> {URL}', flush=True)
    sent, last_log, down_since = 0, 0.0, None
    while True:
        todo = pending()
        for base in todo:
            try:
                send(base)
                sent += 1
                if down_since:
                    print(f'[frame_uploader] memory back after {time.time() - down_since:.0f} s', flush=True)
                    down_since = None
            except Exception as e:
                if down_since is None:
                    down_since = time.time()
                    print(f'[frame_uploader] memory not reachable ({e}); keeping frames on disk', flush=True)
                break
        if time.time() - last_log > 60:
            print(f'[frame_uploader] sent {sent}, waiting {len(pending())}', flush=True)
            last_log = time.time()
        time.sleep(RETRY_S if down_since else 1.0)


if __name__ == '__main__':
    main()
