#!/usr/bin/env python3
"""ground_check.py — where would "go near <thing>" aim? Asks, never drives.

    python3 /opt/rover4/tools/ground_check.py "the door" [--vlm http://HOST:8080]

Takes the newest colour frame, asks the VLM for the thing's box with the
Pi 5 brain's own prompt (approach._VLM_LOCATE_PROMPT), sends that box to
pixel_to_goal exactly as ros2_bridge.ground_pixel does, and prints the reply:
object point, goal, depth, which slab was used and what it passed over.
Saves the photo with the box drawn to /tmp/ground_check.jpg.

pixel_to_goal only answers; nothing here publishes a goal. NAV_PLAN.md D1.
"""
import argparse
import base64
import json
import re
import time
import urllib.request
import uuid

import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PROMPT = (
    'Look at this image. Find: "{description}". It counts even if it is only '
    "partly visible, cut off at the edge of the image, or seen from an unusual "
    "angle; colours may look different under indoor light. "
    "Reply with ONLY a JSON object, no other text: "
    '{{"found": true, "box": [ymin, xmin, ymax, xmax]}} or {{"found": false}}. '
    "The box is the tight bounding box of the whole object, in normalized "
    "image coordinates from 0 to 1000, where (0,0) is the top-left corner."
)


def ask_vlm(url, jpeg, description):
    body = {"messages": [{"role": "user", "content": [
        {"type": "text", "text": PROMPT.format(description=description)},
        {"type": "image_url", "image_url": {
            "url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}}]}],
        "max_tokens": 200, "temperature": 0}
    req = urllib.request.Request(url.rstrip("/") + "/v1/chat/completions",
                                 json.dumps(body).encode(), {"Content-Type": "application/json"})
    text = json.load(urllib.request.urlopen(req, timeout=90))["choices"][0]["message"]["content"]
    m = re.search(r"\{.*\}", text, re.S)
    return json.loads(m.group(0)) if m else {"found": False, "raw": text}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("description")
    ap.add_argument("--vlm", default="http://singireddys-mac-mini.local:8080")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("ground_check")
    frames, replies = [], []
    node.create_subscription(CompressedImage, "/camera/color/image_raw/compressed",
                             lambda m: frames.append(m), 1)
    node.create_subscription(String, "/vision/pixel_result",
                             lambda m: replies.append(json.loads(m.data)), 10)
    pub = node.create_publisher(PointStamped, "/vision/pixel_query", 10)
    hold = node.create_publisher(PointStamped, "/vision/pixel_snapshot", 5)
    end = time.time() + 10
    # a message sent before DDS has matched the reader is dropped silently
    while (not frames or hold.get_subscription_count() == 0
           or pub.get_subscription_count() == 0) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not frames:
        raise SystemExit("no colour frame in 10 s (is ./rover vlm up?)")
    msg = frames[-1]
    jpeg = bytes(msg.data)
    h_msg = PointStamped()                     # keep THIS photo's depth + pose
    h_msg.header.stamp = msg.header.stamp      # while the VLM looks (hold_frame)
    hold.publish(h_msg)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    from PIL import Image, ImageDraw
    import io
    im = Image.open(io.BytesIO(jpeg))
    w, h = im.size
    t0 = time.time()
    found = ask_vlm(a.vlm, jpeg, a.description)
    print(f"VLM ({time.time() - t0:.1f} s): {found}")
    if not found.get("found"):
        raise SystemExit("VLM: not found")
    ymin, xmin, ymax, xmax = (float(v) for v in found["box"])
    box = (xmin * w / 1000, ymin * h / 1000, xmax * w / 1000, ymax * h / 1000)
    u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2

    req_id = uuid.uuid4().hex[:8]
    q = PointStamped()
    q.header.frame_id = (req_id + ";box=" + ",".join(f"{b:.0f}" for b in box)
                         + ";what=" + a.description.replace(";", " ").replace("=", " "))
    q.header.stamp = msg.header.stamp          # ground on THIS photo's depth + pose
    q.point.x, q.point.y = float(u), float(v)
    pub.publish(q)
    end = time.time() + 8
    reply = None
    while reply is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        reply = next((r for r in replies if r.get("id") == req_id), None)
    print("pixel_to_goal:", json.dumps(reply, indent=1) if reply else "no reply in 8 s")

    d = ImageDraw.Draw(im)
    d.rectangle(box, outline=(255, 0, 0), width=3)
    if reply and reply.get("ok"):
        rel = reply["relative"]
        d.text((box[0] + 4, box[1] + 4),
               f'{a.description}: {reply["depth_m"]} m, {reply.get("slab", "pixel")}', fill=(255, 0, 0))
        print(f'\n=> object {rel["forward_m"]} m ahead, {rel["left_m"]} m left '
              f'(bearing {rel["bearing_deg"]} deg); source: {reply.get("source", "camera")}; slab: {reply.get("slab", "pixel")}'
              + (f'; passed over {reply["skipped"]}' if "skipped" in reply else ""))
    im.save("/tmp/ground_check.jpg")
    print("photo with the box: /tmp/ground_check.jpg")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
