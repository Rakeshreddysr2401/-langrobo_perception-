# Frame memory: the last 10 minutes of what the robot saw, searchable by words

**Status (2026-10-29):** Jetson half built, running in `./rover up`, and tested
on the robot standing still, together with the Mac mini's VLM (§2b). The Pi 5
half, which uses it in "go near X", is not built yet (§6). Not yet measured
while driving.

"Go near the fridge" after driving through the kitchen should not start a
search. The robot keeps a **keyframe** every time its view changes, embeds it
once with **EmbeddingGemma 2**, and can then find "a fridge" among the last 10
minutes of frames in a fraction of a second. Each match carries its camera
stamp, so `pixel_to_goal` places the object with **that frame's** depth and pose,
which is the same path a photo takes today.

```
image_bridge JPEG (4 Hz) + /odom
     │ moved 40 cm / turned 20° / 30 s still?
     ├──▶ /vision/keyframe_snapshot ──▶ pixel_to_goal keeps its depth (PNG) + camera pose
     └──▶ EmbeddingGemma 2 (host GPU): whole frame + 4 quarters ──▶ ring, > 10 min deleted
"a fridge" ──▶ EmbeddingGemma 2 (text) ──▶ compare with every frame ──▶ best frames + JPEG
                                                       │
     brain's VLM boxes it in that JPEG ──▶ /vision/pixel_query(stamp, box) ──▶ object x, y
```

## 1. Pieces

| Piece | Where | What |
|---|---|---|
| `phase4/tools/embedder.sh` | Jetson **host** (the image is never modified) | llama.cpp b11753 CUDA build, embeddings-only server on `127.0.0.1:8097`, 70 image tokens, Q8 projector. A guard stops it when available RAM < 600 MB, and when the `rover` container is gone. |
| `phase4/nodes/keyframe_memory.py` | container | picks keyframes, embeds them, keeps 10 min, answers queries |
| `phase4/nodes/pixel_to_goal.py` | container | keeps keyframes' depth in its **own** store (400, ≤ 660 s, PNG), separate from the brain's 24 photos; a query by a keyframe's stamp is placed from it |
| `./rover memory` | rover script | starts both, gates them; part of `./rover up` (not fatal) |
| `phase4/tools/test_keyframe_memory.py` | offline tests | 23 checks, no robot, no model |
| `models/embed/` (gitignored) | host | model files, llama.cpp, `reference/` photos; recovery commands in `embedder.sh`'s header |

## 2. Measured (Jetson Orin, whole stack up, robot standing still)

| | Light setting (chosen) | Heavier setting (rejected) |
|---|---|---|
| | 70 image tokens, Q8 projector | 140 tokens, F16 projector |
| One image | 0.31–0.33 s | 0.57 s |
| Text query | ~0.04 s | ~0.05 s |
| GPU, embedding non-stop | — | 99 % |
| `/odom` (stack alone 18.7–19.9 Hz, gate ≥ 15) | 19.0–19.4 Hz | **13.7 Hz** |
| Visual odometry (stack alone ~30 Hz) | 30.8–31.0 Hz | 25.8 Hz |
| Available RAM, model loaded | 1.4–1.7 GB | fell to ~0.2 GB |
| Right picture first (8 labelled tests) | 8/8 | 8/8 |

The same model on the Pi 5's CPU: 3.9 s (70 tokens) to 29.7 s (280) per image,
using all 4 cores. Images are not embedded on the Pi.

Depth per keyframe: 448×252 uint16, 226 KB raw → 68 KB lossless PNG (22 ms to
encode, 3 ms to decode). 400 keyframes ≈ 27 MB.

## 2b. Live on the robot (2026-10-29, standing still in front of shelves, a window and a black bag)

| words | memory (Jetson) | Mac VLM on the hit's JPEG (slot 4) | placed by pixel_to_goal from that keyframe |
|---|---|---|---|
| a black backpack | **strong** +0.105..+0.117, bottom-left quarter, 68-81 ms | "a blue and black bag in the foreground", box in 3.4 s | 0.53 m ahead, 0.08 m left, ~1400 depth points; also from a 103 s old frame |
| shelves with things on them | **strong** +0.146, top-right | — | — |
| a window with metal bars | possible +0.067, top-left | — | — |
| a fridge / a television / a car | weak (+0.004..+0.041) | "found": false (fridge, 1.6 s) | — |
| a white chair (mostly hidden behind the bag) | weak +0.023..+0.028 | **boxed the bag** | placed at the bag: wrong |

- Memory → VLM box → placement: **~5-7 s** in all, almost all of it the VLM.
- **Rule for the Pi 5 half:** only `strong` / `possible` hits go to the VLM; a
  `weak` hit goes straight to the search (the chair row). The arrival check
  stays as the last word.
- Live load: `keyframe_memory` ~7 % of a core, the model server < 1 % averaged;
  RAM available with the model 1.29-1.38 GB. The Jetson's load (~13 on 6 cores)
  is the stack's own (camera driver 42 %, nvblox, ...).
- **Parked:** a same-view refresh every 30 s **replaces** the last frame, so the
  view in front of the robot is always remembered (fixed the same day: skipping
  those refreshes let the only frame age out at 600 s).
- The same day the brain answered by voice "what do you see?" from its camera
  (4.6 s warm; 48 s on the first question after the Mac had been idle: cold cache).

## 3. Contract

Full detail in `keyframe_memory.py`'s docstring.

- `/vision/keyframe_query` (String JSON) `{"id", "text", "k": 3, "jpeg": true}`
- `/vision/keyframe_result` (String JSON) `{"id", "ok", "frames", "baseline", "hits": [{"stamp": {"sec", "nanosec"}, "age_s", "score", "margin", "confidence", "tile", "tile_box", "robot": {"x", "y", "yaw"}, "jpeg"}]}`
- `/vision/keyframes/status` (String JSON, 1 Hz): frames, oldest, embedder up, embed ms, counters
- `/vision/keyframe_snapshot` (PointStamped): keyframe_memory → pixel_to_goal only
- Placing a hit: the existing `/vision/pixel_query` with `header.stamp` = the hit's stamp and `;box=` from the VLM.

**Confidence** = the best frame's score minus the 90th-percentile score of the
same words on ~40 unrelated photos (`models/embed/reference`, vectors cached
in `logs/embed_reference.npz`). "strong" ≥ 0.07, "possible" ≥ 0.045. On a
webcam, a thing in view scored +0.08 to +0.16 and absent things +0.02 to +0.03.
It finds candidates; the VLM and the arrival check decide.

## 4. Run

```bash
./rover memory                         # start + gate (also part of ./rover up)
phase4/tools/embedder.sh status|stop   # the model server alone
./rover logs memory                    # keyframes, queries
python3 phase4/tools/test_keyframe_memory.py
```

## 5. Rules and limits

- **Do not raise the image tokens** without re-measuring `/odom` with the stack up (§2).
- **One session only.** A new `origin_epoch` on `/fusion/status` (odom restarted) empties the memory.
- **Standing-still numbers.** Not yet measured while driving, when nvblox and nav2 work harder.
- **Weak means skip.** A weak hit is not worth a VLM call (§2b, the hidden chair).
- **RAM is the limit.** If the guard stops the model, keyframes pause (`"embedder": false`) and the robot carries on.

## 6. Next: the Pi 5 half

In `approach.py`, between "the photos" and "search": ask `/vision/keyframe_query`.
On a strong or possible hit (never a weak one, §2b), the VLM boxes the object in the hit's JPEG (vision
slot), then `/vision/pixel_query` with the hit's stamp places it, then the
existing go/arrive steps run. The brain's `INTEGRATION_GAPS.md` gets the contract
in the same change. Then the fast spin: rotate once, let keyframes accumulate
(one per 20°), query, and turn to the best.
