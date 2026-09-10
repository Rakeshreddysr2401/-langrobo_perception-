# PERCEPTION_STATE.md — the robot does not know when it has moved

Written 2026-09-10, after reading the whole look → move → look path across the
Pi 5 brain (`~/ros2_ws`, branch `dev-1.3.1-minimal`) and the Jetson's vision
side. The symptom that started it, reported by the user:

> ask "what are you looking at" → it takes a picture and answers.
> then tell it to move somewhere.
> then ask again → **it answers from the old picture.**

That is `Todays_Todo.md` item 14, which was marked ✅ done earlier the same day
(Pi 5 `86e0b08`). This document records why the symptom is still being seen,
what the actual defect is, and what to build instead — under the constraint
that the fix must not disturb the llama.cpp KV cache.

---

## 0. First, the thing that costs a day if it is not said

**The fix committed at 14:17 today has almost certainly never executed.**

| fact | evidence |
|---|---|
| `86e0b08` (the stale-view note) committed | 2026-09-10 **14:17:45** |
| `langrobo_core` is an editable install | `python3 -c "import langrobo_core"` → `~/ros2_ws/src/langrobo_core/...` |
| …but Python imports at process start, so it **still needs a restart** | — |
| the `agent_node` that could have served the failing test started | **13:20:01** — 57 min *before* the commit |
| the machine rebooted at | 14:33:39 (`journalctl --list-boots`, boot −1 ends there) |
| the current `agent_node` (boot 0) **does** carry the fix | started after the commit |
| …and has served **zero turns** since | `journalctl -u langrobo-brain` since boot: only `KV-cache warmed for 'chat' (0 history messages)`, no turn lines |

So the report describes a brain running pre-fix code. **This is the third time
this exact trap has been hit** — the rotation calibration looked unfixed for two
rounds for the same reason, and `~/.claude` memory already records the rule:

> when a fix "does not work", check the process that actually served the
> request before doubting the fix.

An editable install makes it *worse*, not better, because the source on disk is
right and looks live. **Restart or power-cycle, reproduce, and only then decide
whether item 14 failed.**

That said — item 14 as built would have been fighting a headwind anyway, and the
rest of this document is why.

---

## 1. The real defect: `86e0b08` never touched the prompt it contradicts

`86e0b08` changed four files: `approach.py`, `look.py`, `movement.py`,
`test_movement.py`. It did **not** change `prompts.py`.

`prompts.py` `LOCAL_AGENT_PROMPT` still says, as numbered standing rules:

```
2. Follow-up about the SAME scene you just looked at ("did he wear
   spectacles?", "what colour is it?") → reason over the image already in the
   conversation. Do NOT call look() again.
3. Call look() again only if the user implies a new or changed view ("look
   again", "what do you see now", "is it still there"), or the last view is
   stale.
```

Against that, the fix adds one sentence to the tail of a `ToolMessage`:

```
The robot has MOVED, so the camera view has changed: any photo earlier in
this conversation shows where it used to be. ...call look() for a fresh view first.
```

Weigh those two as a 12B model sees them:

| | the standing rule | the stale note |
|---|---|---|
| where | system prompt, top of context | a tool result, buried mid-history |
| when | present on **every** call | present once, drifts backwards each turn |
| form | numbered imperative, `Do NOT` | prose in a tool's return string |
| whose turn | `local_agent`'s own prompt | written during a **`navigate`** turn |

That last row matters and is easy to miss. Movement is `navigate`'s job, so the
note is emitted while `navigate` is active. The user's next question ("what can
you see") routes to `local_agent`, which reads the shared log — the note *is*
there — but by then it is several messages back and `local_agent` is reading its
own rule 2 at the top of its context saying not to look again.

Rule 3 is worse than useless: it makes looking conditional on *"the last view is
stale"*, which is precisely the thing the model has no way to determine. The
`look()` docstring was fixed to say a fresh look is REQUIRED after movement; the
system prompt still says the opposite. **A tool docstring does not outrank the
system prompt.**

**Cheapest real fix in this document, ~5 lines, no cache cost:** rewrite
`LOCAL_AGENT_PROMPT` rules 2 and 3 so the exemption is scoped to "and the robot
has not moved since", and delete "or the last view is stale" in favour of a
condition the model can actually evaluate. Do this whatever else gets built.

---

## 2. The deeper defect: `[Current camera view]` is a label that never expires

`look()` injects the frame as:

```
[Current camera view]  <image>
```

That string is written once and is **never rewritten**. Ten minutes and four
drives later it still reads "Current". The model is not being careless — it is
reading a label that asserts, in the present tense, that the photo is current.

This is the whole bug in one line. Not "the image is stale" — **"the system
tells the model the stale image is current."**

And it generalises past the camera. Nothing anywhere in the brain's context
carries *where the robot is*. `get_current_pose()` exists on the bridge and is
used by `save_location()`, and that is all. Across a whole conversation the
model never sees a single coordinate about itself. It is asked to reason about a
place it cannot locate itself in.

---

## 3. What real embodied systems do about this

Humans and working humanoids do not solve this by remembering an instruction to
re-check. Three mechanisms, in the order a body actually has them:

**a. Proprioception — always on, and cheap.** You know where your body is
without looking at it, continuously, at no cost. Nothing in a human has to
*decide* to update it. **The rover has this and throws it away**: TF publishes
`odom → base_link` continuously and the brain never reads it except to save a
location.

**b. Percepts are stamped, never labelled "now".** A visual memory carries the
vantage point it was taken from. Currency is *computed* — you compare where you
were when you saw it against where you are now — it is never an attribute
written on the memory. When the two disagree you know your picture is out of
date, without anyone telling you.

**c. An allocentric world model.** Object positions in a world frame, not a
camera frame. This is why you can turn around and still say "the chair is behind
me" without looking again. Re-looking is what you do when the model is *stale or
absent*, not what you do after every step.

The rover has (a) and uses none of it, has none of (b), and **deleted (c)**:
`services/world_model.py` existed (Pi 5 `73767fc`) and was removed in `b2bf231`
because it fed off `/vision/detections_3d`, which nothing on this rover has ever
published. The data file it wrote is **still on disk** —
`~/.langrobo/world_model.json`, last written 2026-09-09 — with pose-stamped,
confidence-scored, `seen_count`ed object entries. The idea was right; only its
input was imaginary.

---

## 4. The KV-cache constraint, stated correctly

Every proposal below has to survive this. The repo already understands it, in
three separate places, and they should be read before touching anything:

- `utils/message_utils.py` — **the projection must be append-only.** Each agent
  has its own llama.cpp slot caching its previous prompt; anything that changes
  or vanishes mid-history diverges the prefix from that point on.
- `utils/history.py` — trimming is the **one** deliberate cache reset, and frame
  eviction is deliberately piggybacked onto it because the suffix re-prefills
  anyway. Cap is `history_turns: 12` × 4 = **48 messages**, not 20.
- `registry.py` — the dynamic prompt tail carries **only the date**, because a
  per-minute timestamp made consecutive turns diverge mid-prompt and re-prefill
  ~2k tokens, ~20 s on the 12B model.

**The one trap to name explicitly**, because it is the obvious place to put a
pose and it is the worst: **do not put the robot's pose in
`AgentSpec.context`.** The system prompt is the *prefix*. A pose that changes
per turn there re-prefills the **entire conversation**, every turn, on every
agent. It is the single most expensive edit available. The safe place for
per-turn state is the **tail of the message list**, which is being appended to
anyway.

### A correction to the reasoning in `86e0b08`

The commit rejects stripping stale images with:

> An image present in turn N and gone in turn N+1 diverges the cached prompt
> prefix and re-prefills everything after it, worst exactly where it would bite.

The conclusion (don't strip) is right; **the stated reason is too strong**, and
carrying it forward as a law will block correct ideas later. llama.cpp matches
the longest common prefix. Removing content at position N costs a re-prefill of
what *remains* after N — and the removed image is no longer among it. So:

- Removing the **newest** image costs re-prefilling the **text** after it — a few
  hundred tokens. Cheap.
- Removing an image with **another image after it** costs re-encoding that later
  image through the vision tower. Expensive.

The correct invariant is **"never remove an image that has another image after
it"** — which is the same intuition `KEEP_FRAMES_ON_TRIM` already encodes. It is
still not worth doing here, because §5 is free and strictly better. **Unmeasured
on this stack** — some servers drop the whole multimodal cache on any image
change. Measure before relying on either version.

---

## 5. Proposal: stamp, don't narrate

Stop telling the model the view is stale. Make it **visibly** stale by putting
two numbers next to each other that disagree.

**Change what `look()` writes.** Not "Current" — the pose and time it was taken
from:

```
[Camera view — taken from x=1.20 y=0.34 heading=45°, 16:31:02]
```

Written once, at capture, and never touched again. Append-only by construction,
costs ~15 tokens, and it is *self-invalidating*: nothing has to expire it,
because it never claimed to be current in the first place.

**Frame each user turn with the body state.** One line prepended to the
`HumanMessage` that `agent_node` already builds at
`agent_node.py:661` (`messages = history + [turn_msg or HumanMessage(...)]`):

```
[now at x=2.10 y=0.34 heading=225° — moved 0.90 m and turned 180° since the last camera view]
what can you see?
```

This is exactly the shape `_frame_telegram_turn()` already uses for sender
identity, and the same cache argument applies verbatim: plain text, appended,
prefix untouched.

The result is that the model no longer has to *remember* an instruction or
*obey* a note. It sees a photo taken from heading 45° and a body at heading
225°. That is how a person knows their mental picture of a room is out of
date — not by being told, but because their body state does not match it.

**This also gives the user the coordinates they asked for**, and for free: the
robot can now say "I've moved about a metre since I last looked" instead of
either lying or silently re-looking.

Cost: two short strings per turn. No prompt-prefix change. No history edit.

---

## 6. If stamping is not enough, gate it — do not write a bigger prompt

Stamping is still an *instruction to a 12B model*. Measure it before assuming it
is enough. If the model reads both numbers and answers from the old photo
anyway, the next step is enforcement, not more words. Ranked by value:

**(i) Look for it, in `local_agent`'s node — recommended.**
Before invoking, compare the newest stamped frame's pose against
`get_current_pose()`. If they differ by more than ~15 cm or ~20°, capture a
fresh frame and append it. The model's compliance stops mattering: the newest
`[Camera view — ...]` in its context is genuinely current.

Why this one: it is **lazy** (costs nothing on turns that are not visual),
**append-only** (a new frame at the end is the cheapest possible prefill), and
it needs **zero** model cooperation. It is the smallest change that converts an
instruction into a guarantee.

**(ii) Auto-look when the base stops.** Closest to how a body actually works —
you do not decide to refresh your visual field after walking. But it pays an
image prefill after *every* movement, including the many that are not followed
by a visual question, and it fills history faster so trims (the one real cache
reset) come sooner. Only worth it over (i) if (i) proves too late in the turn.

**(iii) Bring back the world model.** The genuinely different capability: "where
is the chair now that I have turned around" is not answerable by re-looking at
all. Feed it from the path that actually works — the VLM pixel → Jetson
`ground_pixel` → odom coordinate chain in `approach_described_object` — instead
of `/vision/detections_3d`, which is why it was deleted. `~/.langrobo/world_model.json`
and `73767fc`'s `services/world_model.py` are both still recoverable, and the
file format already carries `at`, `first_seen`, `seen_count` and `conf`.

Do (iii) **after** (i) is proven, and not before item #4 (downscale the VLM
frame) — everything here gets cheaper once a look costs 5–15 s instead of 10–40.

---

## 7. Order of work

| # | do | why now | effort | status |
|---|---|---|---|---|
| 1 | Restart the brain and reproduce. | The old fix had never run (§0). | 2 min | superseded by 2+3 |
| 2 | Fix `LOCAL_AGENT_PROMPT` rules 2–3 (§1). | The system prompt contradicted the fix. | ~5 lines | ✅ Pi 5 `4988fe8` |
| 3 | Stamp the frame; frame the turn (§5). | Removes the lying label, adds the coordinates, append-only. | small | ✅ Pi 5 `4988fe8` |
| 4 | **Measure whether 2+3 suffice.** | `look` → `move_robot("F:60,L:180")` → `what can you see`. Did it call `look()` again? | 10 min | ⬜ **next** |
| 5 | Only if 4 fails: the freshness gate, §6(i). | Enforcement instead of instruction. | small | ⬜ |
| 6 | Later: the world model, §6(iii). | A different capability, not a fix for this bug. | medium | ⬜ |

### What shipped (2026-09-10, Pi 5 `4988fe8`, 196 tests — was 184)

New `langrobo_core/utils/pose_stamp.py` owns both stamps and the last-view pose
(module-level, the way `_bridge.py` holds the ROS adapter — so `StubBridge` and
`ROS2Bridge` stay identical for free rather than being a field two classes must
remember to mirror).

| where | before | after |
|---|---|---|
| `look()`'s label | `[Current camera view]` | `[Camera view — taken at 16:31:02 from x=1.20 y=0.34 heading=45°]` |
| every user turn | nothing | `[Robot now at … — that is 0.90 m and 180° from where the last camera view was taken, so that photo shows somewhere it has left]` |
| `LOCAL_AGENT_PROMPT` | "Do NOT call look() again" | a `== IS YOUR VIEW STILL GOOD? ==` block that says to compare the two poses |
| the movement note | a fixed sentence | the same sentence **plus the measured displacement**, so it lines up with the pose on the frame |
| `SPEECH_STYLE` | — | bracket tags are telemetry: use them, never read them aloud |

Four edges that cost something to find, all now covered by tests:

- **Headings wrap.** 359° and 1° are 2° apart, not 358°. Without folding into
  (−180, 180] every crossing of the wrap point reads as a half-turn and forces a
  needless 10–40 s look.
- **The label and the stamp must print the same convention**, or the model is
  asked to compare `225` with `-135` and reasonably concludes they differ.
- **A pure pivot moves zero metres** and changes the view completely, so distance
  alone cannot decide staleness — hence `MOVED_M` *or* `TURNED_DEG`.
- **Losing TF prints "position unknown"** rather than defaulting to unmoved,
  which would silently license the old photo.

The stamp stays absent until the first `look()` — before any frame there is
nothing to compare against, and a pose on "what's the weather" is noise on every
text turn. And it rides on the `HumanMessage` that is being appended anyway
rather than adding a message of its own: the 48-message cap counts **messages**,
so a third message per turn would pull trims — the one real cache reset —
forward.

**Both packages are `--symlink-install`** (`build/langrobo_ros/langrobo_ros` is a
symlink to `src/`), so a restart is enough and **no `colcon build` is needed**
for a `.py` change in either. A restart is still required — an editable install
does not make a running process reload.

**One measurement rule carried over from `Todays_Todo.md`, and it applies to
every step above that moves the rover:** no motion measurement on this rover is
trustworthy unless `/wheel_state` continuity is checked over the same window
(TODO 33 — the micro-ROS wheel link drops under driving load). A turn that
silently did not happen will read as a model failure here.

---

## 7b. A second, worse bug found in a real transcript (2026-09-10 evening)

Before either fix above was tested, the user pasted a transcript from an
earlier session. It shows the actual failure mode, and stamping the frame
would never have touched it:

```
left, left, "what are you looking at" -> "I am looking at the area around
the chair." No look() call. No handover. No image anywhere in the turn.
```

`navigate` is **sticky** (`registry.py`, since 2026-09-08 — a multi-step drive
should cost one LLM call, not a routing hop on every follow-up). So after
"left", the *next* turn re-enters `navigate` directly, skipping `chat`'s
routing table entirely. `NAVIGATE_PROMPT` rule 6 already says a no-movement
follow-up goes to `chat`; `CHAT_PROMPT` already says a visual question goes to
`local_agent`. **Both are instructions, and the Mac mini's 12B model ignored
them three times in the one transcript**, answering directly in specific,
plausible-sounding prose instead. `local_agent` — the only agent with `look()`
— was never entered.

§5 and §6's stamps live entirely inside `LOCAL_AGENT_PROMPT`. They do nothing
for a turn that never reaches `local_agent`. Fixed Pi 5 `1de1e9e`: a
deterministic backstop in `graph/build.py`'s loop-guard wrapper —
`_vision_backstop` — fires when a non-`local_agent` node is about to end the
turn by speaking with no tool call, `local_agent` has not already run this
turn, and the **user's own words** (not the model's reply — that parsing is
exactly what the model is already failing at) match a narrow vision-question
pattern. On a hit it does not delete the wrong reply (append-only, same rule
as the images), it appends a routing note and chains to `local_agent` in the
same turn — `agent_node` only speaks the *final* message, so the wrong one
never reaches the user. 13 new tests, 209 total.

**Also correctly NOT a bug**, easy to misread as one from the same transcript:
"take fresh pic" routed to `send_telegram_photo`, not `look()`. That tool
grabs a genuinely fresh frame on its own (see its docstring) and is legitimately
`chat`'s — not a symptom of anything above.

## 8. What this does not fix

- **Nothing above gives the robot memory across boots.** Every session starts at
  a fresh odom origin, so a stamped pose is only comparable *within* one power
  cycle. That is the relocalization gap (Phase 2c, deferred by choice) and it
  becomes blocking the moment the world model in §6(iii) is meant to persist.
- **`navigate_to_pose` emits its stale note when navigation *starts*,** not when
  it ends — the arrival lands minutes later as a `[SYSTEM]` turn. The turn
  framing in §5 covers the gap by construction; the note does not.
- **The Studio / `agent_node` split still applies.** Studio runs the same graph
  in a different process. Anything tested through one is not evidence about the
  other — see `Todays_Todo.md` #9 and `scripts/start_studio.sh`.
