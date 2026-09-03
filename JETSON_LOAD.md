# Jetson load — what's on the box, what it costs, what it's for

Measured 2026-09-03 on `rakhi-jetson`, a Jetson Orin Nano Super Developer Kit
(8 GB, 6 CPU cores), branch `rover-v1.0.6`. Every number below was read off the
running machine, not remembered.

---

## The verdict

| | |
|---|---|
| **Right now** | Stack is not running. GPU 0%, CPU ~5–8%, `VDD_IN` ~5 W. All current load is desktop + IDE + Claude sessions, not perception. |
| **Power mode** | `15W` selected. `25W` and `MAXN_SUPER` both exist and are unused — see §1. |
| **Disk** | 172 G / 227 G used (80%), 44 G free. Docker alone owns 126 G of that. |
| **Biggest single win** | `docker builder prune -a` — 23.6 G of build cache from a directory whose own README says nothing has been built from it. |

---

## 1. Power mode is the real finding

```
nvpmodel -q  →  NV Power Mode: 15W   (id 0)
available:      15W (0) / 25W (1) / MAXN_SUPER (2)
```

This board is the *Super* revision — its entire performance story is
`MAXN_SUPER` (the unlocked clocks that give the Orin Nano Super its 67-TOPS
number instead of the stock 40). It has been running at the lowest of three
available modes.

READINESS.md already flags **fusion as CPU-bound** and cuVSLAM/nvblox as the
tightest-margin parts of the stack. Power mode caps CPU and GPU clocks
directly — 15W is the one setting most likely to be *causing* headroom
problems currently being explained by algorithm behavior. Before spending more
effort on the fusion CPU cost, switch to `MAXN_SUPER` (`sudo nvpmodel -m 2 &&
sudo jetson_clocks`) and re-run the fusion rate measurement. Thermal cost is
real (idle temps already ~50°C sitting in 15W) but unmeasured at the higher
modes — that's the next data point, not an assumption.

---

## 2. Disk — Docker is 74% of the used 172 G

```
docker system df
Images          7   TOTAL 102.5 GB   ACTIVE 1
Build Cache    38   TOTAL  23.6 GB   ACTIVE 0
```

**Necessary — do not touch:**

| image | role |
|---|---|
| `orin-nav:1.1` (`2a3d7f1d…`) | the image `rover` actually runs, per `docker/README.md`. Built by `docker commit`, not a Dockerfile — **this image is its own only backup** (TODO §13). |
| `isaac_ros:cuvslam-unified`, `isaac_ros:langrobo-prod` | ancestor layers of the above. Deleting them is not possible without deleting `orin-nav:1.1` — they're the same chain, not separate weight. |

**Unnecessary / worth reclaiming:**

- **Build cache, 23.6 G, safe.** `~/rover/docker/README.md` says outright:
  *"Nothing here has been built or tested from this repo."* No active
  Dockerfile work is happening from this checkout, so this cache has nothing
  to serve. `docker builder prune -a` recovers it — more than half the current
  44 G free.
- **`orin-nav:0.0.2` (`e1c32d08f6c6`), 57.8 G nominal.** A *different* image ID
  from the one actually run, one day older, not named anywhere as in use. The
  repo's own docs warn about exactly this: *"pin the ID, not the tag — a tag
  is a movable label."* Candidate for `docker rmi`, but check `docker history`
  against `orin-nav:1.1` first — if it shares the same base layers the real
  disk win may be small (the images already share ~97% of their nominal 57.8 G
  each, hence build cache showing only 62 MB "reclaimable" today).

**~/robot/data — mapping-crash residue, ~9.3 G, safe to archive/delete:**

| file | size | what it is |
|---|---|---|
| `rtabmap.db` | 5.3 G | **live — keep** |
| `rtabmap.db.reset-20260718-102456` | 7.0 G | a reset backup, 47 days stale |
| `rtabmap.db.corrupt-19700101-053115` | 1.4 G | epoch-timestamped — a crash artifact from a clock-less boot |
| `rtabmap.db.corrupt-20260717-121422` | 967 M | another crash backup |
| `rtabmap.db.corrupt-19700101-053112` | 1.3 M | same pattern, tiny |
| `rtabmap.db.bak-desk-era` | 100 K | named backup, negligible |
| `rtabmap_sim.db` | 2.6 G | simulation DB — keep if sim is still used, else archive |

None of these are referenced from `~/rover` (`TODO.md`, `READINESS.md`) — they
predate this repo's tracked history. Worth a tar-and-move-off-device rather
than outright deletion, in case a corrupt-file postmortem is still wanted.

**Low-priority, small:**

- `~/rerun-venv`, 612 M, untouched since 2026-07-19 (46 days), not referenced
  by anything in `~/rover` or `~/robot/scripts`. Looks like an abandoned
  visualization experiment.

**Necessary, keep as-is:**

- `~/robot/models/hf_cache` (8.1 G) and `whisper_cache` (539 M) — local LLM /
  speech models for the voice pipeline (`VOICE_PIPELINE.md`), which is the
  actual next-phase direction (physical AI on the Pi 5).
- `~/jetson-containers` (2.7 G) — NVIDIA's container build tool. Not currently
  in the critical path, but `docker/README.md` names
  `orin-nav-stack/standalone/Dockerfile.cuvslam-jp72` as "the closest thing to
  a starting point" if the production image is ever lost — this tool is what
  you'd rebuild with. Keep.

---

## 3. What's actually running, and what it costs

At idle, with the rover stack down:

| process | CPU | RSS | what |
|---|---|---|---|
| PyCharm `remote-dev-server` | 65–70% of 1 core | 2.7 G | JetBrains Gateway backend, indexing `~/rover` |
| `claude` (pts/1) | ~46% | 350 M | this session |
| `claude` (pts/0) | ~10% | 476 M | a second, separate Claude Code session, running 12+ min |
| desktop (`gnome-shell`, `gnome-remote-desktop`, `bluetooth`, `avahi-daemon`) | low individually | — | stock Ubuntu-desktop-on-devkit, not part of the rover stack |

None of this is "wrong" — it's how the box is being developed on — but it's
worth being deliberate about on an 8 G board: at idle, 3.6 G is already used
and only 3.7 G is "available." PyCharm indexing plus a local LLM (`hf_cache`
above) plus the full perception/nav stack (cuVSLAM + nvblox + nav2 + fusion,
all on these same 6 cores) will contend for both RAM and CPU if run
concurrently. If a run feels short on headroom, checking what the IDE backend
is doing is a legitimate first move, not just the perception stack.

**Always-on network services worth a deliberate look, not a reflexive
disable** (this box is reached remotely, so don't touch these blind):

- `iperf3.service` — running as a persistent server, meaning a bandwidth-test
  listener is always up. Fine if intentional for link testing; open network
  surface if not.
- `openvpn.service` — active. Confirm this is the intended remote-access path
  before assuming it's unused.
- `gnome-remote-desktop.service`, `avahi-daemon.service`, `bluetooth.service` —
  standard devkit defaults, not part of "the business" (autonomous driving),
  low individual cost.
- `sssd.service` is enabled but **inactive** — present, not running, no cost.

---

## 4. Reproduce these numbers

```bash
uptime; free -h                          # load, memory
tegrastats --interval 1000               # GPU%, temps, VDD_IN — kill after a few lines
nvpmodel -q                               # power mode
docker system df                          # image/cache disk accounting
docker images --format "table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}"
du -sh ~/robot/data/* ~/robot/models/*    # what's actually on disk and how big
systemctl list-unit-files --state=enabled
```

---

## The one line worth remembering

The stack itself is lean — 15 MB of git-tracked repo, one running image that
can't be shrunk without losing the only backup of it. Everything expensive on
this box is either **dev tooling** (IDE, build cache from an unused
Dockerfile) or **crash residue** (corrupt rtabmap snapshots) — not the rover.
And the highest-leverage single change isn't a deletion at all: this Orin Nano
Super has been idling one power tier below what it's built for.
