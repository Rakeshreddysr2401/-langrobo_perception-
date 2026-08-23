# What lives on the laptop, and where it comes from

The laptop renders RViz. It holds **no source of truth** — everything on it is a
copy pushed from this repo, so it can be wiped and rebuilt from here.

| on the laptop | comes from | pushed by |
|---|---|---|
| `~/rover_live.rviz` | `phase2/rviz/rover_live.rviz` | `./rover view`, every run |
| `~/rover_live.sh` | `phase2/rviz/rover_live.sh` | by hand, `scp` |

It used to hold four RViz configs and three launcher scripts, and its copy of the
config drifted from the repo's — which made "I fixed the RViz config" untrue on
the only machine that renders it. The old ones are archived in `~/old-rviz/` and
nothing reads them.

## Rebuilding it from scratch

```bash
scp phase2/rviz/rover_live.sh   rakhi24@<laptop>:~/rover_live.sh
scp phase2/rviz/rover_live.rviz rakhi24@<laptop>:~/rover_live.rviz
```

Then `./rover view` from the Jetson, or `bash ~/rover_live.sh` on the laptop.

**The laptop's IP is not fixed.** DHCP has moved it from .10 to .17. `./rover view`
finds it by looking for the host that answers ssh *and* has `~/rover_live.sh` —
the Jetson and the Pi 5 both answer ssh too.

## Two things that cost a session each

**The `-d` is load-bearing.** `rviz2 ~/rover_live.rviz` silently ignores the file
and starts with defaults — Fixed Frame `map`, zero displays. A blank window, no
error, and the ROS graph shows RViz connected while subscribing to nothing.

**Nobody logged in means nothing renders.** RViz started over ssh into a login
screen renders into a void and exits 0. `./rover view` refuses in that case
rather than reporting success.
