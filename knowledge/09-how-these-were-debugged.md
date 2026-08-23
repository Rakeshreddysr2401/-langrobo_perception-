# How these faults were actually found

**Used by:** all phases. This is method, not subject matter.

Every entry in [`TODO.md`](../TODO.md) has a diagnosis. This collects the moves
that produced them, because the same handful kept working — and the same handful
of mistakes kept costing sessions.

---

## The mistakes that cost the most

### Inferring a formula from its outputs

Two wrong hypotheses about the costmap came from watching what cost values
appeared and reasoning backwards. The answer came from `strings` on the plugin
`.so` and reading which nav2 merge function it linked against: `updateWithMax`,
which can raise a cost and never lower one. That single symbol explained
everything the output could not.

**When a component misbehaves, read what it links against before theorising
about what it computes.**

### Attributing a failure to a setting that was never in the path

Four cuVSLAM divergences were blamed on `planar_constraints` not working. It is a
field of `SlamConfig`; the pose that diverges is the **odometry** pose, and
`OdometryConfig` has no planar option at all. Three sessions of "why isn't this
flag helping" — because it was never connected to the thing that was failing.

**Before blaming a setting, prove it reaches the code path that is wrong.**

### Trusting the fix instead of verifying the effect

A `git push` reported success and pushed nothing — the branch had moved and the
push named a stale one. `ros2 param set` reports "Set parameter successful" and
changes nothing for parameters read only at init; it returned a **byte-identical**
costmap, which is the tell.

**Check the effect, not the return code.** Compare the remote hash to the local
one. Diff the output that was supposed to change.

### Grepping output that was silently truncated

`ros2 topic echo /fusion/status --once` prints `"vo_implausible":...` and stops.
Anything grepping it reads nothing and concludes health. A proper subscriber
found a divergence that had been running for 55 minutes.

---

## The moves that kept working

### Bisect the parameter, not the code

nav2 refused every goal. Rather than reading the planner, we asked it for goals
at 0.3, 0.4 … 1.0 m and watched where it broke: 0.9 m worked, 1.0 m failed, in
**every direction**. "A radius, not an obstacle" fell straight out of that, and
it was one command.

### Separate what the user did from what the machine did

`loop_test` printed FAIL because the rover did not finish within 10 cm of its
mark. That grades the *driving*. Once it asked for a tape reading, the same run
read 1.6% error — good stereo VO — and the "failure" disappeared.

**When a test grades a human and a machine together, it grades neither.**

### Ask whether it is a jump or a drift

The cuVSLAM z error could have been accumulating or discrete, and those need
opposite fixes. Ninety seconds stationary (`z` held `0.000`, zero steps) plus
4 m of driving (`z` inside 4 cm) ruled out accumulation entirely and pointed at
a discrete event. Neither test needed any new code — just watching the number.

### Make the machine refuse instead of warn

`goto.py` printed a warning that `/cmd_vel` was contended and drove anyway,
wasting the run. **A warning you drive through is not a check.** It now aborts.
The same applies to unreachable goals, blocked routes, and a diverged pose.

### Instrument before the evidence is destroyed

Restarting `vo_node` fixes a divergence *and* rotates away the log that would
explain it. The fix destroys the evidence, so the counters that would identify
the next one — frame gaps, gaps past `max_frame_delta_s`, tracker resets — have
to be in place *before* it happens again.

### Write down the wrong theory, not just the right one

[`TODO.md`](../TODO.md) §18 blamed map noise for nav2 not planning. The map was
rebuilt, every metric improved, and nav2 still could not plan a metre. That entry
is kept, marked wrong, pointing at §19. A plausible cause that fits the symptoms
will otherwise be re-derived by the next person — probably you, in a month.

---

## The check that should exist for every silent failure

Each of these looked healthy while being wrong:

| what looked fine | what was actually happening |
|---|---|
| `/odom` at 20 Hz, all gates green | pose dead-reckoning for 21 m, cuVSLAM 40 m underground |
| costmap publishing, rover "planning" | every goal refused; the rover stood in an unknown cell |
| RViz running, connected | subscribed to nothing — config path given without `-d` |
| map at 5 Hz, growing | 8× too much wall; unplannable |
| nav2 driving | teleop zeros cancelling every command |

**The pattern: a rate is not a health check.** Every one of these had a correct
frequency. `./rover status` now checks the *content* — whether the pose is
tracking or dead reckoning — because that is the question, and no rate answers it.
