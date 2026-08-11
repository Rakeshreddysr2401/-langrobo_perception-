# Task 06 — obstacles, without calling the floor one

> **STUB.** Written in full when you get here.

**Needs:** task 05 passed.
**Moves the robot:** no — this is about what the costmap believes.

## Goal

Real obstacles appear as obstacles; the floor never does.

## What you will learn

- What **inflation** is, and why the costmap is bigger than the obstacle
- The difference between "lethal", "inflated" and "free" cells
- Why a wide band of nonzero cost with **no lethal cells** is the signature of
  the floor being mapped

## Known state going in

- `esdf_slice_min_height: 0.12` is a **stale workaround** (`TODO.md §4`). It was
  raised from 0.05 to dodge a 3.7 cm camera-height error that is now fixed, and
  the 2026-08-11 slice experiment showed the floor is not what fills the map.
  Both reasons are gone. It should come down toward ~0.06.
- Cost of leaving it: every obstacle shorter than 12 cm is invisible to nav2.
- **The collision monitor is effectively unprotected** (`TODO.md §3`) — its
  source publishes at ~0.5 Hz against a 2.5 s timeout, and a stale source is
  *ignored*, not treated as danger. Resolve this before task 07 moves anything.

## Gate

*To be written.* Roughly: a box on the floor appears as lethal cells; open floor
has zero cost; lowering the slice band does not reintroduce the floor plateau.

## Learn first

Read [`06-costmaps-and-inflation.md`](../knowledge/06-costmaps-and-inflation.md) — what cost 0/253/254 mean, why obstacles are inflated by the robot's radius, and the signature of the floor being mapped as an obstacle.

Not to memorise. Just so the words in this task mean something.

---

## What I learned doing it

*Fill this in while you still remember being confused — that is the valuable
part, and it evaporates within a day.*

**What surprised me**

>

**What I got wrong first**

>

**Numbers I measured**

>

**Codebase things worth remembering** — a file, a parameter, a line that turned
out to matter more than it looked

>

**Still don't understand**

>

> Housekeeping when you finish: a measured number belongs in `FACTS.md`, a
> broken or unverified thing belongs in `TODO.md`, and anything you learned about
> the *concept* rather than about today should be promoted into
> [`knowledge/`](../knowledge/).
