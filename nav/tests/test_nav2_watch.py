"""reach's two watches on a running nav2: NEAR (hand to the exact finish) and PROGRESS (no_progress).

Rules chosen on the 16 nav2 runs of the real bags 2026-11-10 / 11-14 / 11-14b (kino_sim
tools/nav2_progress.py). The traces below are nav2's remaining distance from those runs, 2 s apart.
"""
import ast
import math
from pathlib import Path

src = (Path(__file__).resolve().parent.parent / 'nodes' / 'reach_node.py').read_text()
tree = ast.parse(src)
keep = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name == 'no_progress') or
        (isinstance(n, ast.Assign) and any(isinstance(t, ast.Tuple) and [e.id for e in t.elts] == ['PROGRESS_S', 'PROGRESS_M', 'PROGRESS_RESET_M'] for t in n.targets))]
assert len(keep) == 2
ns = {'math': math}
exec(compile(ast.Module(body=keep, type_ignores=[]), 'reach_node.py', 'exec'), ns)
no_progress = ns['no_progress']


def run(trace, dt=2.0):
    """First time (s) the watch fires on a trace of remaining distances, or None."""
    fb = []
    for i, d in enumerate(trace):
        fb.append((i * dt, d))
        if no_progress(fb, i * dt):
            return i * dt
    return None


# 11-14b at 4102 (34 s, finished): steady approach
good = [1.6, 1.4, 1.2, 1.0, 0.9, 0.8, 0.7, 0.62, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.18]
assert run(good) is None
# a half turn on the spot first (13 s without progress), then driving
assert run([1.2] * 7 + [1.1, 1.0, 0.8, 0.6, 0.4, 0.3, 0.2]) is None
# re-planned a longer way (+0.9 m) after 20 s, then progress on it: no fire
assert run([1.0, 0.95, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 1.8, 1.7, 1.6, 1.45, 1.3, 1.15, 1.0, 0.85, 0.7]) is None
# 11-14b at 3832: wandering 0.33-0.84 m along the path for 75 s, never nearer
wander = [0.42, 0.33, 0.59, 0.65, 0.75, 0.72, 0.46, 0.80, 0.65, 0.46, 0.62, 0.84, 0.61, 0.58, 0.73, 0.51, 0.74, 0.36, 0.71]
t = run(wander, dt=4.0)
assert t is not None and 28 <= t <= 40, t
# stuck in place (tonight's "the controller made no progress"): fires at ~30 s, not 120
assert 30 <= run([0.9] * 40) <= 34
assert no_progress([], 100.0) == ''
# NEAR is wired in run_nav2 and both can be switched off together
assert "if self.dist(g) <= NAV2_ARRIVED_M:" in src and "NEAR_HOLD_S" in src and "NAV2_WATCH = os.environ.get('NAV2_WATCH', '1') == '1'" in src
assert "feedback_callback=on_fb" in src
print('nav2 watch PASS')
