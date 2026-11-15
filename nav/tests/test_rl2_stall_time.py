"""reach gives rl2 the stall time it was trained with (rl2_drive.STALL_S), not the older 15 s.

Real robot 2026-11-15 (rl_dumps/20261114-202740_failed): a goal 1.5 m behind the rover; rl2 turned
round in 12 s and was closing in when reach ended it at 15 s. Training (kino_sim rl2/env.py STALL_S)
truncates at 25 s and the policy observes time-since-progress / 25.
"""
import ast
import sys
from pathlib import Path

nodes = Path(__file__).resolve().parent.parent / 'nodes'
sys.path.insert(0, str(nodes))
import rl2_drive as R2  # noqa: E402

src = (nodes / 'reach_node.py').read_text()
assert R2.STALL_S == 25.0, 'kino_sim rl2/env.py STALL_S; change both together'
assert "stall_s = R2.STALL_S if getattr(pol, 'rl2', False) else RL_STALL_S" in src
assert 'if now - best_t > stall_s:' in src and 'if now - best_t > RL_STALL_S:' not in src
ast.parse(src)
print('rl2 stall time PASS')
