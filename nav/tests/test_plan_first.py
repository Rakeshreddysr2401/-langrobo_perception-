"""The width hint asks the planner first (reach_node.nav2_takes_it, PLAN_FIRST).

Recorded case, real robot 2026-11-14 (goal stamp 1794674141, session_20261114b): widest way 55.0 cm
(< nav2's 56), goal 1.22 m away -> 47 s in the slow drivers, then nav2 drove it. With the planner
asked first that goal goes to nav2 at once; a tour round the house or no route keeps the old order.
"""
import ast
from pathlib import Path

src = (Path(__file__).resolve().parent.parent / 'nodes' / 'reach_node.py').read_text()
tree = ast.parse(src)
keep = [n for n in tree.body
        if (isinstance(n, ast.FunctionDef) and n.name == 'nav2_takes_it')
        or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Tuple) and [e.id for e in t.elts] == ['PLAN_DIRECT', 'PLAN_SLACK_M'] for t in n.targets))]
assert len(keep) == 2, 'nav2_takes_it and its constants must exist at module level'
ns = {}
exec(compile(ast.Module(body=keep, type_ignores=[]), 'reach_node.py', 'exec'), ns)
takes = ns['nav2_takes_it']

assert takes(1.4, 1.22), 'the recorded goal: a direct route goes to nav2'
assert takes(1.22 * 1.5 + 0.5, 1.22)
assert not takes(None, 1.22), 'no route / no answer: the hint leads, as before'
assert not takes(6.0, 1.22), 'a tour round the house is not this way: the hint leads'
assert not takes(2.9, 1.22)

# asked once per goal: after nav2 has failed the hint must lead (no nav2 loop)
loop = src[src.index('plan_asked = False'):]
assert loop.count('self.plan_length(g) if PLAN_FIRST and not plan_asked') == 1
assert loop.index('plan_asked = True') < loop.index("tight_led = True")
print('plan-first decision PASS')
