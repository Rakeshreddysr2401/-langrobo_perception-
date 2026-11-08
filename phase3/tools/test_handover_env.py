"""The opt-in must reach Docker's launch command and default to disabled."""
import re,subprocess,sys
from pathlib import Path
p=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[2]/'rover'
s=p.read_text();fn=re.search(r'^rl_env\(\)\{.*?^\}',s,re.S|re.M).group(0)
for setting,expected in [('',0),('REACH_CAMERA_HANDOVER=1',1)]:
 code='set -eu\nREPO=/nonexistent; REACH_RL=1; REACH_RL_MODEL=/models/test.npz\n'+setting+'\n'+fn+'\nrl_env\n'
 r=subprocess.run(['bash','-c',code],text=True,capture_output=True)
 assert r.returncode==0,r.stderr
 assert f'REACH_CAMERA_HANDOVER={expected}' in r.stdout,r.stdout
 assert 'REACH_RL=1 REACH_RL_MODEL=/models/test.npz' in r.stdout,r.stdout
print('handover opt-in propagation/default regressions PASS')
