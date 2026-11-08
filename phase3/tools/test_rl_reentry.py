"""Real mixed-policy S-corridor recovery, retry bounds and opt-out propagation."""
import math,re,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'nodes'))
import rl_reentry as R

def main():
    # Recorded first RL stop and next recovery after a checked fallback maneuver.
    stop=(1.2110543,.2329175,23.32)
    next_pose=(1.3203093,.9769301,.5018972)
    assert R.allowed(0,[],(0,0,0),0)
    assert not R.allowed(1,[stop],stop[:2],59.21)
    assert R.allowed(1,[stop],next_pose,59.21)
    assert not R.allowed(1,[stop],next_pose,25), 'cooldown must prevent rapid switching'
    assert not R.allowed(1,[stop],next_pose,59.21,max_rounds=1), 'default budget must remain one'
    assert not R.allowed(3,[stop],next_pose,59.21), 'total budget must be finite'
    second=(next_pose[0],next_pose[1],60)
    assert not R.allowed(2,[stop,second],stop[:2],90), 'no return to an old stalled spot'
    assert R.allowed(2,[stop,second],(-.5,-.5,0),90), 'checked retreat may open a new context'
    assert not R.allowed(1,[stop],(math.nan,0,0),90)
    assert not R.allowed(1,[stop],None,90)
    script=Path(__file__).resolve().parents[2]/'rover'
    s=script.read_text();fn=re.search(r'^rl_env\(\)\{.*?^\}',s,re.S|re.M).group(0)
    for setting,value in [('',0),('REACH_RL_REENTRY=1',1)]:
        code='set -eu\nREPO=/nonexistent; REACH_RL=1; REACH_RL_MODEL=/models/test.npz\n'+setting+'\n'+fn+'\nrl_env\n'
        result=subprocess.run(['bash','-c',code],capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        assert f'REACH_RL_REENTRY={value}' in result.stdout,result.stdout
    print('recorded recovery, no-repeat/cooldown/budget, finite pose and launch opt-out PASS')
if __name__=='__main__':main()
