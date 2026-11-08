"""Reach must issue zero before waiting for Nav2 cancellation (no ROS nodes)."""
import ast,sys
from pathlib import Path
from types import SimpleNamespace as NS
source=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent.parent/'nodes/reach_node.py'
tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Reach');fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='run_nav2')
class Future:
 def __init__(self,value=None,done=True):self.value=value;self.ready=done
 def done(self):return self.ready
 def result(self):return self.value
class Goal:
 accepted=True
 def __init__(self,owner):self.owner=owner;self.result_future=Future(done=False)
 def get_result_async(self):return self.result_future
 def cancel_goal_async(self):self.owner.events.append(('cancel',self.owner.t));return Future()
class Fake:
 pose=(0.,0.,0.)
 def __init__(self,stop):
  self.stop_req=stop;self.t=0.;self.events=[];self.goal=Goal(self)
  self.nav=NS(wait_for_server=lambda **kw:True,send_goal_async=lambda g:Future(self.goal))
 def spin_for(self,dt):self.t+=dt
 def stop(self):self.events.append(('stop',self.t))
 def get_clock(self):return NS(now=lambda:NS(to_msg=lambda:None))
 def dist(self,g):return 2.
 def dithering(self,*a):return ''
 def pinned(self,*a):return ''
 def nav_reason(self):return 'test timeout'
for requested_stop in [True,False]:
 f=Fake(requested_stop)
 ns=dict(NAV_TIMEOUT=120,NAV_LOG=Path('/nonexistent/nav.log'),NavigateToPose=NS(Goal=lambda:NS()),pose_msg=lambda *a:NS(header=NS()),time=NS(time=lambda:f.t))
 exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),ns)
 result=ns['run_nav2'](f,(2.,0.,0.),timeout=120 if requested_stop else 0)
 assert f.events[0]==('stop',.1),f.events
 assert f.events[1][0]=='cancel',f.events
 assert f.events[-1][0]=='stop',f.events
 assert result[0]==('cancelled' if requested_stop else 'failed'),result
print('immediate stop on cancel/timeout regressions PASS')
