import ast,math,sys,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
run=Path(__file__).resolve().parent
sys.path.insert(0,str(run.parent/'nodes'))
import widest as WD, memory_cells as MC
source=ast.parse((run.parent/'nodes/reach_node.py').read_text())
cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='Reach')
fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tight_should_lead')
ns=dict(np=np,math=math,time=time,WD=WD,MC=MC,NEED_W=.42,NAV2_W=.56,WIDEST_MAX_M=4.,CAMERA_HANDOVER=True)
exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual candidate method>','exec'),ns)
class Fake:
 tight_should_lead=ns['tight_should_lead']
 pose=(0.,0.,0.)
 def dist(self,g):return math.hypot(*g[:2])
 def widest_way(self,g):return WD.widest(np.vstack(self.scans),g[:2])[0]
 def say(self,**kw):self.last=kw
 @staticmethod
 def outside_body(p):return p[~((p[:,0]<.182)&(p[:,0]>-.178)&(abs(p[:,1])<.19))]

def scene(name):
 rm=dict(np.load(run/'fixtures'/f'handover_{name}.npz'));p=rm['pts']-rm['start'][:2];th=rm['start'][2];c,s=math.cos(th),math.sin(th)
 pb=p@np.array([[c,-s],[s,c]])
 g=rm['goal']-rm['start'][:2];gb=g@np.array([[c,-s],[s,c]])
 f=Fake();f.scans=[pb[rm['lidar']]]
 view=(pb[:,0]>.35)&(np.hypot(*pb.T)<3)&(abs(pb[:,1])<pb[:,0]*math.tan(math.radians(43.5)))
 points=pb[rm['camera']&view]
 a=np.zeros((320,320),np.int8);ij=np.floor((points+4)/.025).astype(int);ij=ij[((ij>=0)&(ij<320)).all(1)];a[ij[:,1],ij[:,0]]=100
 stamp=time.time();sec=int(stamp)
 f.lcm=NS(data=a.ravel().tolist(),info=NS(height=320,width=320,resolution=.025,origin=NS(position=NS(x=-4.,y=-4.))),header=NS(stamp=NS(sec=sec,nanosec=int((stamp-sec)*1e9))))
 return f,(*gb,0.)
f,g=scene('gap45');assert f.widest_way(g) is None
ns['CAMERA_HANDOVER']=False;assert not f.tight_should_lead(g), 'opt-out must preserve existing decisions'
ns['CAMERA_HANDOVER']=True;assert f.tight_should_lead(g),getattr(f,'last',None)
f.lcm.header.stamp.sec-=10;assert not f.tight_should_lead(g),'stale map must not guide handover'
f,g=scene('gap36');assert not f.tight_should_lead(g),'map hint must not override impossible LiDAR width'
f=Fake();f.scans=[np.array([[3.,3.]])];f.lcm=None;assert not f.tight_should_lead((1.,0.,0.))
print('candidate handover geometry/freshness/impossible-route regressions PASS')
