"""Actual installed Nav2 consumer regression. Domain42/private outputs ONLY."""
import json, math, os, signal, subprocess, time, sys
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from lifecycle_msgs.srv import ChangeState
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nodes'))
from sonar_nav2_compat import finite_no_echo

assert os.environ['ROS_DOMAIN_ID'] == '42'
use_bridge = os.environ.get('RANGE_PROBE_USE_BRIDGE') == '1'
rclpy.init()
n = Node('range_consumer_probe')
processes = []
received = {k: [] for k in ('original', 'compatible')}
publishers = {}
report = {'domain': 42, 'scope': 'installed collision monitor, isolated velocity outputs', 'phases': {}}

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)

def transition(name, ident):
    c = n.create_client(ChangeState, '/' + name + '/change_state')
    assert c.wait_for_service(timeout_sec=15), name
    req = ChangeState.Request(); req.transition.id = ident
    future = c.call_async(req)
    end = time.monotonic()+15
    while not future.done() and time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.05)
    assert future.done() and future.result().success, (name, ident)
    n.destroy_client(c)

try:
    if use_bridge:
        log = open('/tmp/range_probe_bridge.log', 'w')
        bridge = Path(__file__).resolve().parents[1] / 'nodes' / 'sonar_nav2_compat.py'
        if not bridge.is_file(): bridge = Path(__file__).with_name('sonar_nav2_compat.py')
        processes.append(subprocess.Popen(['python3', str(bridge)], stdout=log, stderr=subprocess.STDOUT))
        log.close()
    report['uses_actual_bridge_node'] = use_bridge
    for kind in received:
        name = 'range_' + kind
        root = '/range_probe/' + kind
        params = dict(base_frame_id='base_link', odom_frame_id='base_link',
                      cmd_vel_in_topic=root+'/in', cmd_vel_out_topic=root+'/out',
                      state_topic=root+'/state', transform_tolerance=.2,
                      source_timeout=1., base_shift_correction=False,
                      stop_pub_timeout=100., enable_stamped_cmd_vel=False,
                      polygons=['Stop'], observation_sources=['sonar'])
        params.update({'Stop.type':'polygon', 'Stop.points':'[[0.207,0.215],[0.207,-0.215],[-0.203,-0.215],[-0.203,0.215]]',
                       'Stop.action_type':'stop', 'Stop.min_points':2,
                       'Stop.visualize':False, 'Stop.enabled':True,
                       'sonar.type':'range', 'sonar.topic':('/nav2_sonar/right' if use_bridge and kind=='compatible' else root+'/range'),
                       'sonar.obstacles_angle':.0349, 'sonar.source_timeout':0., 'sonar.enabled':True})
        cfg=Path('/tmp/'+name+'.yaml');cfg.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        log=open('/tmp/'+name+'.log','w')
        p=subprocess.Popen(['/opt/ros/lyrical/lib/nav2_collision_monitor/collision_monitor',
                            '--ros-args','-r','__node:='+name,'--params-file',str(cfg)], stdout=log, stderr=subprocess.STDOUT)
        log.close();processes.append(p)
        publishers[kind]=(n.create_publisher(Range,('/sonar/right' if use_bridge and kind=='compatible' else root+'/range'),10), n.create_publisher(Twist,root+'/in',10))
        n.create_subscription(Twist,root+'/out',lambda m,k=kind: received[k].append((time.monotonic(),m.linear.x)),10)
        transition(name,1);transition(name,3)
    spin(2)
    for phase,value in [('initial_far',4.),('near',.05),('no_echo',math.inf),('near_again',.05),('invalid_after_near',math.nan),('far_again',4.)]:
        start=time.monotonic();next_pub=0
        while time.monotonic()-start<2.5:
            if time.monotonic()>=next_pub:
                raw=Range();raw.header.stamp=n.get_clock().now().to_msg();raw.header.frame_id='base_link'
                raw.radiation_type=0;raw.field_of_view=.52;raw.min_range=.02;raw.max_range=4.;raw.range=value
                for kind,(rp,cp) in publishers.items():
                    rp.publish(finite_no_echo(raw) if kind=='compatible' and not use_bridge else raw)
                    cmd=Twist();cmd.linear.x=.1;cp.publish(cmd)
                next_pub=time.monotonic()+.05
            rclpy.spin_once(n,timeout_sec=.01)
        summary={}
        for kind,values in received.items():
            tail=[v for t,v in values if t>start+1.5]
            assert len(tail)>=5,(phase,kind,'missing output')
            summary[kind]={'samples':len(tail),'min_vx':min(tail),'max_vx':max(tail)}
            expect_move=phase in ('initial_far','far_again') or (phase=='no_echo' and kind=='compatible')
            assert all(v>.09 if expect_move else abs(v)<1e-8 for v in tail),(phase,kind,tail)
        report['phases'][phase]=summary
    report['passed']=True
finally:
    for p in processes:
        if p.poll() is None:p.send_signal(signal.SIGINT)
    for p in processes:
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    n.destroy_node();rclpy.shutdown()
    Path('/tmp/range_consumer_probe.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
