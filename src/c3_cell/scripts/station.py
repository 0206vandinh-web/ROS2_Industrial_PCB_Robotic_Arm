#!/usr/bin/env python3
"""Task supervisor. ROS callbacks keep the CAN heartbeat alive during planning."""
import json,threading,time,math,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from std_msgs.msg import String,UInt8MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped,Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan,GetStateValidity,ApplyPlanningScene,GetPlanningScene
from moveit_msgs.msg import (Constraints,JointConstraint,RobotState,CollisionObject,AttachedCollisionObject,
                             DisplayTrajectory,RobotTrajectory,AllowedCollisionEntry,PlanningSceneComponents)
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation
import numpy as np
from core import *
ROOT=Path(get_package_share_directory('c3_cell'));DATA=Path.home()/'.ros/c3_arm'
NAMES=[f'j{i}' for i in range(1,7)]
STEPS=['observe','prepick','pick','close','lift','transfer','land','screw','up','to_good','lower','open','retreat']
def atomic(path,obj):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(obj,indent=2));os.replace(tmp,path)
def pose(xyz,quat=(0.,0.,0.,1.)):
    p=Pose();p.position.x,p.position.y,p.position.z=map(float,xyz)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat);return p
def box(name,xyz,size,quat=(0,0,0,1)):
    o=CollisionObject();o.id=name;o.header.frame_id='world';o.pose.orientation.w=1.
    o.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,size)))]
    o.primitive_poses=[pose(xyz,quat)];return o
class Station(Node):
    def __init__(self):
        super().__init__('c3_station');DATA.mkdir(parents=True,exist_ok=True)
        self.kin=Kin(Path(get_package_prefix('c3_cell'))/'lib/libc3_kin.so')
        for k,v in [('recipe','assembly'),('speed',0.),('accel',0.),('loops',1),('teach_slot','good')]:self.declare_parameter(k,v)
        self.fabric=self.create_publisher(UInt8MultiArray,'/c3/tx',1)
        self.stat=self.create_publisher(String,'/c3/status',10)
        self.marker=self.create_publisher(Marker,'/c3/label',1)
        self.preview=self.create_publisher(DisplayTrajectory,'/display_planned_path',1)
        self.create_subscription(UInt8MultiArray,'/c3/rx',self.receive,10)
        self.create_subscription(PoseStamped,'/c3/target',self.observe,10)
        self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.valid=self.create_client(GetStateValidity,'/check_state_validity')
        self.apply=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.scene=self.create_client(GetPlanningScene,'/get_planning_scene')
        self.feedback=None;self.target=None;self.rx_at=self.seen=0.;self.seq=0;self.goal=None;self.io=0;self.jaw=.014
        self.running=False;self.abort=False;self.stop=False;self.estop=False;self.label='IDLE';self.err='';self.idx=0;self.cycle=0
        self.path=DATA/'checkpoint.json';self.trace=(DATA/f'run-{time.time_ns()}.jsonl').open('a',buffering=1)
        self.ready=False;self.src=None;self.yaw=0.;self.params={};self.returning=False;self.latched=False
        for name in ['auto','step','pause','resume','reset','estop','teach','save_point','reload']:
            self.create_service(Trigger,'/c3/'+name,lambda req,res,k=name:self.service(k,res))
        self.create_timer(.01,self.tick);self.create_timer(.20,self.report)
    def observe(self,msg):
        self.target=msg;self.seen=time.monotonic()
    def record(self,**v):self.trace.write(json.dumps(dict(t=time.time(),**v))+'\n')
    def receive(self,m):
        try:f=unpack(bytes(m.data))
        except ValueError:return
        if f['kind']!=2:return
        self.feedback=f;self.rx_at=time.monotonic()
        if self.goal is None:self.goal=f['q'].copy()
        if f['fault'] and not self.latched:
            self.latched=True;self.abort=True;self.err=f"device fault 0x{f['fault']:02x}";self.label='FAULT'
        self.record(rx=bytes(m.data).hex(),q=f['q'].tolist(),pcb_attached=bool(f['io']&1))
    def tick(self):
        if self.goal is None:return
        if time.monotonic()-self.rx_at>.5:
            self.latched=True;self.abort=True;self.err='feedback timeout';self.label='FAULT'
        self.seq+=1
        data=packet(1,self.seq,int(time.monotonic()*1000),self.goal,self.jaw,self.io,
                    flags=2 if self.estop else int((self.running or self.label=='PAUSED') and not self.latched))
        self.fabric.publish(UInt8MultiArray(data=list(data)));self.record(tx=data.hex(),target=self.goal.tolist())
    def report(self):
        q=self.feedback['q'] if self.feedback else None
        status=dict(state=self.label,step=self.idx,cycle=self.cycle,error=self.err,
                    tcp_m=self.kin.fk(q)[:3,3].round(5).tolist() if q is not None else [],q_rad=q.tolist() if q is not None else [])
        self.stat.publish(String(data=json.dumps(status)))
        m=Marker();m.header.frame_id='world';m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose=pose([.04,0,.50])
        m.scale.z=.026;m.color.a=1.;m.color.r=1.;m.color.g=.2 if self.err else .8
        m.text=self.label+' '+self.err+'\nTCP '+str(status['tcp_m'])+'\nJ deg '+str(np.round(np.degrees(q),1).tolist() if q is not None else []);self.marker.publish(m)
    def service(self,k,r):
        try:
            if k=='estop':self.estop=self.latched=self.abort=True;self.label='ESTOP'
            elif k=='pause':self.stop=True # Finish the current atomic step; stop before the next one.
            elif self.running:raise ValueError('task busy')
            elif k=='reset':
                if self.feedback is None or self.feedback['fault'] or self.feedback['io']&1:raise ValueError('clear physical fault / unload first')
                self.latched=self.estop=self.abort=False;self.idx=self.cycle=0;self.label='IDLE';self.err='';self.ready=False;self.path.unlink(missing_ok=True)
            elif k in ('reload','save_point') and self.path.exists() and (not self.ready or json.loads(self.path.read_text())['pending']):raise ValueError('restore clean checkpoint first')
            elif k=='reload':
                self.load_recipe()
                if self.path.exists():self.snapshot(False)
            elif k=='save_point':
                if not self.feedback or time.monotonic()-self.rx_at>.5:raise ValueError('no fresh feedback')
                self.load_recipe()
                key=self.get_parameter('teach_slot').value
                if key not in ('good','fixture','infeed'):raise ValueError('invalid teach_slot')
                self.params[key]=self.kin.fk(self.feedback['q'])[:3,3].tolist()
                atomic(DATA/(self.get_parameter('recipe').value+'.json'),self.params)
                if self.path.exists():self.snapshot(False)
            elif k=='teach':
                if not self.feedback or time.monotonic()-self.rx_at>.5:raise ValueError('no fresh feedback')
                p=DATA/'teach.json';poses=json.loads(p.read_text()) if p.exists() else []
                poses.append(dict(q=self.feedback['q'].tolist(),tcp=self.kin.fk(self.feedback['q']).tolist()));atomic(p,poses)
            else:
                if self.latched or not self.feedback or time.monotonic()-self.rx_at>.5:raise ValueError('no healthy live feedback')
                if self.path.exists() and not self.ready and k!='resume':raise ValueError('checkpoint exists: resume or reset')
                if k=='resume' and self.path.exists():self.restore()
                if not self.ready:self.load_recipe()
                self.stop=self.abort=False;self.running=True;self.err=''
                threading.Thread(target=self.work,args=(k=='step',),daemon=True).start()
            r.success=True;r.message=k+' accepted'
        except Exception as e:r.success=False;r.message=str(e)
        return r
    def load_recipe(self):
        key=self.get_parameter('recipe').value
        if Path(key).name!=key:raise ValueError('invalid recipe name')
        p=DATA/(key+'.json');self.params=json.loads((p if p.exists() else ROOT/'config'/p.name).read_text())
        for k in ['infeed','good','fixture']:
            if len(self.params[k])!=3 or not np.isfinite(self.params[k]).all():raise ValueError('invalid station '+k)
        self.ready=True
    def check(self):
        if self.abort or self.latched:raise RuntimeError(self.err or 'stopped')
    def wait(self,pred,seconds=5):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.check()
            if pred():return
            time.sleep(.008)
        raise TimeoutError('timeout during '+self.label)
    def call(self,client,req):
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError(client.srv_name+' unavailable')
        f=client.call_async(req);self.wait(f.done,15);return f.result()
    def state(self,q):
        s=RobotState(is_diff=True);s.joint_state=JointState(name=NAMES,position=list(map(float,q)));return s
    def setup_scene(self):
        req=ApplyPlanningScene.Request();sc=req.scene;sc.is_diff=sc.robot_state.is_diff=True
        # Bench and six-axis base touch by design. Fixtures enter MoveIt as envelope boxes.
        fixtures=json.loads((ROOT/'config/environment.json').read_text())
        sc.world.collision_objects=[box(e['name'],e['xyz'],e['size']) for e in fixtures]
        p=self.target.pose;o=p.orientation
        sc.world.collision_objects.append(box('pcb',[p.position.x,p.position.y,p.position.z],[.054,.030,.006],[o.x,o.y,o.z,o.w]))
        query=GetPlanningScene.Request();query.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        ac=self.call(self.scene,query).scene.allowed_collision_matrix
        for a,b in [('base','bench'),('l1','bench'),('pcb','left'),('pcb','right'),('pcb','infeed'),('pcb','good_tray'),('pcb','screw_nest')]:
            for name in (a,b):
                if name not in ac.entry_names:
                    ac.entry_names.append(name)
                    for row in ac.entry_values:row.enabled.append(False)
                    ac.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(ac.entry_names)))
            i,j=ac.entry_names.index(a),ac.entry_names.index(b);ac.entry_values[i].enabled[j]=ac.entry_values[j].enabled[i]=True
        sc.allowed_collision_matrix=ac
        if not self.call(self.apply,req).success:raise RuntimeError('scene init rejected')
    def attachment(self,held):
        req=ApplyPlanningScene.Request();req.scene.is_diff=req.scene.robot_state.is_diff=True
        a=AttachedCollisionObject(link_name='tcp',touch_links=['left','right','l6','tcp'])
        a.object.id='pcb';a.object.operation=a.object.ADD if held else a.object.REMOVE
        req.scene.robot_state.attached_collision_objects=[a]
        if not self.call(self.apply,req).success:raise RuntimeError('MoveIt attach/detach rejected')
    def snapshot(self,pending):
        atomic(self.path,dict(index=self.idx,cycle=self.cycle,pending=pending,recipe=self.params,src=self.src,yaw=self.yaw,
            returning=self.returning,held=bool(self.feedback['io']&1),q=self.feedback['q'].tolist(),jaw=self.jaw,io=self.io))
    def restore(self):
        d=json.loads(self.path.read_text())
        if d['pending']:raise ValueError('interrupted within a phase: inspect part and reset')
        if bool(self.feedback['io']&1)!=d['held'] or max(abs(self.feedback['q']-d['q']))>.035:raise ValueError('checkpoint disagrees with plant')
        self.idx=d['index'];self.cycle=d['cycle'];self.src=d['src'];self.params=d['recipe'];self.returning=d['returning']
        self.yaw=d['yaw'];self.jaw=d['jaw'];self.io=d['io'];self.ready=True
    def move(self,xyz,mode):
        q0=self.feedback['q'].copy();seed=q0.copy();start=self.kin.fk(q0);end=downward(xyz,self.yaw);goal=self.kin.ik(end,q0)
        if mode=='joint':
            req=GetMotionPlan.Request();m=req.motion_plan_request;m.group_name='arm';m.start_state=self.state(q0)
            m.allowed_planning_time=4.;m.num_planning_attempts=3;m.max_velocity_scaling_factor=.4;m.max_acceleration_scaling_factor=.4
            c=Constraints();c.joint_constraints=[JointConstraint(joint_name=j,position=float(v),tolerance_above=.001,tolerance_below=.001,weight=1.) for j,v in zip(NAMES,goal)]
            m.goal_constraints=[c];res=self.call(self.plan,req).motion_plan_response
            if res.error_code.val!=1:raise ValueError('OMPL returned '+str(res.error_code.val))
            tr=res.trajectory.joint_trajectory;idx=[tr.joint_names.index(j) for j in NAMES];path=[q0]+[np.array(p.positions)[idx] for p in tr.points]
        else:
            a=start[:3,3];b=end[:3,3];count=max(3,int(np.linalg.norm(b-a)/.002)+2)
            pts=circle(a,(a+b)/2+[0,0,.016],b,count) if mode=='arc' else np.linspace(a,b,count)
            path=[q0]
            for p in pts[1:]:
                q=self.kin.ik(downward(p,self.yaw),seed)
                if max(abs(q-seed))>.25:raise ValueError('discontinuous IK branch')
                path.append(q);seed=q
        v=self.get_parameter('speed').value or self.params['speed'];a=self.get_parameter('accel').value or self.params['accel']
        if not (.03<=v<=1.5 and .05<=a<=4):raise ValueError('speed / accel outside range')
        traj=profile(path,v,a);prev=None
        for index,q in enumerate(traj):
            if index<len(traj)-1 and prev is not None and max(abs(q-prev))<.025:continue
            if self.kin.sigma(q)<.0025:raise ValueError('path near singularity')
            req=GetStateValidity.Request();req.robot_state=self.state(q);req.group_name='arm'
            if not self.call(self.valid,req).valid:raise ValueError('MoveIt collision at '+self.label)
            prev=q
        view=DisplayTrajectory(model_id='c3_arm',trajectory_start=self.state(q0));rt=RobotTrajectory();rt.joint_trajectory.joint_names=NAMES
        for i in sorted(set([*range(0,len(traj),5),len(traj)-1])):
            point=JointTrajectoryPoint(positions=traj[i].tolist());point.time_from_start.sec=i//100;point.time_from_start.nanosec=(i%100)*10000000
            rt.joint_trajectory.points.append(point)
        view.trajectory=[rt];self.preview.publish(view)
        # Advance against feedback simulation time, never skip forward on a frozen plant.
        begin=self.feedback['tick'];index=0
        while index<len(traj):
            self.check();self.goal=traj[index];elapsed=(self.feedback['tick']-begin)&0xffffffff
            index=min(index+1,elapsed//10+1);time.sleep(.005)
        self.goal=traj[-1];self.wait(lambda:max(abs(self.feedback['q']-self.goal))<.004,8)
    def phase(self,k):
        p=self.params;lift=np.array([0,0,p['lift']])
        if k=='observe':
            self.wait(lambda:self.target is not None and time.monotonic()-self.seen<.3)
            obs=self.target.pose;self.src=[obs.position.x,obs.position.y,obs.position.z];o=obs.orientation
            self.yaw=Rotation.from_quat([o.x,o.y,o.z,o.w]).as_euler('xyz')[2]
            if not self.returning and not(self.feedback['io']&8):
                if np.linalg.norm(np.array(self.src)-p['good'])<.015:self.returning=True
                else:raise ValueError('infeed photoeye empty')
            self.setup_scene();return
        source=np.array(self.src);nest=np.array(p['fixture']);out=np.array(p['infeed'] if self.returning else p['good'])
        if self.returning and k in ('transfer','land','screw','up'):return
        if k in ['close','open']:
            hold=k=='close'
            if hold:self.jaw=0.;self.io=5
            else:self.io=4;self.jaw=.014
            self.wait(lambda:bool(self.feedback['io']&1)==hold and bool(self.feedback['io']&2))
            self.attachment(hold)
            if not hold:
                def deposited():
                    if self.target is None or time.monotonic()-self.seen>.3:return False
                    pos=self.target.pose.position
                    return np.linalg.norm(np.array([pos.x,pos.y,pos.z])-out)<.006
                self.wait(deposited,3.)
            return
        if k=='screw':
            self.io=1|4|16|32
            self.wait(lambda:(self.feedback['io']&48)==48)
            start=self.feedback['tick']
            while (self.feedback['tick']-start)&0xffffffff < int(p['hold_s']*1000):
                self.check()
                if self.feedback['io']&48!=48:raise ValueError('lost press/torque confirmation')
                time.sleep(.01)
            self.io=5;return
        positions={'prepick':source+lift,'pick':source,'lift':source+lift,'transfer':nest+lift,'land':nest,
                   'up':nest+lift,'to_good':out+lift,'lower':out,'retreat':out+lift}
        mode='joint' if k=='prepick' else ('arc' if p['use_arc'] and k=='to_good' else 'line')
        self.io=(self.io&1)|(4 if k in ('lift','land','up','lower','pick') else 0)
        self.move(positions[k],mode)
    def work(self,single):
        try:
            while True:
                self.label=STEPS[self.idx];self.record(phase=self.label,cycle=self.cycle);self.snapshot(True);self.phase(self.label)
                self.idx+=1
                if self.idx==len(STEPS):
                    self.idx=0
                    if not self.returning:self.cycle+=1
                    if not self.returning and self.cycle>=max(1,self.get_parameter('loops').value):
                        self.label='DONE';self.record(phase='DONE');self.path.unlink(missing_ok=True);self.ready=False;self.cycle=0;self.returning=False;break
                    self.returning=not self.returning
                self.snapshot(False)
                if single or self.stop:self.label='PAUSED';break
        except Exception as e:
            if self.feedback:self.goal=self.feedback['q'].copy()
            self.err=str(e);self.label='FAULT';self.latched=True;self.record(error=self.err)
        finally:self.running=False
if __name__=='__main__':
    rclpy.init();node=Station();rclpy.spin(node)
