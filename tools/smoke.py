#!/usr/bin/env python3
"""Standalone native verification, including independent FK, singularity and interpolation tests."""
import ctypes as C,json,sys,subprocess,tempfile
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];PKG=ROOT/'src/c3_cell';FW=PKG/'firmware';sys.path.insert(0,str(PKG/'scripts'))
from core import *
cfg=json.loads((PKG/'config/arm.json').read_text());U=E.parse(PKG/'models/arm.urdf');S=E.parse(PKG/'models/workcell.sdf')
def urdfk(q):
    t=np.eye(4);links={'world':t}
    for j in U.findall('joint'):
        o=j.find('origin');h=np.eye(4);h[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy'),sep=' ')).as_matrix();h[:3,3]=np.fromstring(o.get('xyz'),sep=' ')
        if j.get('type')=='revolute':h[:3,:3]=h[:3,:3]@Rotation.from_rotvec([0,0,q[int(j.get('name')[1:])-1]]).as_matrix()
        links[j.find('child').get('link')]=links[j.find('parent').get('link')]@h
    return links
def run():
    with tempfile.TemporaryDirectory() as d:
        lib=Path(d)/'lib.so';subprocess.run(['cc','-std=c11','-Wall','-Werror','-fPIC','-shared',FW/'kin.c',FW/'control.c',FW/'wire.c','-lm','-o',lib],check=True)
        ctl=Path(d)/'test';subprocess.run(['cc','-std=c11','-Ifake','-I'+str(FW),ROOT/'tools/test_control.c',FW/'control.c',FW/'wire.c','-lm','-o',ctl],check=True)
        data=bytes.fromhex(subprocess.check_output([ctl],text=True).strip())
        assert data==packet(1,65535,800,[0,.1,-.2,.3,0,.5],.014,8)
        print('PASS: C/Python CAN-FD byte agreement, CRC, sequence rollover, timeout, estop, stall, limits and power')
        servo=Path(d)/'servo'
        subprocess.run(['cc','-std=c11','-I'+str(FW),ROOT/'tools/test_servo.c',FW/'control.c',FW/'wire.c','-lm','-o',servo],check=True)
        subprocess.run([servo],check=True)
        kin=Kin(lib);rng=np.random.default_rng(723)
        for q in rng.uniform(-1.7,1.7,(160,6)):
            assert np.max(abs(urdfk(q)['tcp']-kin.fk(q)))<2e-8
        assert kin.sigma([0]*6)<1e-6
        print('PASS: URDF vs C FK across 160 configurations; singular configuration detected')
        assert len(S.findall(".//model[@name='arm']/joint[@type='revolute']"))==6
        assert {x.get('name') for x in S.findall('./world/model')} >= {'pcb','screw_nest','good_tray','infeed','conveyor'}
        seed=cfg['home'];count=0;worst=0.;min_sigma=1
        for recipe in ('assembly','soft'):
            p=json.loads((PKG/'config'/f'{recipe}.json').read_text());up=np.array([0.,0.,p['lift']])
            a,b,c=[np.array(p[n]) for n in ['infeed','fixture','good']]
            path=[a+up,a,a+up,b+up,b,b+up,c+up,c,c+up,a+up]
            for i,(s,t) in enumerate(zip(path,path[1:])):
                xyz=circle(s,(s+t)/2+[0,0,.016],t,35) if p['use_arc'] and i==5 else np.linspace(s,t,35)
                qs=[]
                for pos in xyz:
                    seed=kin.ik(downward(pos),seed);qs.append(seed);count+=1
                    worst=max(worst,np.linalg.norm(kin.fk(seed)[:3,3]-pos));min_sigma=min(min_sigma,kin.sigma(seed))
                tr=profile(qs,p['speed'],p['accel'])
                assert np.max(abs(np.diff(tr,axis=0)))*100 < p['speed']*1.03
                assert np.max(abs(np.diff(tr,n=2,axis=0)))*10000 < p['accel']*1.05
        print(f'PASS: {count} Cartesian IK targets; max numerical residual {worst*1e3:.4f} mm; min sigma {min_sigma:.4f}; S timing speed/acceleration')
if __name__=='__main__':run()
