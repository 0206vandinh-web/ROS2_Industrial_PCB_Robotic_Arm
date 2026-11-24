"""Host side of the firmware protocol and standard-DH kinematics."""
import binascii
import ctypes as C
import struct
import numpy as np
from scipy.interpolate import PchipInterpolator

FORMAT='<BBHI6iHHHB7x'
def packet(kind,seq,ms,q,jaw=0,io=0,fault=0,flags=1):
    data=struct.pack(FORMAT,1,kind,seq&65535,ms&0xffffffff,*np.rint(np.array(q)*1e6).astype(int),
                     round(jaw*1e6),io,fault,flags)
    return data+struct.pack('<H',binascii.crc_hqx(data,65535))
def unpack(data):
    if len(data)!=48 or binascii.crc_hqx(data[:46],65535)!=int.from_bytes(data[46:],'little'):
        raise ValueError('bad DLC/CRC')
    v=struct.unpack(FORMAT,data[:46])
    if v[0]!=1 or v[1] not in (1,2):raise ValueError('invalid version/type')
    return dict(kind=v[1],seq=v[2],tick=v[3],q=np.array(v[4:10])*1e-6,jaw=v[10]*1e-6,io=v[11],fault=v[12],flags=v[13])
class Kin:
    def __init__(self,path):
        self.lib=C.CDLL(str(path));p=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        self.lib.c3_fk.argtypes=[p,p];self.lib.c3_ik.argtypes=[p,p,p]
        self.lib.c3_sigma.argtypes=[p];self.lib.c3_sigma.restype=C.c_double
    def fk(self,q):
        t=np.zeros((4,4));self.lib.c3_fk(np.ascontiguousarray(q,dtype=float),t);return t
    def sigma(self,q):return self.lib.c3_sigma(np.ascontiguousarray(q,dtype=float))
    def ik(self,t,seed):
        t=np.ascontiguousarray(t,dtype=float)
        for s in [seed,[0,.9,-1.65,.1,.85,0],[0,.8,-1.6,2.,-1.3,1.0],[0,-.8,1.6,.1,-1.1,0]]:
            q=np.zeros(6)
            if self.lib.c3_ik(t,np.ascontiguousarray(s,dtype=float),q) and self.sigma(q)>.003:return q
        raise ValueError('IK: target unreachable or near singularity')
def downward(xyz,yaw=0.):
    c,s=np.cos(yaw),np.sin(yaw)
    t=np.array([[c,s,0,0],[s,-c,0,0],[0,0,-1,0],[0,0,0,1]],dtype=float);t[:3,3]=xyz;return t
def circle(a,b,c,n):
    """Circumcircle of start/via/end, with the sweep selected through via."""
    u=b-a;v=c-a;normal=np.cross(u,v);den=2*np.dot(normal,normal)
    if den<1e-14:raise ValueError('three arc points are collinear')
    center=a+(np.dot(u,u)*np.cross(v,normal)+np.dot(v,v)*np.cross(normal,u))/den
    nrm=normal/np.linalg.norm(normal);ex=a-center;r=np.linalg.norm(ex);ex=ex/r;ey=np.cross(nrm,ex)
    angle=lambda p:np.arctan2(np.dot(p-center,ey),np.dot(p-center,ex))%(2*np.pi)
    end=angle(c);end=end if angle(b)<=end else end-2*np.pi
    return [center+r*(ex*np.cos(t)+ey*np.sin(t)) for t in np.linspace(0,end,n)]
def profile(path,vmax,amax,hz=100):
    path=np.array(path);keep=np.r_[True,np.max(np.abs(np.diff(path,axis=0)),axis=1)>1e-8];path=path[keep]
    if len(path)==1:return path
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))];s/=s[-1]
    curve=PchipInterpolator(s,path,axis=0);t=np.linspace(0,1,1001);u=t**3*(10-15*t+6*t*t)
    p=curve(u);speed=np.gradient(p,.001,axis=0);acc=np.gradient(speed,.001,axis=0)
    dur=1.06*max(.35,np.max(abs(speed))/vmax,np.sqrt(np.max(abs(acc))/amax))
    t=np.linspace(0,1,int(np.ceil(hz*dur))+1);return curve(t**3*(10-15*t+6*t*t))
