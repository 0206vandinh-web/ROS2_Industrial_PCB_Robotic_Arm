#!/usr/bin/env python3
"""Linux SocketCAN-FD endpoint; configure netdev of Ethernet/CAN-FD gateway first."""
import socket,struct,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
from core import unpack
class Gateway(Node):
    def __init__(self):
        super().__init__('c3_gateway');self.declare_parameter('interface','can0')
        self.sock=None;self.retry=0;self.seq=None;self.last=0.
        self.rx=self.create_publisher(UInt8MultiArray,'/c3/rx',10);self.j=self.create_publisher(JointState,'/joint_states',10)
        self.create_subscription(UInt8MultiArray,'/c3/tx',self.send,1);self.create_timer(.002,self.poll)
    def down(self):
        if self.sock:self.sock.close()
        self.sock=None;self.retry=time.monotonic()+1.;self.seq=None
    def send(self,m):
        if not self.sock:return
        try:
            if unpack(bytes(m.data))['kind']==1:self.sock.send(struct.pack('=IBB2x64s',0x180,48,1,bytes(m.data)))
        except (OSError,ValueError):self.down()
    def poll(self):
        if not self.sock:
            if time.monotonic()<self.retry:return
            try:
                self.sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
                self.sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FD_FRAMES,1)
                self.sock.bind((self.get_parameter('interface').value,));self.sock.setblocking(False);self.last=time.monotonic()
            except OSError:self.down()
            return
        try:
            for _ in range(32):
                data=self.sock.recv(72)
                if len(data)!=72:continue
                ident,n,_,buf=struct.unpack('=IBB2x64s',data)
                if ident!=0x280 or n!=48:continue
                f=unpack(buf[:48])
                if f['kind']!=2 or self.seq is not None and not 0<(f['seq']-self.seq)%65536<32768:continue
                self.seq=f['seq'];self.last=time.monotonic();self.rx.publish(UInt8MultiArray(data=list(buf[:48])))
                m=JointState();m.header.stamp=self.get_clock().now().to_msg();m.name=[f'j{i}' for i in range(1,7)]+['left_joint','right_joint'];m.position=[*f['q'],f['jaw'],f['jaw']];self.j.publish(m)
        except BlockingIOError:pass
        except ValueError:pass
        except OSError:self.down()
        if self.sock and time.monotonic()-self.last>1:self.down()
if __name__=='__main__':
    rclpy.init();rclpy.spin(Gateway())
