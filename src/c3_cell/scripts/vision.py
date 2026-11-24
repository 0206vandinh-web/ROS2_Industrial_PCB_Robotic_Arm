#!/usr/bin/env python3
"""Simulation supplies measured PCB pose; external vision supplies calibrated camera-frame pose."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation
import numpy as np

class Vision(Node):
    def __init__(self):
        super().__init__('c3_vision')
        self.declare_parameter('external',False)
        self.declare_parameter('camera_xyz',[0.,0.,0.])
        self.declare_parameter('camera_xyzw',[0.,0.,0.,1.])
        external=self.get_parameter('external').value
        self.source='camera_optical' if external else 'world'
        self.r=Rotation.from_quat(self.get_parameter('camera_xyzw').value)
        self.xyz=np.array(self.get_parameter('camera_xyz').value)
        self.rng=np.random.default_rng(743)
        self.pixels=self.create_publisher(Float64MultiArray,'/c3/vision_pixel',10)
        self.pub=self.create_publisher(PoseStamped,'/c3/target',10)
        self.create_subscription(PoseStamped,'/camera/pcb_pose' if external else '/c3/observed_pcb',self.convert,10)
    def convert(self,m):
        if m.header.frame_id!=self.source:return
        if self.source!='world':
            p=m.pose.position;o=m.pose.orientation
            p.x,p.y,p.z=self.r.apply([p.x,p.y,p.z])+self.xyz
            o.x,o.y,o.z,o.w=(self.r*Rotation.from_quat([o.x,o.y,o.z,o.w])).as_quat()
        else:
            # Synthetic overhead fiducial camera: project the measured board centre, sample
            # sub-pixel noise, then back-project at the board's simulated plane height.
            p=m.pose.position;depth=.65-p.z
            if depth<=0:return
            u=640+850*(p.x-.23)/depth+self.rng.normal(0,.06)
            v=360-850*p.y/depth+self.rng.normal(0,.06)
            if not (0<=u<1280 and 0<=v<720):return
            self.pixels.publish(Float64MultiArray(data=[float(u),float(v),float(depth)]))
            p.x=.23+(u-640)*depth/850;p.y=-(v-360)*depth/850
        m.header.frame_id='world';self.pub.publish(m)
if __name__=='__main__':
    rclpy.init();n=Vision();rclpy.spin(n)
