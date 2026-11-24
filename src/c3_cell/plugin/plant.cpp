// A world system. Commands and feedback use the same 48-byte frames as the MCU.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
extern "C" {
#include "control.h"
}
namespace c3 {
namespace sim=gz::sim; namespace cp=sim::components;
using Packet=std_msgs::msg::UInt8MultiArray;
class Plant:public sim::System,public sim::ISystemConfigure,public sim::ISystemPreUpdate {
 rclcpp::Context::SharedPtr context; rclcpp::Node::SharedPtr node;
 std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
 rclcpp::Subscription<Packet>::SharedPtr sub;
 rclcpp::Publisher<Packet>::SharedPtr fb; rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js;
 rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detail;
 sim::Entity robot=0,pcb=0,tool=0,joint=0;
 std::array<sim::Entity,8> axes{};std::array<std::string,8> names{"j1","j2","j3","j4","j5","j6","left_joint","right_joint"};
 C3Control ctl{};std::array<double,8> last{},velocity{}; uint32_t now=0;unsigned pads=0;double screw_time=0;bool have=false;uint16_t seq=0;
 bool setup(sim::EntityComponentManager &e){
  robot=e.EntityByComponents(cp::Model(),cp::Name("arm")); auto m=e.EntityByComponents(cp::Model(),cp::Name("pcb"));
  if(!robot||!m)return false;
  sim::Model arm(robot);pcb=sim::Model(m).LinkByName(e,"body");tool=arm.LinkByName(e,"tcp");
  for(size_t i=0;i<8;i++){axes[i]=arm.JointByName(e,names[i]);if(!axes[i])return false;
   if(!e.Component<cp::JointPosition>(axes[i]))e.CreateComponent(axes[i],cp::JointPosition());}
  if(!pcb||!tool)return false;
  const double initial[8]={-.55,1.28,-1.45,.12,.65,0,.014,.014};
  for(int i=0;i<8;i++)e.SetComponentData<cp::JointPositionReset>(axes[i],std::vector<double>{initial[i]});
  last={-.55,1.28,-1.45,.12,.65,0,.014,.014};return true;
 }
 void contacts(sim::EntityComponentManager &e){
  e.Each<cp::ContactSensorData>([&](const sim::Entity &,const cp::ContactSensorData *s){
   for(auto &c:s->Data().contact()){
    auto link=[&](uint64_t id){auto p=e.Component<cp::ParentEntity>(id);
     auto n=p?e.Component<cp::Name>(p->Data()):nullptr;
     auto m=p?e.Component<cp::ParentEntity>(p->Data()):nullptr;
     auto mn=m?e.Component<cp::Name>(m->Data()):nullptr;
     return n&&mn?mn->Data()+"/"+n->Data():std::string();};
    auto a=link(c.collision1().id()),b=link(c.collision2().id());
    if(a.empty()||b.empty()||a==b)continue;
    if(a>b)std::swap(a,b);
    bool finger=a=="arm/left"||a=="arm/right";
    if(finger && b=="pcb/body" && (ctl.cmd.io&1))pads|=a=="arm/left"?1:2;
    bool expected=(finger&&b=="pcb/body")||((a=="arm/base"||a=="arm/l1")&&b=="bench/body");
    // Wrist housings share a bearing. Their cosmetic surfaces and immediate neighbours are exempt.
    if(a.rfind("arm/",0)==0 && b.rfind("arm/",0)==0){
     auto near=[](const std::string& x){return x=="arm/l3"||x=="arm/l4"||x=="arm/l5"||x=="arm/l6";};
     if(near(a)&&near(b))expected=true;
    }
    if(a.rfind("arm/",0)!=0 && b.rfind("arm/",0)!=0){
     if(!joint || (a!="pcb/body" && b!="pcb/body"))continue;
     if((ctl.cmd.io&4) && (a=="good_tray/body"||a=="screw_nest/body"||a=="infeed/body"||b=="screw_nest/body"))expected=true;
    }
    if(!expected && !(ctl.fault&C3_CONTACT)){ctl.fault|=C3_CONTACT;std_msgs::msg::String msg;msg.data=a+" <> "+b;detail->publish(msg);}
   }return true;});
 }
public:
 void Configure(const sim::Entity &,const std::shared_ptr<const sdf::Element>&,sim::EntityComponentManager&,sim::EventManager&)override{
  context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);rclcpp::NodeOptions opt;opt.context(context);
  node=std::make_shared<rclcpp::Node>("c3_gazebo_io",opt);rclcpp::ExecutorOptions eo;eo.context=context;
  exec=std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);exec->add_node(node);
  sub=node->create_subscription<Packet>("/c3/tx",1,[this](Packet::ConstSharedPtr p){if(p->data.size()==48)c3_receive(&ctl,p->data.data(),now);});
  fb=node->create_publisher<Packet>("/c3/rx",10);js=node->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
  clock=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);vision=node->create_publisher<geometry_msgs::msg::PoseStamped>("/c3/observed_pcb",10);
  detail=node->create_publisher<std_msgs::msg::String>("/c3/contact_pair",rclcpp::QoS(1).transient_local());
 }
 void PreUpdate(const sim::UpdateInfo &info,sim::EntityComponentManager &e)override{
  if(info.paused)return;
  auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();now=ns/1000000;exec->spin_some();
  if(!have){have=setup(e);return;}
  C3Inputs x{};x.volts=24;x.axis_online=63;std::array<double,8> q{};double dt=std::max(1.e-5,std::chrono::duration<double>(info.dt).count());
  for(int i=0;i<8;i++){auto v=e.Component<cp::JointPosition>(axes[i]);if(!v||v->Data().empty())return;q[i]=v->Data()[0];}
  for(int i=0;i<6;i++){x.q[i]=q[i];x.velocity[i]=(q[i]-last[i])/dt;x.amp[i]=std::min(3.,8.*std::abs(ctl.cmd.axis[i]*1e-6-q[i]));}
  x.jaw_m=(q[6]+q[7])/2;
  if(!(ctl.cmd.io&1))pads=0;
  if(ctl.armed)contacts(e);
  double speed[6];c3_tick(&ctl,&x,now,speed);
  auto pp=sim::worldPose(pcb,e),tp=sim::worldPose(tool,e);
  double jaw=ctl.cmd.jaw_um*1e-6;bool jaws=(std::abs(q[6]-jaw)<.0006 || (pads&1)) && (std::abs(q[7]-jaw)<.0006 || (pads&2));
  if(ctl.armed && !ctl.fault){
   if((ctl.cmd.io&1)&&!joint&&jaws&&jaw<.001 && tp.Pos().Distance(pp.Pos())<.008){
    joint=e.CreateEntity();e.CreateComponent(joint,cp::DetachableJoint({tool,pcb,"fixed"}));}
   if(!(ctl.cmd.io&1)&&joint){e.RequestRemoveEntity(joint);joint=0;}
  }
  for(int i=0;i<8;i++){
   double target=i<6?speed[i]+.0003*std::sin(now*.037+i):std::clamp(16*(jaw-q[i]),-.065,.065);
   if(i>=6 && (pads&(1u<<(i-6))) && target<0)target=0;
   velocity[i]+=dt/(.012+dt)*(target-velocity[i]);if(!ctl.armed||ctl.fault)velocity[i]=0;
   e.SetComponentData<cp::JointVelocityCmd>(axes[i],std::vector<double>{velocity[i]});
  }
  // Virtual compliant fixture: spring compression and torque rise from dwell.
  const bool in_nest=std::hypot(pp.Pos().X()-.30,pp.Pos().Y()-.015)<.009;
  const double newtons=joint && in_nest?std::clamp((.100-pp.Pos().Z())*1600.,0.,12.):0.;
  const bool pressing=ctl.armed && (ctl.cmd.io&16) && newtons>=.8 && newtons<=8.;
  screw_time=pressing && (ctl.cmd.io&32)?screw_time+dt:0.;
  if(now%10==0){auto stamp=rclcpp::Time(ns);rosgraph_msgs::msg::Clock ck;ck.clock=stamp;clock->publish(ck);
   sensor_msgs::msg::JointState msg;msg.header.stamp=stamp;msg.name.assign(names.begin(),names.end());msg.position.assign(q.begin(),q.end());
   for(int i=0;i<8;i++)msg.velocity.push_back((q[i]-last[i])/dt);js->publish(msg);
   C3Frame f{};f.kind=2;f.seq=++seq;f.tick=now;f.jaw_um=std::clamp(x.jaw_m*1e6,0.,16000.);f.fault=ctl.fault;
   f.io=(pressing?16:0)|(screw_time>.35?32:0)|(joint?1:0)|(jaws?2:0)|(pp.Pos().Distance(gz::math::Vector3d(.235,-.18,.079))<.025?8:0);
   for(int i=0;i<6;i++)f.axis[i]=std::lround(q[i]*1e6);
   Packet pkt;pkt.data.resize(48);c3_pack(&f,pkt.data.data());fb->publish(pkt);
   geometry_msgs::msg::PoseStamped v;v.header.stamp=stamp;v.header.frame_id="world";
   v.pose.position.x=pp.Pos().X();v.pose.position.y=pp.Pos().Y();v.pose.position.z=pp.Pos().Z();
   v.pose.orientation.x=pp.Rot().X();v.pose.orientation.y=pp.Rot().Y();v.pose.orientation.z=pp.Rot().Z();v.pose.orientation.w=pp.Rot().W();vision->publish(v);
  }last=q;
 }
};}
GZ_ADD_PLUGIN(c3::Plant,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
