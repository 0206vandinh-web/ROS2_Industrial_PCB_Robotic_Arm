#include "control.h"
#include <assert.h>
#include <stdio.h>
int main(void){
 C3Inputs x={.volts=24,.axis_online=63};C3Control c={0};double vel[6];uint8_t b[48];
 C3Frame f={.kind=1,.seq=65535,.tick=800,.axis={0,100000,-200000,300000,0,500000},.jaw_um=14000,.io=8,.flags=1};
 c3_pack(&f,b);for(int i=0;i<48;i++)printf("%02x",b[i]);puts("");
 assert(c3_receive(&c,b,0));assert(!c3_receive(&c,b,1));b[4]^=1;assert(!c3_receive(&c,b,2));
 f.seq=0;c3_pack(&f,b);assert(c3_receive(&c,b,3));c3_tick(&c,&x,3,vel);assert(vel[1]>0);
 c3_tick(&c,&x,104,vel);assert((c.fault&C3_TIMEOUT)&&!c.armed&&vel[1]==0);
 c=(C3Control){0};x.estop=1;c3_tick(&c,&x,0,vel);assert(c.fault&C3_STOP);x.estop=0;c3_tick(&c,&x,1,vel);assert(c.fault&C3_STOP);
 c=(C3Control){0};x.amp[2]=2.;for(unsigned n=0;n<205;n++){f.seq++;c3_pack(&f,b);assert(c3_receive(&c,b,n));c3_tick(&c,&x,n,vel);}assert(c.fault&C3_STALL);
 x.limits=1;x.volts=30;x.axis_online=31;c3_tick(&c,&x,205,vel);assert((c.fault&(C3_LIMIT|C3_POWER|C3_CAN))==(C3_LIMIT|C3_POWER|C3_CAN));
 return 0;
}
