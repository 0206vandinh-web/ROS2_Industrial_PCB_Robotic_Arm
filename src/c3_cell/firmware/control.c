#include "control.h"
#include "dh_values.h"
#include <math.h>
int c3_receive(C3Control *c,const uint8_t b[48],uint32_t ms){
 C3Frame f;if(!c3_unpack(b,&f)||f.kind!=1)return 0;
 uint16_t delta=(uint16_t)(f.seq-c->cmd.seq);
 if(c->seen && (delta==0 || delta>=32768))return 0;
 if(f.jaw_um>16000){c->fault|=C3_RANGE;return 0;}
 for(int i=0;i<6;i++)if(fabs(f.axis[i]*1e-6)>LIMIT[i]){c->fault|=C3_RANGE;return 0;}
 c->seen=1;c->last_ms=ms;c->cmd=f;if(f.flags&2)c->fault|=C3_STOP;
 c->armed=(f.flags&1)&&!c->fault;return 1;
}
void c3_tick(C3Control *c,const C3Inputs *x,uint32_t ms,double v[6]){
 if(x->estop)c->fault|=C3_STOP;
 if(x->limits)c->fault|=C3_LIMIT;
 if(x->axis_online!=63)c->fault|=C3_CAN;
 if(!isfinite(x->bus_amps)||fabs(x->bus_amps)>10.||!isfinite(x->volts)||x->volts>28||x->volts<19)c->fault|=C3_POWER;
 if(c->seen && ms-c->last_ms>100){c->fault|=C3_TIMEOUT;c->seen=0;}
 for(int i=0;i<6;i++){
  double e=c->cmd.axis[i]*1e-6-x->q[i];
  if(!isfinite(e)||!isfinite(x->amp[i])||!isfinite(x->velocity[i])||fabs(x->q[i])>LIMIT[i]+.005)c->fault|=C3_RANGE;
  if(fabs(x->amp[i])>4.0)c->fault|=C3_POWER;
  if(c->armed && fabs(e)>.12 && fabs(x->velocity[i])<.01 && fabs(x->amp[i])>1.0)c->jam_ms[i]++;else c->jam_ms[i]=0;
  if(c->jam_ms[i]>200)c->fault|=C3_STALL;
  double raw=fmax(-1.5,fmin(1.5,18.*e));c->filtered[i]+=.15*(raw-c->filtered[i]);
 }
 if(c->fault)c->armed=0;
 for(int i=0;i<6;i++){v[i]=c->armed?c->filtered[i]:0;if(!c->armed)c->filtered[i]=0;}
}
