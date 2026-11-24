/* Closed-loop bench: 100 Hz command frames to 1 kHz controller and a lagged velocity plant. */
#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 C3Control ctl={0}; C3Inputs x={.volts=24,.axis_online=63};
 double plant_v[6]={0},track=0; C3Frame f={.kind=1,.flags=1};uint8_t b[48];
 for(unsigned ms=0;ms<4500;ms++){
  if(ms%10==0){double u=fmin(1.,ms/3000.);double s=u*u*u*(10+u*(-15+6*u));
   for(int i=0;i<6;i++)f.axis[i]=(int32_t)lrint((i%2?-1.:1.)*.6*s*1.e6);
   f.seq++;f.tick=ms;c3_pack(&f,b);assert(c3_receive(&ctl,b,ms));}
  double v[6];c3_tick(&ctl,&x,ms,v);assert(!ctl.fault);
  for(int i=0;i<6;i++){plant_v[i]+=(v[i]-plant_v[i])/13.;
   x.q[i]+=.001*plant_v[i];x.velocity[i]=plant_v[i];
   track=fmax(track,fabs(x.q[i]-f.axis[i]*1.e-6));}
 }
 for(int i=0;i<6;i++)assert(fabs(x.q[i]-f.axis[i]*1.e-6)<.0001);
 printf("PASS: 4.5 s host servo bench, six axes settled; peak dynamic error %.5f rad\n",track);
}
