#pragma once
#include "wire.h"
typedef struct {double q[6],velocity[6],amp[6],volts,bus_amps,jaw_m;uint16_t io;uint8_t axis_online,estop,limits;} C3Inputs;
typedef struct {C3Frame cmd;uint32_t last_ms;uint16_t fault;uint8_t armed,seen;unsigned jam_ms[6];double filtered[6];} C3Control;
int c3_receive(C3Control *c,const uint8_t bytes[48],uint32_t ms);
void c3_tick(C3Control *c,const C3Inputs *in,uint32_t ms,double speed[6]);
