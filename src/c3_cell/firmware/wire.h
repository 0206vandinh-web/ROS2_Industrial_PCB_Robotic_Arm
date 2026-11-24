#pragma once
#include <stdint.h>
#define C3_BYTES 48
#define C3_CMD 0x180
#define C3_STATE 0x280
enum {C3_STOP=1,C3_LIMIT=2,C3_TIMEOUT=4,C3_STALL=8,C3_POWER=16,C3_CONTACT=32,C3_RANGE=64,C3_CAN=128};
typedef struct {uint16_t seq;uint32_t tick;int32_t axis[6];uint16_t jaw_um,io,fault;uint8_t kind,flags;} C3Frame;
uint16_t c3_crc(const uint8_t *p,unsigned n);
int c3_unpack(const uint8_t p[C3_BYTES],C3Frame *out);
void c3_pack(const C3Frame *in,uint8_t p[C3_BYTES]);
