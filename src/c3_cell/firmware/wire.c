#include "wire.h"
#include <string.h>
static unsigned get(const uint8_t *b,int n){unsigned x=0;for(int i=0;i<n;i++)x|=(unsigned)b[i]<<(8*i);return x;}
static void put(uint8_t *b,unsigned x,int n){for(int i=0;i<n;i++)b[i]=(uint8_t)(x>>(8*i));}
uint16_t c3_crc(const uint8_t *p,unsigned n){uint16_t c=65535;while(n--){c^=(uint16_t)(*p++)<<8;for(int i=0;i<8;i++)c=(c&32768)?(c<<1)^0x1021:c<<1;}return c;}
void c3_pack(const C3Frame *f,uint8_t p[48]){
 memset(p,0,48);p[0]=1;p[1]=f->kind;put(p+2,f->seq,2);put(p+4,f->tick,4);
 for(int i=0;i<6;i++)put(p+8+4*i,(uint32_t)f->axis[i],4);
 put(p+32,f->jaw_um,2);put(p+34,f->io,2);put(p+36,f->fault,2);p[38]=f->flags;put(p+46,c3_crc(p,46),2);
}
int c3_unpack(const uint8_t p[48],C3Frame *f){
 if(p[0]!=1 || (p[1]!=1 && p[1]!=2) || c3_crc(p,46)!=get(p+46,2))return 0;
 f->kind=p[1];f->seq=get(p+2,2);f->tick=get(p+4,4);
 for(int i=0;i<6;i++)f->axis[i]=(int32_t)get(p+8+4*i,4);
 f->jaw_um=get(p+32,2);f->io=get(p+34,2);f->fault=get(p+36,2);f->flags=p[38];return 1;
}
