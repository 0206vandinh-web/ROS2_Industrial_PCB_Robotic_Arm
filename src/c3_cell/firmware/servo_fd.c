/* Reference downstream servo protocol. Use only with drives implementing C3-SERVO.
 * Each drive buffers its velocity setpoint by epoch and applies it on SYNC.
 * A failed enqueue never emits a commit for a partial epoch. */
#include "board_port.h"
#include <math.h>
#include "wire.h"
static int push(FDCAN_HandleTypeDef *bus,unsigned id,uint8_t p[8]){
 FDCAN_TxHeaderTypeDef h={0};h.Identifier=id;h.IdType=FDCAN_STANDARD_ID;
 h.TxFrameType=FDCAN_DATA_FRAME;h.DataLength=FDCAN_DLC_BYTES_8;h.FDFormat=FDCAN_FD_CAN;
 h.BitRateSwitch=FDCAN_BRS_ON;h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
 uint16_t crc=c3_crc(p,6);p[6]=crc;p[7]=crc>>8;
 return HAL_FDCAN_AddMessageToTxFifoQ(bus,&h,p)==HAL_OK;
}
int board_stage_six_and_sync(FDCAN_HandleTypeDef *bus,const double rad_s[6],uint16_t jaw,uint16_t io){
 static uint16_t epoch;epoch++;
 for(unsigned i=0;i<6;i++){
  uint32_t v=(uint32_t)(int32_t)lrint(rad_s[i]*1000000.);
  uint8_t b[8]={epoch,epoch>>8,v,v>>8,v>>16,v>>24};
  if(!push(bus,0x201+i,b))return 0;
 }
 uint8_t tool[8]={epoch,epoch>>8,jaw,jaw>>8,io,io>>8};
 if(!push(bus,0x207,tool))return 0;
 uint8_t sync[8]={epoch,epoch>>8,0,0,0,0};return push(bus,0x080,sync);
}
