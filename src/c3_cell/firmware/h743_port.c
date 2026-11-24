/* Include in a CubeMX STM32H743 + FreeRTOS project. Board-specific I/O is in board_port.h. */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "control.h"
#include "board_port.h"
#include <math.h>
#if configTICK_RATE_HZ != 1000
#error Configure FreeRTOS at 1000 Hz for this port
#endif
extern FDCAN_HandleTypeDef hfdcan1,hfdcan2;
static C3Control control;
static volatile unsigned tripped;
void HAL_GPIO_EXTI_Callback(uint16_t pin){
 unsigned cause=board_trip_input(pin);
 if(cause){board_disable_outputs_isr();tripped|=cause;}
}
static int send_fd(FDCAN_HandleTypeDef *bus,unsigned id,uint32_t dlc,uint8_t *p){
 FDCAN_TxHeaderTypeDef h={0};h.Identifier=id;h.IdType=FDCAN_STANDARD_ID;h.TxFrameType=FDCAN_DATA_FRAME;
 h.DataLength=dlc;h.FDFormat=FDCAN_FD_CAN;h.BitRateSwitch=FDCAN_BRS_ON;h.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
 return HAL_FDCAN_AddMessageToTxFifoQ(bus,&h,p)==HAL_OK;
}
void c3_motion_task(void *arg){
 (void)arg;TickType_t wake=xTaskGetTickCount();uint16_t seq=0;
 for(;;){uint32_t now=HAL_GetTick();uint8_t p[64];FDCAN_RxHeaderTypeDef h;
  for(int i=0;i<8 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1,FDCAN_RX_FIFO0);i++)
   if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&h,p)==HAL_OK && h.Identifier==C3_CMD && h.DataLength==FDCAN_DLC_BYTES_48)c3_receive(&control,p,now);
  C3Inputs in={0};board_sample(&in);control.fault|=tripped;double speed[6];c3_tick(&control,&in,now,speed);
  if(!control.armed)board_disable_outputs();
  else if((seq%5)==0 && !board_stage_six_and_sync(&hfdcan2,speed,control.cmd.jaw_um,control.cmd.io)){control.fault|=C3_CAN;board_disable_outputs();}
  if((seq++%10)==0){C3Frame f={.kind=2,.seq=seq,.tick=now,.io=in.io,.fault=control.fault,.jaw_um=(uint16_t)(in.jaw_m*1e6)};
   for(int i=0;i<6;i++)f.axis[i]=(int32_t)lrint(in.q[i]*1e6);
   c3_pack(&f,p);if(!send_fd(&hfdcan1,C3_STATE,FDCAN_DLC_BYTES_48,p)){control.fault|=C3_CAN;board_disable_outputs();}}
  vTaskDelayUntil(&wake,pdMS_TO_TICKS(1));
 }
}
