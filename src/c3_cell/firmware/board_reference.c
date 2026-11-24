/* Board binding template for the reference C3-SERVO firmware. CubeMX main.h supplies
 * the pins below. ADC DMA buffer is owned by the application, in coherent DMA memory. */
#include "board_port.h"
#include "wire.h"
#include <string.h>
#include <math.h>
extern FDCAN_HandleTypeDef hfdcan2;
extern volatile uint16_t c3_adc_dma[2];
/* Set both from the measured analogue front end, counts to volts / amps. */
#ifndef C3_BUS_VOLTS_PER_COUNT
#error Define C3_BUS_VOLTS_PER_COUNT from the board calibration
#endif
#ifndef C3_BUS_AMPS_PER_COUNT
#error Define C3_BUS_AMPS_PER_COUNT and C3_BUS_CURRENT_ZERO from board calibration
#endif
static double q[6],v[6],amp[6],jaw;
static uint16_t tool_io;static uint32_t stamp[7];static unsigned seen;
static uint32_t u32(const uint8_t *b){return (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);}
unsigned board_trip_input(uint16_t pin){
 if(pin==ESTOP_Pin)return C3_STOP;
 if(pin==LIMIT_ANY_Pin)return C3_LIMIT;
 return 0;
}
void board_disable_outputs_isr(void){DRIVE_ENABLE_GPIO_Port->BSRR=(uint32_t)DRIVE_ENABLE_Pin<<16;}
void board_disable_outputs(void){board_disable_outputs_isr();}
void board_sample(C3Inputs *out){
 memset(out,0,sizeof(*out));uint32_t now=HAL_GetTick();uint8_t data[64];FDCAN_RxHeaderTypeDef h;
 for(unsigned k=0;k<32 && HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2,FDCAN_RX_FIFO0);k++){
  if(HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO0,&h,data)!=HAL_OK)break;
  if(h.Identifier<0x281||h.Identifier>0x287||h.DataLength!=FDCAN_DLC_BYTES_16)continue;
  if(c3_crc(data,14)!=(uint16_t)(data[14]|(data[15]<<8)))continue;
  unsigned i=h.Identifier-0x281;stamp[i]=now;seen|=1u<<i;
  if(i<6){q[i]=(int32_t)u32(data)*1.e-6;v[i]=(int32_t)u32(data+4)*1.e-6;amp[i]=(int16_t)(data[8]|(data[9]<<8))*.001;}
  else{jaw=(data[0]|(data[1]<<8))*1.e-6;tool_io=data[2]|(data[3]<<8);}
 }
 for(unsigned i=0;i<6;i++){out->q[i]=q[i];out->amp[i]=amp[i];out->velocity[i]=v[i];
  if((seen&(1u<<i)) && now-stamp[i]<40)out->axis_online|=1u<<i;}
 out->jaw_m=jaw;out->io=tool_io;
 if(!(seen&64)||now-stamp[6]>=50)out->axis_online=0;
 out->bus_amps=((double)c3_adc_dma[1]-C3_BUS_CURRENT_ZERO)*C3_BUS_AMPS_PER_COUNT;
 out->volts=c3_adc_dma[0]*C3_BUS_VOLTS_PER_COUNT;
 out->estop=HAL_GPIO_ReadPin(ESTOP_GPIO_Port,ESTOP_Pin)==GPIO_PIN_SET;
 out->limits=HAL_GPIO_ReadPin(LIMIT_ANY_GPIO_Port,LIMIT_ANY_Pin)==GPIO_PIN_SET;
 if(HAL_GPIO_ReadPin(INFEED_GPIO_Port,INFEED_Pin)==GPIO_PIN_SET)out->io|=8;
}
