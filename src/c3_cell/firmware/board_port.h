#pragma once
#include "control.h"
#include "main.h"
/* Board contract. Implement against the selected servo's manual and electrical design. */
unsigned board_trip_input(uint16_t pin);
void board_disable_outputs_isr(void);
void board_disable_outputs(void);
void board_sample(C3Inputs *inputs);
int board_stage_six_and_sync(FDCAN_HandleTypeDef *bus,const double rad_per_sec[6],uint16_t jaw_um,uint16_t io);
