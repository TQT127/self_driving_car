#ifndef __PS2_H
#define __PS2_H

#include "main.h"

#define DI 		HAL_GPIO_ReadPin(DAT_GPIO_Port,DAT_Pin)

#define DO_H 	HAL_GPIO_WritePin(CMD_GPIO_Port, CMD_Pin,GPIO_PIN_SET)
#define DO_L 	HAL_GPIO_WritePin(CMD_GPIO_Port, CMD_Pin,GPIO_PIN_RESET)

#define CS_H 	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin,GPIO_PIN_SET)
#define CS_L 	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin,GPIO_PIN_RESET)

#define CLK_H HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,GPIO_PIN_SET)
#define CLK_L HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin,GPIO_PIN_RESET)

#define PSB_SELECT		1
#define PSB_L3				2
#define PSB_R3				3
#define PSB_START			4
#define PSB_PAD_UP		5
#define PSB_PAD_RIGHT	6
#define PSB_PAD_DOWN	7
#define PSB_PAD_LEFT	8
#define PSB_L2				9
#define PSB_R2				10	
#define PSB_L1				11
#define PSB_R1				12
#define PSB_GREEN			13
#define PSB_RED				14
#define PSB_BLUE			15
#define PSB_PINK			16
#define PSB_TRIANGLE	13
#define PSB_CIRCLE		14
#define PSB_CROSS			15
#define PSB_SQUARE		16

#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

void delay_us(uint32_t us);

void PS2_Init(void);
uint8_t PS2_RedLight(void); 
void PS2_ReadData(void);
void PS2_Cmd(uint8_t CMD);
uint8_t PS2_DataKey(void);
uint8_t PS2_AnalogData(uint8_t button);
void PS2_ClearData(void);
void PS2_Vibration (uint8_t motorl,uint8_t motor2);
void PS2_EnterConfigMode(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
void PS2_ExitConfing(void);
void PS2_SetInit(void);
#endif
