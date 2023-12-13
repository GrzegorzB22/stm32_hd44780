/*
 * hd44780.h
 *
 *  Created on: Dec 2, 2023
 *      Author: linadmin
 */

#ifndef INC_HD4478044780_H_
#define INC_HD4478044780_H_

#include "main.h"
#include "tim.h"

#define HD44780_USE_RW       0
#define HD44780_USE_CONTRAST 1

#define delay_us_timer htim3
extern TIM_HandleTypeDef delay_us_timer;

#define contrast_timer         htim4
#define contrast_timer_channel TIM_CHANNEL_1
extern TIM_HandleTypeDef contrast_timer;

#define HD44780_BUSY_FLAG 0x80

#define HD44780_WIDTH  16
#define HD44780_HEIGHT 2

#define HD44780_LINE_1_ADDRESS 0x00
#define HD44780_LINE_2_ADDRESS 0x40

#define HD44780_ENTRY_ENTER     0x04
#define HD44780_DISPLAY_SHIFT   0x01
#define HD44780_SHIFT_DIRECTION 0x02

#define HD44780_DISPLAY_ENTER 0x08
#define HD44780_CURSOR_BLINK  0x01
#define HD44780_CURSOR_STATE  0x02
#define HD44780_DISPLAY_STATE 0x04

#define HD44780_FUNCTION_ENTER 0x20
#define HD44780_FONT_SIZE      0x04
#define HD44780_LINES_NUMBER   0x08

#define HD44780_SET_DDRAM_ADDRESS 0x80
#define HD44780_SET_CGRAM_ADDRESS 0x40

#define HD44780_CLEAR 0x01
#define HD44780_HOME  0x02

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t screen_buffer[HD44780_HEIGHT][HD44780_WIDTH];
} hd44780_data;

typedef struct {
	uint8_t entry_mode;
	uint8_t display_control;
	uint8_t function;
} hd44780_settings;

typedef struct {
	hd44780_data data;
	hd44780_settings config;
} hd44780_chip;

#define HD44780_START_PROCESS HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_SET)
#define HD44780_STOP_PROCESS  HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET)

#define HD44780_SET_WRITE     HAL_GPIO_WritePin(HD44780_RW_PORT, HD44780_RW_PIN, GPIO_PIN_RESET)
#define HD44780_SET_READ      HAL_GPIO_WritePin(HD44780_RW_PORT, HD44780_RW_PIN, GPIO_PIN_SET)

#define HD44780_SET_COMMAND   HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_RESET)
#define HD44780_SET_DATA      HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_SET)

void HD44780_Init(void);
void HD44780_ClearBuffer(void);
void HD44780_WriteChar(char c);
void HD44780_WriteString(char *s);
void HD44780_SetCursor(uint8_t x, uint8_t y);
void HD44780_Update(void);

#if HD44780_USE_CONTRAST == 1
void HD44780_SetContrast(uint8_t percent);
#endif

#endif /* INC_HD4478044780_H_ */
