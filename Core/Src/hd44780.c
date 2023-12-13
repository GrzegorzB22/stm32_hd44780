
#include "hd44780.h"
#include "pin_defines.h"
#include "main.h"
#include "tim.h"

#if HD44780_USE_RW == 1
#include <stdbool.h>
#endif

#if HD44780_USE_CONTRAST == 1
#define TIMER_AUTORELOAD (__HAL_TIM_GET_AUTORELOAD(&contrast_timer) + 1)
#endif

static hd44780_chip lcd = {
		{0, 0, {{0}}},
		{
				HD44780_ENTRY_ENTER | HD44780_SHIFT_DIRECTION,
				HD44780_DISPLAY_ENTER | HD44780_CURSOR_STATE | HD44780_DISPLAY_STATE,
				HD44780_FUNCTION_ENTER | HD44780_LINES_NUMBER
		}
};

static GPIO_TypeDef *HD44780_DATA_PORTS[4] = {HD44780_DB4_PORT, HD44780_DB5_PORT, HD44780_DB6_PORT, HD44780_DB7_PORT};
static uint16_t HD44780_DATA_PINS[4] = {HD44780_DB4_PIN, HD44780_DB5_PIN, HD44780_DB6_PIN, HD44780_DB7_PIN};

static void hd44780_data_out(void)
{
	GPIO_InitTypeDef GPIO_data_out;

	GPIO_data_out.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_data_out.Pin = GPIO_NOPULL;
	GPIO_data_out.Speed = GPIO_SPEED_FREQ_LOW;

	for (uint8_t i = 0; i < 4; i++) {
		GPIO_data_out.Pin = HD44780_DATA_PINS[i];
		HAL_GPIO_Init(HD44780_DATA_PORTS[i], &GPIO_data_out);
	}
}

#if HD44780_USE_RW == 1
static void hd44780_data_in(void)
{
	GPIO_InitTypeDef GPIO_data_in;

	GPIO_data_in.Mode = GPIO_MODE_INPUT;
	GPIO_data_in.Pull = GPIO_PULLUP;
	GPIO_data_in.Speed = GPIO_SPEED_FREQ_LOW;

	for (uint8_t i = 0; i < 4; i++) {
		GPIO_data_in.Pin = HD44780_DATA_PINS[i];
		HAL_GPIO_Init(HD44780_DATA_PORTS[i], &GPIO_data_in);
	}
}

static uint8_t hd44780_read4bits(void)
{
	uint8_t bits = 0;

	for (uint8_t i = 0; i < 4; i++)
		bits |= (HAL_GPIO_ReadPin(HD44780_DATA_PORTS[i], HD44780_DATA_PINS[i]) << i);

	return bits;
}

static uint8_t hd44780_readByte(void)
{
	uint8_t byte = 0;

	HD44780_SET_READ;
	hd44780_data_in();

	HD44780_START_PROCESS;
	byte = (hd44780_read4bits() << 4);
	HD44780_STOP_PROCESS;

	HD44780_START_PROCESS;
	byte |= hd44780_read4bits();
	HD44780_STOP_PROCESS;

	return byte;
}

static bool hd44780_is_busy(void)
{
	HD44780_SET_COMMAND;
	return (hd44780_readByte() & HD44780_BUSY_FLAG);
}
#endif

static void hd44780_write4bits(uint8_t bits)
{
	for (uint8_t i = 0; i < 4; i++)
		HAL_GPIO_WritePin(HD44780_DATA_PORTS[i], HD44780_DATA_PINS[i], bits & (1 << i));
}

static void hd44780_writeByte(uint8_t byte)
{
	HD44780_SET_WRITE;
	hd44780_data_out();

	HD44780_START_PROCESS;
	hd44780_write4bits(byte >> 4);
	HD44780_STOP_PROCESS;

	HD44780_START_PROCESS;
	hd44780_write4bits(byte);
	HD44780_STOP_PROCESS;

#if HD44780_USE_RW == 1
	while (hd44780_is_busy())
		;
#else
	HAL_Delay(1);
#endif
}

static void hd44780_writeData(uint8_t data)
{
	HD44780_SET_DATA;
	hd44780_writeByte(data);
}

static void hd44780_writeCommand(uint8_t command)
{
	HD44780_SET_COMMAND;
	hd44780_writeByte(command);
}

static void delay_us(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&delay_us_timer, 0);
	while (__HAL_TIM_GET_COUNTER(&delay_us_timer) < us)
		;
}

void HD44780_Init(void)
{
#if HD44780_USE_CONTRAST == 1
	HAL_TIM_PWM_Start(&contrast_timer, contrast_timer_channel);
	HD44780_SetContrast(40);
#endif
	HAL_TIM_Base_Start(&delay_us_timer);

	GPIO_InitTypeDef GPIO_command_pins;

	GPIO_command_pins.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_command_pins.Pin = GPIO_NOPULL;
	GPIO_command_pins.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_command_pins.Pin = HD44780_E_PIN;
	HAL_GPIO_Init(HD44780_E_PORT, &GPIO_command_pins);

	GPIO_command_pins.Pin = HD44780_RS_PIN;
	HAL_GPIO_Init(HD44780_RS_PORT, &GPIO_command_pins);

#if HD44780_USE_RW == 1
	GPIO_command_pins.Pin = HD44780_RW_PIN;
	HAL_GPIO_Init(HD44780_RW_PORT, &GPIO_command_pins);
	HAL_GPIO_WritePin(HD44780_RW_PORT, HD44780_RW_PIN, GPIO_PIN_RESET);
#endif

	HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_RESET);
	hd44780_data_out();

	HAL_Delay(100);

	for (uint8_t i = 0; i < 3; i++) {
		HD44780_START_PROCESS;
		hd44780_write4bits(0x03);
		HD44780_STOP_PROCESS;
		HAL_Delay(5);
	}

	HD44780_START_PROCESS;
	hd44780_write4bits(0x02);
	HD44780_STOP_PROCESS;
	delay_us(150);

	hd44780_writeCommand(lcd.config.function);
	hd44780_writeCommand(0x08);
	hd44780_writeCommand(0x01);
	hd44780_writeCommand(lcd.config.entry_mode);
	hd44780_writeCommand(lcd.config.display_control);
	hd44780_writeCommand(HD44780_CLEAR);
	hd44780_writeCommand(HD44780_HOME);
	HD44780_ClearBuffer();

}

void HD44780_ClearBuffer(void)
{
	lcd.data.x = lcd.data.y = 0;

	for (uint8_t y = 0; y < HD44780_HEIGHT; y++)
		for (uint8_t x = 0; x < HD44780_WIDTH; x++)
			lcd.data.screen_buffer[y][x] = ' ';
}

void HD44780_SetCursor(uint8_t x, uint8_t y)
{
	/*
	lcd.data.x = x % HD44780_WIDTH;
	lcd.data.y = y % HD44780_HEIGHT;
	*/

	lcd.data.x = x;
	lcd.data.y = y;
}

void HD44780_WriteChar(char c)
{
	lcd.data.screen_buffer[lcd.data.y][lcd.data.x++] = c;

	lcd.data.x %= HD44780_WIDTH;
	if (lcd.data.x == 0)
		lcd.data.y = (lcd.data.y + 1) % HD44780_HEIGHT;
}

void HD44780_WriteString(char *s)
{
	while (*s)
		HD44780_WriteChar(*s++);
}

void HD44780_Update(void)
{
	static uint8_t last_screen_buffer[HD44780_HEIGHT][HD44780_WIDTH];

	for (uint8_t y = 0; y < HD44780_HEIGHT; y++) {
		for (uint8_t x = 0; x < HD44780_WIDTH; x++) {
			if (last_screen_buffer[y][x] != lcd.data.screen_buffer[y][x]) {
				hd44780_writeCommand(HD44780_SET_DDRAM_ADDRESS + (y * 0x40) + x);
				hd44780_writeData(lcd.data.screen_buffer[y][x]);
				last_screen_buffer[y][x] = lcd.data.screen_buffer[y][x];
			}
		}
	}
}

#if HD44780_USE_CONTRAST == 1
void HD44780_SetContrast(uint8_t percent)
{
	uint16_t pwm_new_value = (percent * TIMER_AUTORELOAD) / 100;

	__HAL_TIM_SET_COMPARE(&contrast_timer, contrast_timer_channel, pwm_new_value);
}
#endif
