/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <u8g2.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

uint8_t send_buf[128];
size_t send_buf_len = 0;

#define ERROR(...) do { \
	fprintf(stderr, __VA_ARGS__); \
	uhoh(); \
} while (false);

_Noreturn void
uhoh() 
{
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
		sleep_ms(128);
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
		sleep_ms(128);
	}
}

uint8_t
u8g2_byte_rpi_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch (msg) {
	break; case U8X8_MSG_BYTE_INIT:
		i2c_init(i2c0, 100 * 1000);
		gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
		gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
		gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
		gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	break; case U8X8_MSG_BYTE_START_TRANSFER:
		memset(send_buf, 0, 128);
		send_buf_len = 0;

	break; case U8X8_MSG_BYTE_SEND: {
		uint8_t *data = (uint8_t *)arg_ptr;
		while (arg_int > 0) {
			send_buf[send_buf_len++] = *data;
			data++;
			arg_int--;
		}
		break;
	}

	break; case U8X8_MSG_BYTE_END_TRANSFER: {
		ssize_t ret = i2c_write_blocking(
			i2c0, 0x3C,
			(uint8_t *)&send_buf, send_buf_len,
			false
		);
		if (ret == -1) {
			ERROR("Write failed (slave ignored data).");
		}
	}

	break; default:
		ERROR("unknown msg type %d\n", msg);
		return 0;
	break;
	}
	return 0;
}

uint8_t
u8g2_gpio_and_delay_rpi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch(msg) {
	break; case U8X8_MSG_GPIO_AND_DELAY_INIT:
		gpio_init(arg_int);
		gpio_set_dir(arg_int, GPIO_OUT);
	break; case U8X8_MSG_DELAY_MILLI:
		sleep_ms(arg_int);
	break; case U8X8_MSG_DELAY_10MICRO:
		sleep_us(arg_int * 10);
	break; case U8X8_MSG_GPIO_RESET:
		gpio_put(arg_int, 0);
		sleep_ms(10);
		gpio_put(arg_int, 1);
	break; default:
		ERROR("Unknown delay message %d\n", msg);
	}

	return 1;
}

int
main()
{
	stdio_init_all();
	if (cyw43_arch_init()) {
		ERROR("Cyw43 init failed.");
	}

	u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2, U8G2_R0, u8g2_byte_rpi_hw_i2c, u8g2_gpio_and_delay_rpi);

	u8g2_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);

	u8g2_DrawBox(&u8g2, 5, 5, 5, 5);
	u8g2_SendBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);

	//sensirion_i2c_hal_init(); // I2C is already init'ed, commented out

	int16_t scd_error = 0;
	uint16_t co2;
	float temperature, humidity;

	scd4x_wake_up();
	scd4x_stop_periodic_measurement();
	scd4x_reinit();

	scd_error = scd4x_start_periodic_measurement();
	if (scd_error) {
		ERROR("Couldn't start measurement: %d\n", scd_error);
	}

	while ("I hate this") {
		bool data_ready_flag = false;
		sensirion_i2c_hal_sleep_usec(50000);
		scd_error = scd4x_get_data_ready_flag(&data_ready_flag);
		if (scd_error) {
			//fprintf(stderr, "Error executing scd4x_get_data_ready_flag(): %i\n ", scd_error);
			continue;
		}

		if (!data_ready_flag) {
			continue;
		}

		scd_error = scd4x_read_measurement(&co2, &temperature, &humidity);
		if (scd_error) {
			ERROR("Error executing scd4x_read_measurement(): %i\n", scd_error);
		} else if (co2 == 0) {
			ERROR("Invalid sample detected, skipping.\n");
		} else {
			temperature *= 9;
			temperature /= 5;
			temperature += 32;

			u8g2_ClearBuffer(&u8g2);
			char buf[32];

			snprintf(
				(char *)&buf, 32,
				"%4d C | %.01f\xb0""F | %.01f RH",
				co2, temperature, humidity
			);
			u8g2_DrawStr(&u8g2, 1, 8, (char *)&buf);
			u8g2_SendBuffer(&u8g2);
		}
	}

	return 0;
}
