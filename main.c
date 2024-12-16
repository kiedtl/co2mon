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

#define INDIC_PIN 14
#define BLINK_PIN 15

#define HIST_LEN 127
#define HIST_ITER(stuff) \
	for (size_t i = 0, ind = co2_samples_ind - 1; i < HIST_LEN; ++i) { \
		{ stuff; }; \
		if (ind == 0) ind = HIST_LEN; \
		ind -= 1; \
	}

#define WARN2(estr, ...) do { \
	u8g2_DrawStr(&u8g2, 2, 8, estr); \
	fprintf(stderr, __VA_ARGS__); \
} while (false);

#define ERROR2(estr, ...) do { \
	u8g2_DrawStr(&u8g2, 2, 8, estr); \
	u8g2_SendBuffer(&u8g2); \
	ERROR(__VA_ARGS__); \
} while (false);

#define ERROR(...) do { \
	fprintf(stderr, __VA_ARGS__); \
	uhoh(); \
} while (false);

void
delayed_blink(size_t times, size_t delay)
{
	for (size_t i = 0; i < times; ++i) {
		gpio_put(BLINK_PIN, 0);
		sleep_ms(delay);
		gpio_put(BLINK_PIN, 1);
		sleep_ms(delay);
	}
}

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
	static uint8_t send_buf[128];
	static size_t send_buf_len = 0;

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

uint8_t
record(uint16_t *samples, uint8_t samples_ind, uint16_t sample)
{
	samples[samples_ind++] = sample;
	if (samples_ind == HIST_LEN)
		samples_ind = 0;
	return samples_ind;
}

int
main()
{
	gpio_init(BLINK_PIN);
	gpio_init(INDIC_PIN);
	gpio_set_dir(BLINK_PIN, GPIO_OUT);
	gpio_set_dir(INDIC_PIN, GPIO_OUT);

	gpio_put(INDIC_PIN, 1);

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
	uint16_t co2_samples[HIST_LEN] = {0};
	uint8_t co2_samples_ind = 0;
	float temperature, humidity;

	scd4x_wake_up();
	scd4x_stop_periodic_measurement();
	scd4x_set_automatic_self_calibration(false);
	scd4x_reinit();

	uint16_t scd_status;
	scd_error = scd4x_perform_self_test(&scd_status);
	if (scd_error) {
		ERROR2("E_NOTEST", "Couldn't initiate self-test: %d\n", scd_error);
	}
	if (scd_status != 0) {
		ERROR2("E_FAILTEST", "Self-test failed.\n");
	}

	scd_error = scd4x_start_periodic_measurement();
	if (scd_error) {
		ERROR2("E_NOSTART", "Couldn't start measurement: %d\n", scd_error);
	}

	while ("static types are the best") {
		delayed_blink(2, 1200);

		bool is_data_ready = false;
		scd_error = scd4x_get_data_ready_flag(&is_data_ready);
		if (scd_error) {
			ERROR("E_NOREADY", "Error when getting data ready: %i\n ", scd_error);
			continue;
		}

		if (!is_data_ready) {
			gpio_put(INDIC_PIN, 1);
			continue;
		}
		gpio_put(INDIC_PIN, 0);

		scd_error = scd4x_read_measurement(&co2, &temperature, &humidity);
		if (scd_error) {
			ERROR2("E_NOMEASURE", "Error measuring: %i\n", scd_error);
		} else if (co2 == 0) {
			WARN2("E_INVMEASURE", "Invalid sample detected, skipping.\n");
			gpio_put(INDIC_PIN, 1);
		} else {
			co2_samples_ind = record((uint16_t *)&co2_samples, co2_samples_ind, co2);

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
			u8g2_DrawStr(&u8g2, 2, 8, (char *)&buf);

			size_t min = 999999;
			size_t max = 300;

			HIST_ITER(
				if (co2_samples[ind] == 0) continue;
				if (co2_samples[ind] > max) max = co2_samples[ind];
				if (co2_samples[ind] < min) min = co2_samples[ind];
			);
			max = max > 4000 ? 4000 : max;
			min = min <  200 ?  200 : min;
			min = min >  900 ?  900 : min;
			max = max - min < 120 ? min + 120 : max;

			HIST_ITER({
				size_t x = 127 - i;
				double s = co2_samples[ind];
				s = s > min ? s - min : 0;
				s = s > max ? max : s;
				s = s / (max - min);
				s = 1 - s;
				size_t y = (size_t)(s * (64 - 16));
				y += 16;
				u8g2_DrawPixel(&u8g2, x, y);
			});

			memset(buf, 0, 32);
			snprintf((char *)&buf, 32, "%4d ", max);
			u8g2_DrawStr(&u8g2, 0, 16 + 8, (char *)&buf);

			memset(buf, 0, 32);
			snprintf((char *)&buf, 32, "%4d ", min);
			u8g2_DrawStr(&u8g2, 0, 64, (char *)&buf);

			u8g2_SendBuffer(&u8g2);
		}
	}

	return 0;
}
