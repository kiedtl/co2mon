#include <u8g2.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "wrapper.h"
#include "scenery.h"
#include "font.h"

#define RF_NODE_ID 1

#define FONT_WIDTH  5
#define FONT_HEIGHT 8

#define RF_CE_PIN 17 // nRF24L01+ CE pin
#define RF_CS_PIN 20 // nRF24L01+ CSn pin
#define UIROT_PIN 13 // Button to rotate UI interface
#define INDIC_PIN 14 // Indicator LED (blue)
#define BLINK_PIN 15 // Blinker LED (yellow)

#define HIST_LEN (128 / 2)

enum UiMode {
	UM_Co2,
	UM_Tmp,
	UM_Hum,
};

typedef struct dataset {
	uint16_t co2;
	float tmp, hum;
} dataset_t;

#ifdef PICO_W
#define LED_GPIO_INIT() if (cyw43_arch_init()) { ERROR("Cyw43 init failed."); }
#define LED_GPIO_PUT(n) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, n)
#else
#define LED_GPIO_INIT(n) (void)(0)
#define LED_GPIO_PUT(n) gpio_put(25, n) // Aliexpress pin number <shrug>
#endif

#define HIST_ITER(stuff) \
	for (size_t i = 0, ind = samples_ind - 1; i < HIST_LEN; ++i) { \
		{ stuff; }; \
		if (ind == 0) ind = HIST_LEN; \
		ind -= 1; \
	}

#define INFO(estr) do { \
	u8g2_ClearBuffer(&u8g2); \
	u8g2_DrawXBM(&u8g2, 0, 0, SCENE_W, SCENE_H, scenery_bits); \
	u8g2_DrawStr(&u8g2, 2, 8, estr); \
	u8g2_SendBuffer(&u8g2); \
} while (false);

#define WARN(estr, ...) do { \
	u8g2_ClearBuffer(&u8g2); \
	u8g2_DrawXBM(&u8g2, 0, 0, SCENE_W, SCENE_H, scenery_bits); \
	u8g2_DrawStr(&u8g2, 2, 8, estr); \
	u8g2_SendBuffer(&u8g2); \
	fprintf(stderr, __VA_ARGS__); \
} while (false);

#define ERROR2(estr, ...) do { \
	u8g2_ClearBuffer(&u8g2); \
	u8g2_DrawXBM(&u8g2, 0, 0, SCENE_W, SCENE_H, scenery_bits); \
	u8g2_DrawStr(&u8g2, 2, 8, estr); \
	u8g2_SendBuffer(&u8g2); \
	ERROR(__VA_ARGS__); \
} while (false);

#define ERROR(...) do { \
	fprintf(stderr, __VA_ARGS__); \
	uhoh(); \
} while (false);

u8g2_t u8g2;
enum UiMode uimode;
RF24MeshWrapper *mesh;
RF24 *radio;

double co2_samples[HIST_LEN] = {0}, tmp_samples[HIST_LEN] = {0}, hum_samples[HIST_LEN] = {0};
uint8_t co2_samples_ind = 0, tmp_samples_ind = 0, hum_samples_ind = 0;

void draw(uint16_t co2, float temperature, float humidity);

_Noreturn void
uhoh() 
{
	while (true) {
		LED_GPIO_PUT(1);
		sleep_ms(128);
		LED_GPIO_PUT(0);
		sleep_ms(128);
	}
}

void
callback_button_uirot(size_t gpio, uint32_t event_mask)
{
	switch (uimode) {
	break; case UM_Co2: uimode = UM_Tmp;
	break; case UM_Tmp: uimode = UM_Hum;
	break; case UM_Hum: uimode = UM_Co2;
	break; default:
		ERROR2("E_UNREACHABLE", "Unknown UI mode.");
	break;
	}

	// Just for an immediate response.
	draw(
		(uint16_t)co2_samples[co2_samples_ind - 1],
		(float)tmp_samples[tmp_samples_ind - 1],
		(float)hum_samples[hum_samples_ind - 1]
	);
}

size_t
calc_graph_y(size_t sample, size_t min, size_t max)
{
	double s = (double)sample;
	s = s > min ? s - min : 0;
	s = s > max ? max : s;
	s = s / (max - min);
	s = 1 - s;
	size_t y = (size_t)(s * (64 - 16));
	y += 16;
	return y;
}

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

uint8_t
u8g2_byte_rpi_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	static uint8_t send_buf[128];
	static size_t send_buf_len = 0;

	switch (msg) {
	break; case U8X8_MSG_BYTE_INIT:
		/* Already init'd */

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
record(double *samples, uint8_t samples_ind, uint16_t sample)
{
	samples[samples_ind++] = (double)sample;
	if (samples_ind == HIST_LEN)
		samples_ind = 0;
	return samples_ind;
}

uint8_t
recordf(double *samples, uint8_t samples_ind, float sample)
{
	samples[samples_ind++] = (double)sample;
	if (samples_ind == HIST_LEN)
		samples_ind = 0;
	return samples_ind;
}

void
draw(uint16_t co2, float temperature, float humidity)
{
	u8g2_ClearBuffer(&u8g2);

	// ----- Status bar (top) -----

	char buf[32];
	size_t x = 2;
	size_t y = FONT_HEIGHT + 1;

	if (uimode == UM_Co2) {
		u8g2_DrawBox(&u8g2, x, 0, 8 * FONT_WIDTH, 10);
		u8g2_SetDrawColor(&u8g2, 0);
	}
	snprintf((char *)&buf, 32, " %4d C ", co2);
	x += u8g2_DrawStr(&u8g2, x, y, (char *)&buf);
	u8g2_SetDrawColor(&u8g2, 1);

	if (uimode == UM_Tmp) {
		u8g2_DrawBox(&u8g2, x, 0, 8 * FONT_WIDTH, 10);
		u8g2_SetDrawColor(&u8g2, 0);
	}
	snprintf((char *)&buf, 32, " %.01f\xb0""F ", temperature);
	x += u8g2_DrawStr(&u8g2, x, y, (char *)&buf);
	u8g2_SetDrawColor(&u8g2, 1);

	if (uimode == UM_Hum) {
		u8g2_DrawBox(&u8g2, x, 0, 9 * FONT_WIDTH, 10);
		u8g2_SetDrawColor(&u8g2, 0);
	}
	snprintf((char *)&buf, 32, " %.01f RH ", humidity);
	x += u8g2_DrawStr(&u8g2, x, y, (char *)&buf);
	u8g2_SetDrawColor(&u8g2, 1);

	// ----- Graphs (bottom) -----

	double max_max, min_min, max_min, max_min_diff;
	uint8_t samples_ind;
	double *samples;

	switch (uimode) {
	break; case UM_Co2:
		samples_ind = co2_samples_ind;
		samples = (double *)&co2_samples;
		max_max = 2200, min_min = 200, max_min = 800;
		max_min_diff = 80;
		// Eddy current sensor experiment
		max_max = 2200, min_min = 0, max_min = 15;
		max_min_diff = 20;
	break; case UM_Tmp:
		samples_ind = tmp_samples_ind;
		samples = (double *)&tmp_samples;
		max_max = 100, min_min = 0, max_min = 58;
		max_min_diff = 4;
	break; case UM_Hum:
		samples_ind = hum_samples_ind;
		samples = (double *)&hum_samples;
		max_max = 100, min_min = 0, max_min = 30; // RH is a percent, so 100 is max anyways
		max_min_diff = 6;
	break; default:
		ERROR2("E_UNREACHABLE", "Unknown UI mode.");
	break;
	}

	size_t min = 999999;
	size_t max = 0;

	HIST_ITER(
		if (samples[ind] == 0) continue;
		if (samples[ind] > max) max = samples[ind];
		if (samples[ind] < min) min = samples[ind];
	);

	if (max - min < max_min_diff) {
		min -= max_min_diff / 2;
		max += max_min_diff / 2;
	}
	max = max > max_max ? max_max : max;
	min = min < min_min ? min_min : min;
	min = min > max_min ? max_min : min;

	size_t prev_ind = co2_samples_ind - 2;
	HIST_ITER({
		size_t x = 127 - (i * 2);
		size_t y = calc_graph_y(samples[ind], min, max);
		u8g2_DrawPixel(&u8g2, x, y);

		if (x > 0) {
			size_t prevy = calc_graph_y(samples[prev_ind], min, max);
			ssize_t hdiff = (((ssize_t)prevy) - ((ssize_t)y)) / 2;
			u8g2_DrawPixel(&u8g2, x - 1, (size_t)(((ssize_t)y) - hdiff));
		}

		prev_ind = ind;
	});

	memset(buf, 0, 32);
	snprintf((char *)&buf, 32, "%4d ", max);
	u8g2_DrawStr(&u8g2, 0, 16 + 8, (char *)&buf);

	memset(buf, 0, 32);
	snprintf((char *)&buf, 32, "%4d ", min);
	u8g2_DrawStr(&u8g2, 0, 64, (char *)&buf);

	u8g2_SendBuffer(&u8g2);
}

/* Assumes that radio is enabled. */
void
try_renew_addr(size_t tries)
{
	for (size_t i = 0; i < tries && RF24Mesh_renewAddress(mesh) == MESH_DEFAULT_ADDRESS; ++i)
		delayed_blink(2, 128);
}

int
main()
{
	i2c_init(i2c0, 45 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	gpio_init(BLINK_PIN);
	gpio_init(INDIC_PIN);
	gpio_init(UIROT_PIN);
	gpio_set_dir(BLINK_PIN, GPIO_OUT);
	gpio_set_dir(INDIC_PIN, GPIO_OUT);
	gpio_set_dir(UIROT_PIN, GPIO_IN);
	gpio_set_irq_enabled_with_callback(UIROT_PIN, GPIO_IRQ_EDGE_RISE, true, &callback_button_uirot);

	gpio_put(INDIC_PIN, 1);

	uimode = UM_Co2;

	stdio_init_all();
	LED_GPIO_INIT();

	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2, U8G2_R0, u8g2_byte_rpi_hw_i2c, u8g2_gpio_and_delay_rpi);

	u8g2_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_ClearDisplay(&u8g2);
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, font_5x8_tf);

	//sensirion_i2c_hal_init(); // I2C is already init'ed, commented out

	int16_t scd_error = 0;

	uint16_t co2;
	float temperature, humidity;

	INFO("Booting radio");
	mesh = RF24Mesh_create(RF_CE_PIN, RF_CS_PIN);
	RF24Mesh_setNodeID(mesh, RF_NODE_ID);
	radio = RF24Mesh_getRadio(mesh);
	_Bool radio_enabled = true;

	if (!RF24_begin(radio)) {
		WARN("E_NORADIO", "Hardware issue with nRF24L01+ module.\n");
		sleep_ms(1048);
		radio_enabled = false;
	} else {
		RF24_setChannel(radio, 46);
		RF24_setPALevel(radio, 0); // RF24_PA_LOW
		RF24_setPayloadSize(radio, 20);
		RF24_setCRCLength(radio, RF24_CRC_16);
		RF24_setDataRate(radio, RF24_250KBPS);
	}

	INFO("Connecting");
	if (!RF24Mesh_begin(mesh, 46, RF24_250KBPS, 1000)) {
		WARN("E_NOCONNECT", "Could not connect to mesh. Possibly hardware issue.");
		sleep_ms(1048);
		radio_enabled = false;
	} else {
		try_renew_addr(7);
	}

	INFO("Booting sensor");
	scd4x_wake_up();
	scd4x_stop_periodic_measurement();
	scd4x_set_automatic_self_calibration(false);
	scd4x_reinit();

	INFO("Testing sensor");
	uint16_t scd_status;
	scd_error = scd4x_perform_self_test(&scd_status);
	if (scd_error) {
		ERROR2("Test failed", "Couldn't initiate self-test: %d\n", scd_error);
	}
	if (scd_status != 0) {
		ERROR2("Sensor failed", "Self-test failed.\n");
	}

	INFO("Starting sensor");
	scd_error = scd4x_start_periodic_measurement();
	if (scd_error) {
		ERROR2("Measure failed", "Couldn't start measurement: %d\n", scd_error);
	}

	while ("static types are the best") {
		delayed_blink(2, 1150);

		if (radio_enabled) RF24Mesh_update(mesh);

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
			ERROR2("Measure failed", "Error measuring: %i\n", scd_error);
		} else if (co2 == 0) {
			WARN("Invalid sample", "Invalid sample detected, skipping.\n");
			gpio_put(INDIC_PIN, 1);
			continue;
		}

		// Convert to Fahrenheit
		temperature *= 9;
		temperature /= 5;
		temperature += 32;

		co2_samples_ind = record((double *)&co2_samples, co2_samples_ind, co2);
		tmp_samples_ind = recordf((double *)&tmp_samples, tmp_samples_ind, temperature);
		hum_samples_ind = recordf((double *)&hum_samples, hum_samples_ind, humidity);

		dataset_t d;
		d.co2 = co2, d.tmp = temperature, d.hum = humidity;
		if (radio_enabled && !RF24Mesh_write(mesh, 0, &d, sizeof(d), 'D')) {
			if (!RF24Mesh_checkConnection(mesh)) {
				if (!RF24_isChipConnected(radio)) {
					WARN("Radio disabled", "Hardware issue with nRF24L01+ module.\n");
					radio_enabled = false;
					continue;
				}

				INFO("Connecting...");
				try_renew_addr(2);
			}
		}

		draw(co2, temperature, humidity);
	}

	return 0;
}
