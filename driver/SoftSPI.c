#include "driver/SoftSPI.h"

#include "c_types.h"
#include "eagle_soc.h"
#include "gpio.h"
#include "osapi.h"

//GPIO0 - MOSI
//GPIO2 - SCK

#define PIN_MOSI	(0x00)
#define PIN_SCK		(0x02)

#define HIGH	(0x01)
#define LOW		(0x00)

void SoftSPI_init() {
	gpio_init();

	//Configure pins as GPIO
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	PIN_PULLUP_DIS(PERIPHS_IO_MUX_GPIO2_U);

	//
	GPIO_OUTPUT_SET(GPIO_ID_PIN(PIN_MOSI), LOW);
	GPIO_OUTPUT_SET(GPIO_ID_PIN(PIN_SCK), HIGH);
}

void SoftSPI_send(uint8_t *data, uint32_t len) {
	uint8_t *end  = data + len;

	while(data != end) {
		uint8_t mask;
		for(mask = 0x80; mask > 0; mask >>= 1) {
			//Clock low, set MOSI
			GPIO_OUTPUT_SET(PIN_SCK, LOW);
			GPIO_OUTPUT_SET(PIN_MOSI, (!!(*data & mask)));

			//Clock high
			GPIO_OUTPUT_SET(PIN_SCK, HIGH);
		}
		data++;
	}
}

