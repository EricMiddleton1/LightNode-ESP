#include "SoftSPI.h"

#include "gpio.h"

//GPIO0 - MOSI
//GPIO2 - SCK

#define PIN_MOSI	(0x00)
#define PIN_SCK		(0x02)

#define HIGH	(0x01)
#define LOW		(0x00)

void SoftSPI_start() {
	gpio_init();

	//Configure pins as GPIO
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);

	//
	GPIO_OUTPUT_SET(PIN_MOSI, LOW);
	GPIO_OUTPUT_SET(PIN_SCK, HIGH);
}

void SoftSPI_send(uint8_t *data, uint32_t len) {

