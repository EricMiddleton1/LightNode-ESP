#pragma once

#include <stdint.h>

void SoftSPI_init();

void SoftSPI_send(uint8_t *data, uint32_t len)
