#pragma once
#include <stdint.h>

typedef union APA102_Frame {
	struct {
		uint32_t header : 3;
		uint32_t globalBrightness : 5;

		uint32_t blue : 8;
		uint32_t green : 8;
		uint32_t red : 8;
	} fields;
	uint32_t raw;
} APA102_Frame;

typedef struct APA102_Strip {
	APA102_Frame *frames;
	uint32_t ledCount;
} APA102_Strip;


int APA102_init();

APA102_Strip *APA102_alloc(uint32_t ledCount);
void APA102_free(APA102_Strip *strip);

void APA102_setColor(APA102_Strip *strip, uint32_t id,
	uint8_t r, uint8_t g, uint8_t b);
