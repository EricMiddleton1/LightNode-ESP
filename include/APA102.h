#pragma once
#include <stdint.h>

#define APA102_ERR_OK		(0)
#define APA102_ERR_NULL	(-1)
#define APA102_ERR_ID		(-2)


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
int APA102_free(APA102_Strip *strip);

int APA102_clear(APA102_Strip *strip);

int APA102_setAll(APA102_Strip *strip, uint8_t r, uint8_t g, uint8_t b);
int APA102_setColor(APA102_Strip *strip, uint32_t id, uint8_t r, uint8_t g, uint8_t b);

int APA102_getColor(const APA102_Strip *strip, uint32_t id, uint8_t *r, uint8_t *g, uint8_t *b);

int APA102_display(const APA102_Strip *strip);
