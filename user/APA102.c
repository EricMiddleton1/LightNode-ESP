#include "c_types.h"
#include "APA102.h"
#include "driver/SoftSPI.h"

#include "driver/uart.h" //For debugging

#include <mem.h>
#include <math.h>

#define GAMMA		(2.3f)
#define GAMMA_MAX	(8192.f)

//NOTE: xtensa architecture is big endian!
#define RAW_HEADER_VALUE	0x000000FF

static uint32_t __ledToFrameCount(uint32_t ledCount);
static void __setFrameColor(APA102_Frame *frame,
	uint8_t r, uint8_t g, uint8_t b);
static uint16_t *__gammaTable = NULL;

inline int max(int a, int b) {
	return (a > b) ? a : b;
}

int APA102_init() {
	SoftSPI_init();

	if(__gammaTable == NULL) {
		__gammaTable = (uint16_t*)os_malloc(256 * 2);
		if(__gammaTable == NULL) {
			return APA102_ERR_MEM;
		}

		unsigned int i;
		for(i = 0; i < 256; ++i) {
			__gammaTable[i] = pow((float)i / 255.f, GAMMA) * (GAMMA_MAX) + 0.5;
		}
	}

	return APA102_ERR_OK;
}

APA102_Strip *APA102_alloc(uint32_t ledCount) {
	if(ledCount == 0) {
		return NULL;
	}

	uint32_t frameCount = __ledToFrameCount(ledCount);
	
	APA102_Strip *strip = (APA102_Strip*)os_malloc(sizeof(APA102_Strip));
	if(strip == NULL) {
		return NULL;
	}

	strip->frames = (APA102_Frame*)os_malloc(frameCount * sizeof(APA102_Frame));
	if(strip->frames == NULL) {
		os_free(strip);
		return NULL;
	}

	//Clear first frame
	strip->frames[0].raw = 0;

	unsigned int i;
	for(i = 1; i < frameCount; ++i) {
		strip->frames[i].raw = RAW_HEADER_VALUE;
	}

	strip->ledCount = ledCount;

	return strip;
}

int APA102_free(APA102_Strip *strip) {
	if(strip == NULL)
		return APA102_ERR_NULL;
	
	os_free(strip->frames);
	os_free(strip);

	return APA102_ERR_OK;
}

int APA102_clear(APA102_Strip *strip) {
	if(strip == NULL) {
		return APA102_ERR_NULL;
	}

	uint32_t frameCount = __ledToFrameCount(strip->ledCount);

	while((--frameCount) > 0) {
		strip->frames[frameCount].raw = RAW_HEADER_VALUE;
	}

	return APA102_ERR_OK;
}

int APA102_setAll(APA102_Strip *strip, uint8_t r, uint8_t g, uint8_t b) {
	if(strip == NULL) {
		return APA102_ERR_NULL;
	}

	uint32_t i;
	for(i = 1; i <= strip->ledCount; ++i) {
		/*
		APA102_Frame *frame = strip->frames + i;
		frame->fields.red = __gammaTable[r];
		frame->fields.green = __gammaTable[g];
		frame->fields.blue = __gammaTable[b];
		*/
		__setFrameColor(strip->frames+i, r, g, b);
	}

	return APA102_ERR_OK;
}

int APA102_setColor(APA102_Strip *strip, uint32_t id, uint8_t r, uint8_t g, uint8_t b) {

	if(strip == NULL) {
		return APA102_ERR_NULL;
	}
	else if(id >= strip->ledCount) {
		return APA102_ERR_ID;
	}

/*
	APA102_Frame *frame = strip->frames + id + 1;

	frame->fields.red = __gammaTable[r];
	frame->fields.green = __gammaTable[g];
	frame->fields.blue = __gammaTable[b];
*/
	__setFrameColor(strip->frames+id+1, r, g, b);

	return APA102_ERR_OK;
}

int APA102_display(const APA102_Strip *strip) {
	if(strip == NULL) {
		return APA102_ERR_NULL;
	}

	uint32_t frameCount = __ledToFrameCount(strip->ledCount);

	SoftSPI_send((uint8_t*)(strip->frames), sizeof(APA102_Frame) * frameCount);

	return APA102_ERR_OK;
}

uint32_t __ledToFrameCount(uint32_t ledCount) {
	//Total frame count is led count, plus 1 frame header,
	//and 1 + (ledCount/64) footer frames
	return ledCount + 2 + ledCount/64;
}

static void __setFrameColor(APA102_Frame *frame,
	uint8_t r, uint8_t g, uint8_t b) {

	uint16_t gr, gg, gb;

	gr = __gammaTable[r];
	gg = __gammaTable[g];
	gb = __gammaTable[b];

	uint16_t largest = max(gr, max(gg, gb));

	if(largest == 0) {
		//Just set color to 0
		frame->fields.red = 0;
		frame->fields.green = 0;
		frame->fields.blue = 0;

		return;
	}

	uint8_t brightness = 30*255/largest + 1;

	uint16_t c;

	c = (31 * gr / brightness) >> 5;
	if(c > 255) {
		uart_debugSend("Red > 255\r\n");

		c = 255;
	}
	frame->fields.red = c;

	c = (31 * gg / brightness) >> 5;
	if(c > 255) {
		uart_debugSend("Green > 255\r\n");

		c = 255;
	}
	frame->fields.green = c;

	c = (31 * gb / brightness) >> 5;
	if(c > 255) {
		uart_debugSend("Blue > 255\r\n");

		c = 255;
	}
	frame->fields.blue = c;

	frame->fields.globalBrightness = brightness;
}
