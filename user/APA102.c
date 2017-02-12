#include "APA102.h"
#include "driver/SoftSPI.h"

#include <mem.h>

#define NULL	(0)

//NOTE: xtensa architecture is big endian!
#define RAW_HEADER_VALUE	0x000000FF

static uint32_t __ledToFrameCount(uint32_t ledCount);

int APA102_init() {
	SoftSPI_init();

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
		APA102_Frame *frame = strip->frames + i;
		frame->fields.red = r;
		frame->fields.green = g;
		frame->fields.blue = b;
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

	APA102_Frame *frame = strip->frames + id + 1;

	frame->fields.red = r;
	frame->fields.green = g;
	frame->fields.blue = b;

	return APA102_ERR_OK;
}

int APA102_getColor(const APA102_Strip *strip, uint32_t id, uint8_t *r, uint8_t *g, uint8_t *b) {
	if(strip == NULL) {
		return APA102_ERR_NULL;
	}
	else if(id >= strip->ledCount) {
		return APA102_ERR_ID;
	}

	APA102_Frame *frame = strip->frames + id + 1;

	*r = frame->fields.red;
	*g = frame->fields.green;
	*b = frame->fields.blue;

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
	//Total frame count is led count, plus 1 frame header, and 1 + (ledCount/64) footer frames
	return ledCount + 2 + ledCount/64;
}
