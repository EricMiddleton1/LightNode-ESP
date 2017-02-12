#include "APA102.h"
#include "drivers/SoftSPI.h"

#define RAW_HEADER_VALUE	0xE0000000

int APA102_init() {
}

APA102_Strip *APA102_alloc(uint32_t ledCount) {
	if(ledCount == 0)
		return NULL;
	
	APA102_Strip *strip = (APA102_Strip*)os_malloc(sizeof(APA102_Strip));
	if(strip == NULL) {
		return NULL;
	}

	strip->frames = (APA102_Frame*)os_malloc(ledCount * sizeof(APA102_Frame));
	if(strip->frames == NULL) {
		free(strip);
		return NULL;
	}

	//Clear first frame
	strip->frames[0].raw = 0;

	unsigned int i;
	for(i = 0; i < ledCount; ++i) {
		strip->frames[i].raw = RAW_HEADER_VALUE;
	}

	strip->ledCount = ledCount;

	return strip;
}

void APA102_free(APA102_Strip *strip) {
	if(strip == NULL)
		return;
	
	free(strip->frames);
	free(strip);
}

void APA102_setColor(APA102_Strip *strip, uint32_t id,
	uint8_t r, uint8_t g, uint8_t b) {

	if( (strip == NULL) || (id >= strip->ledCount) ) {
		return;
	}

	APA102_Frame *frame = strip->frames + id;

	frame->fields.red = r;
	frame->fields.green = g;
	frame->fields.blue = b;
}
