#ifndef __ST7735_FONTS_H_
#define __ST7735_FONTS_H_

#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
}FontDef_t;

extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;
extern FontDef_t Font_16x26;

#endif /* __ST7735_FONTS_H_ */
