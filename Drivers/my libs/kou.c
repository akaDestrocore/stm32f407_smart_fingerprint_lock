#include "kou.h"
#include "kou_logo.h"
#include "st7735.h"
void KOU_Logo(void)
{
	ST7735_FillScreen(ST7735_DARK_GREEN);
	ST7735_FillCircle(78,44,37,ST7735_BLACK);
	ST7735_FillCircle(78,44,35,ST7735_WHITE);
	ST7735_FillCircle(78,44,31,ST7735_WHITE);
	ST7735_DrawImage(53, 20, 50 , 50, (const uint16_t*) koulogo);
	ST7735_DrawCircle(78,44,33,ST7735_GREEN);
	ST7735_DrawCircle(78,44,32,ST7735_GREEN);
	ST7735_DrawString(0,84,"    KOCAELi     UNiVERSiTESi  ",Font_11x18,ST7735_WHITE,ST7735_DARK_GREEN);
}

void FP_icon(void)
{
	ST7735_DrawImage(53, 70, 40 , 40, (const uint16_t*) fingerprintIMG);
}

void alert_icon()
{
	ST7735_FillTriangle(55, 40, 75, 5, 95, 40, ST7735_YELLOW);
	ST7735_DrawString(70,18,"!", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
	ST7735_DrawTriangle(55, 40, 75, 5, 95, 40, ST7735_BLACK);
	ST7735_DrawTriangle(56, 39, 75, 6, 94, 39, ST7735_BLACK);
}
