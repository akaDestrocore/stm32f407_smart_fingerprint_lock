#include "fingerprint_module.h"

extern R308_t R308;
extern R308_System_Params parameters;

void EnrollNewFingerPrint(void)
{
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(0, 30, "SABLONU SAKLA MAK iCiN BOS  BiR SLOT ARANIYOR", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(500);
	int16_t fid;
	if (GetFreeID(&fid))
		EnrollFinger(fid);
	else{
//		buzzer_alert(errorSound);
		ST7735_FillScreen(ST7735_WHITE);
		ST7735_DrawString(15, 40, "FLASH'DA BOS     SLOT YOK!", Font_11x18, ST7735_RED, ST7735_WHITE);
		delay_ms(500);
//		welcome_message();
		}
	return;
}

void EnrollFinger(int16_t fid)
{
	int16_t state = -1;
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(0, 10, "KAYDETMEK iCiN PARMAK BEKLENiYOR", Font_11x18, ST7735_BLUE, ST7735_WHITE);
//	FP_icon();
	delay_ms(500);
	while (state != R308_OK) {
		state = R308_GetImage(&R308);
		switch (state) {
			case R308_OK:
				break;
			case R308_NOFINGER:
				break;
			default:
//				buzzer_alert(errorSound);
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_Print(0, 50, "HATA KODU: 0x", state, Font_11x18, ST7735_RED, ST7735_BLACK);
				delay_ms(1000);
//				database_operations_menu();
				break;
		}
	}
	// OK success!

	state = R308_Image2Tz(&R308, 1);
	switch (state) {
		case R308_OK:
			ST7735_FillScreen(ST7735_GREEN);
			ST7735_DrawString(0, 50, "GORSEL DONUSTURULDU", Font_11x18, ST7735_WHITE, ST7735_GREEN);
			break;
		default:
//			buzzer_alert(errorSound);
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_Print(0, 50, "HATA KODU: 0x", state, Font_11x18, ST7735_RED, ST7735_BLACK);
			delay_ms(1000);
//			database_operations_menu();
			return;
	}

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 50, "PARMAGINIZI     KALDIRIN", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	state = 0;
	while (state != R308_NOFINGER) {
		state = R308_GetImage(&R308);
		delay_ms(10);
	}

	state = -1;
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(15, 10, "AYNI PARMAGI      TEKRAR       YERLESTiRiN", Font_11x18, ST7735_BLUE, ST7735_WHITE);
//	FP_icon();
	while (state != R308_OK) {
		state = R308_GetImage(&R308);
		switch (state) {
			case R308_OK:
				break;
			case R308_NOFINGER:
				break;
			default:
//				buzzer_alert(errorSound);
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_Print(0, 50, "HATA KODU: 0x", state, Font_11x18, ST7735_RED, ST7735_BLACK);
				delay_ms(500);
//				database_operations_menu();
				break;
		}
	}
		// OK success!

	state = R308_Image2Tz(&R308, 2);
	switch (state) {
		case R308_OK:
			break;
		default:
//			buzzer_alert(errorSound);
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_Print(0, 50, "HATA KODU: 0x", state, Font_11x18, ST7735_RED, ST7735_BLACK);
			delay_ms(500);
//			database_operations_menu();
			return;
	}


	// OK converted!
	state = R308_CreateModel(&R308);
	switch (state) {
		case R308_OK:
			ST7735_FillScreen(ST7735_GREEN);
			ST7735_DrawString(10, 45, "PARMAK iZLERi    ESLESTi!", Font_11x18, ST7735_WHITE, ST7735_GREEN);
			delay_ms(500);
			break;
		default:
//			buzzer_alert(errorSound);
			ST7735_InvertColors(false);
			ST7735_FillScreen(ST7735_WHITE);
			ST7735_DrawString(0, 45, "ESLESME HATASI TEKRAR DENEYiNiZ!", Font_11x18, ST7735_RED, ST7735_WHITE);
			delay_ms(100);
			ST7735_FillScreen(ST7735_RED);
			ST7735_DrawString(0, 45, "ESLESME HATASI TEKRAR DENEYiNiZ!", Font_11x18, ST7735_WHITE, ST7735_RED);
			delay_ms(100);
			ST7735_FillScreen(ST7735_WHITE);
			ST7735_DrawString(0, 45, "ESLESME HATASI TEKRAR DENEYiNiZ!", Font_11x18, ST7735_RED, ST7735_WHITE);
			delay_ms(100);
//			database_operations_menu();
			return;
	}

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_Print(50, 50, "ID", fid, Font_16x26, ST7735_BLUE, ST7735_WHITE);
	delay_ms(500);
	state = R308_StoreModel(&R308, fid, 1);
	switch (state) {
		case R308_OK:
			ST7735_FillScreen(ST7735_GREEN);
			ST7735_DrawString(20, 50, "KAYDEDiLDi!", Font_11x18, ST7735_WHITE, ST7735_GREEN);
			delay_ms(500);
//			database_operations_menu();
			break;
		default:
//			buzzer_alert(errorSound);
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_Print(0, 50, "HATA KODU: 0x", state, Font_11x18, ST7735_RED, ST7735_BLACK);
			delay_ms(500);
//			database_operations_menu();
			return;
	}
}

void SearchDatabase(void)
{
	int16_t state = -1;

	/* first get the finger image */
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(30, 10, "PARMAGIN       BASMASI       BEKLENiYOR.", Font_11x18, ST7735_BLUE, ST7735_WHITE);
//	FP_icon();         //just some finger print picture to make it look prettier
	delay_ms(500);
	while (state != R308_OK) {
		state = R308_GetImage(&R308);
		switch (state) {
			case R308_OK:
				break;
			case R308_NOFINGER:
				break;
			default:
//				buzzer_alert(errorSound);
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_Print(15, 50, "HATA KODU:", state, Font_11x18, ST7735_RED, ST7735_BLACK);
				delay_ms(500);
//				welcome_message();
				return;
		}
	}
	/* convert it and store in buffer/slot 1 */
	state = R308_Image2Tz(&R308, 1);
	switch (state) {
		case R308_OK:
			break;
		default:
//			buzzer_alert(errorSound);
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_Print(25, 50, "HATA:", state, Font_11x18, ST7735_RED, ST7735_BLACK);
			delay_ms(500);
//			welcome_message();
			return;
	}
	/* search the database for the converted print (now in slot 1) */
	uint16_t fid, score;
	state = R308_SearchDatabase(&R308, &fid, &score, 1);

	if (state == R308_OK) {
		ST7735_FillScreen(ST7735_GREEN);
		ST7735_DrawString(25, 35, "PARMAK iZi      ESLESMESi      BULUNDU!", Font_11x18, ST7735_WHITE, ST7735_GREEN);
		delay_ms(400);
	}
	else if (state == R308_NOTFOUND) {
//		buzzer_alert(alarm);
		return;
	}
	else {
//		buzzer_alert(errorSound);
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_Print(25, 50, "HATA:", state, Font_11x18, ST7735_RED, ST7735_BLACK);
		delay_ms(1000);
//		welcome_message();
		return;
	}

	// found a match!
//	if(fid == adminID){ AccessStep = 3;}
//	else{first_step_confirmed(fid,score);}
}

uint16_t GetFreeID(int16_t * fid)
{
	int16_t state = -1;
	for (int page = 0; page < (parameters.capacity / R308_TEMPLATES_PER_PAGE) + 1; page++) {
		state = R308_GetFreeIndex(&R308, page, fid);
		switch (state) {
			case R308_OK:
				if (*fid != R308_NOFREEINDEX)
				{
					ST7735_FillScreen(ST7735_WHITE);
					ST7735_Print(30, 40, "BOS SLOT       BULUNDU        ID =", *fid, Font_11x18, ST7735_BLUE, ST7735_WHITE);
					delay_ms(500);
					return 1;
				}
				break;
			default:
//				buzzer_alert(errorSound);
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_Print(15, 50, "HATA KODU:", state, Font_11x18, ST7735_RED, ST7735_BLACK);
				delay_ms(500);
//				welcome_message();
				return 0;
		}
	}

	return 0;
}

void EmptyDB(void)
{
	uint8_t empty_status = R308_EmptyDatabase(&R308);
	if(empty_status == 0)
	{
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_DrawString(25, 40, "VERiTABANI      SiLiNDi", Font_11x18, ST7735_GREEN, ST7735_BLACK);
		delay_ms(1000);
		EnrollNewFingerPrint();
//		database_operations_menu();
	}
//	else { ST7735_FillScreen(ST7735_BLACK); ST7735_DrawString(25, 50, "HATA OLUSTU", Font_11x18, ST7735_RED, ST7735_BLACK);
//	delay_ms(500);admin_profile();}
}
