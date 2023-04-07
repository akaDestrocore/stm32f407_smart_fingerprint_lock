#include "screen_menus.h"

extern TIM_RegDeg_t BUZZER;
extern TIM_RegDeg_t TIMER;
extern R308_t R308;

extern uint8_t Key;
extern uint8_t SystemState;
extern uint8_t AdminState;
extern int counter_1s;
extern int	digit_number;
extern int	alarm_time;

extern char keypad_input[20];
extern char admin_password[6];
extern char input_password[6];	//array for entered password
extern char user_password[4];

GPIO_RegDef_t * LOCK_GPIO_PORT = GPIOD;

GPIO_PIN_t LOCK_GPIO_PIN = PIN_3;

void welcome_message(void)
{
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 50, "LUTFEN BiR    TUSA BASINIZ", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	return;
}

void main_menu(void)
{
	Key = NOT_PRESSED;
	Key = KEYPAD_Read();
	if(Key != NOT_PRESSED)
	{
		if(Key == '*')
		{
			Key = NOT_PRESSED;
			while(Key == NOT_PRESSED)
			{
				Key = NOT_PRESSED;
				Key = KEYPAD_Read();
				if(Key != NOT_PRESSED)
				{
					if(Key == 'A'){SystemState = 3; return;}
					if(Key != 'A' && Key != NOT_PRESSED){SearchDatabase(); return;}
					else continue;

				}
			}
		}
		if(Key == NOT_PRESSED){SystemState = 0; return;}
		else{SearchDatabase(); return;}
	}
}

void enter_password(void)
{
	counter_1s = 0;
	digit_number = 0;
	if(SystemState == USR_PWD_REQUESTED){ST7735_FillScreen(ST7735_WHITE);ST7735_DrawString(5, 40, "SiFRE GiRiNiZ:", Font_11x18, ST7735_BLUE, ST7735_WHITE);}
	if(AdminState == USR_PWD_SCR){ST7735_FillScreen(ST7735_BLACK);ST7735_DrawString(5, 10, "YENi KULLANICI SiFRESiNi GiRiNiZ:", Font_11x18, ST7735_GREEN, ST7735_BLACK);}
	if(SystemState == ADM_PWD_REQUESTED){ST7735_InvertColors(false);ST7735_FillScreen(ST7735_BLACK);ST7735_DrawString(5, 40, "ADMiN SiFRESiNi GiRiNiZ:", Font_7x10, ST7735_GREEN, ST7735_BLACK);}
	if(AdminState == ADM_PWD_SCR){ST7735_FillScreen(ST7735_BLACK);ST7735_DrawString(5, 10, "YENi ADMiNiSTRATOR SiFRESiNi GiRiNiZ:", Font_11x18, ST7735_GREEN, ST7735_BLACK);}
	if(SystemState == USR_PWD_REQUESTED || AdminState == USR_PWD_SCR)
	{
		while(digit_number < 4)
		{
			if(counter_1s < 10)
			{
				Key = NOT_PRESSED;
				Key = KEYPAD_Read();
				if(Key != NOT_PRESSED)
				{
					if(Key == '0'){input_password[digit_number] = '0'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '1'){input_password[digit_number] = '1'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '2'){input_password[digit_number] = '2'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '3'){input_password[digit_number] = '3'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '4'){input_password[digit_number] = '4'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '5'){input_password[digit_number] = '5'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '6'){input_password[digit_number] = '6'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '7'){input_password[digit_number] = '7'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '8'){input_password[digit_number] = '8'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == '9'){input_password[digit_number] = '9'; digit_number++; ST7735_DrawString(4+digit_number*27.5, 100, "*", Font_16x26, ST7735_BLUE, ST7735_WHITE); delay_ms(70); continue;}
					if(Key == 'A'){delay_ms(70); continue;}
					if(Key == 'B'){delay_ms(70); continue;}
					if(Key == 'C'){ST7735_DrawString(4+(digit_number)*27.5, 100, " ", Font_16x26, ST7735_BLUE, ST7735_WHITE); digit_number--; delay_ms(70); continue;}
					if(Key == 'D'){delay_ms(70); continue;}
					if(Key == '*'){delay_ms(70); continue;}
					if(Key == '#'){delay_ms(70); continue;}
				}
			}
			else{timeout_message_screen(); return;} //time is over
		 }
		if(digit_number == 4 && SystemState == 1)		//user entered their password
		{
			digit_number = 0;
			SystemState = 2;
			return;
		}
		if(digit_number == 4 && AdminState == USR_PWD_SCR)		//admin has changed user password
		{
			user_password[0] = input_password[0];
			user_password[1] = input_password[1];
			user_password[2] = input_password[2];
			user_password[3] = input_password[3];

			input_password[0] = admin_password[0];
			input_password[1] = admin_password[1];
			input_password[2] = admin_password[2];
			input_password[3] = admin_password[3];
			digit_number = 0;
			AdminState = PWD_OPERATIONS_SCR;
		}
	}
	if(SystemState == 3 || AdminState == ADM_PWD_SCR)
	{
		while(digit_number < 6)
		{
			if(counter_1s < 20)
			{
				Key = NOT_PRESSED;
				Key = KEYPAD_Read();
				if(Key != NOT_PRESSED)
				{
					if(Key == '0'){input_password[digit_number] = '0'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '1'){input_password[digit_number] = '1'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '2'){input_password[digit_number] = '2'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '3'){input_password[digit_number] = '3'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '4'){input_password[digit_number] = '4'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '5'){input_password[digit_number] = '5'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '6'){input_password[digit_number] = '6'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '7'){input_password[digit_number] = '7'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '8'){input_password[digit_number] = '8'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '9'){input_password[digit_number] = '9'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == 'A'){input_password[digit_number] = 'A'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == 'B'){input_password[digit_number] = 'B'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == 'C'){input_password[digit_number] = 'C'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == 'D'){input_password[digit_number] = 'D'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '*'){input_password[digit_number] = '*'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
					if(Key == '#'){input_password[digit_number] = '#'; digit_number++; ST7735_DrawString(4+digit_number*18, 100, "*", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
				}
			}
			else{timeout_message_screen(); return;} //time is over
		}
		if(digit_number == 6 && SystemState == ADM_PWD_REQUESTED)
		{//user has entered admin password right
			SystemState = ADM_ACCESS_GRANTED;
			digit_number = 0;
			return;
		}
		if(digit_number == 6 && AdminState == ADM_PWD_SCR)
		{//admin has changed admin password
			admin_password[0] = input_password[0];
			admin_password[1] = input_password[1];
			admin_password[2] = input_password[2];
			admin_password[3] = input_password[3];
			admin_password[4] = input_password[4];
			admin_password[5] = input_password[5];
			digit_number = 0;
			AdminState = PWD_OPERATIONS_SCR;
		}
	}
}


void first_step_confirmed(uint16_t fid, uint16_t score)
{
	ST7735_FillScreen(ST7735_GREEN);
	ST7735_Print(25, 30, "PARMAK iZi       BULUNDU        ID =", fid, Font_11x18, ST7735_WHITE, ST7735_GREEN);
	SystemState = USR_PWD_REQUESTED;
	delay_ms(400);
}

void change_password_menu(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "DEGiSTiRMEK iSTEDiGiNiZ SiFREYi SECiN", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(0, 30, "A-KULLANICI                                   B-ADMINISTRATOR                               D-GERi", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){change_user_password_warning(); return;}
			if(Key == 'B'){change_admin_password_warning();return;}
			if(Key == 'D'){SystemState = ADM_ACCESS_GRANTED; return;}
			if(Key == '#')
			{
				SystemState = 0;
				AdminState = 0;
				input_password[0] = '\0';
				input_password[1] = '\0';
				input_password[2] = '\0';
				input_password[3] = '\0';
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
				delay_ms(2000);
				welcome_message(); return;
			}else change_password_menu();
		}
	}
}

void database_operations_menu(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "YAPMAK iSTEDiGiNiZ     VERiTABANI iSLEMiNi    SECiNiZ", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(5, 40, "A-PARMAK iZi KAYDI                             B-VERiTABANIN SiLiNMESi                                             D-GERi", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){EnrollNewFingerPrint(); return;}
			if(Key == 'B'){AdminState = RESET_DB;  return;}
			if(Key == 'D'){SystemState = ADM_ACCESS_GRANTED; return;}
			if(Key == '#')
			{
				SystemState = 0;
				AdminState = 0;
				input_password[0] = '\0';
				input_password[1] = '\0';
				input_password[2] = '\0';
				input_password[3] = '\0';
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
				delay_ms(2000);
				welcome_message(); return;
			}
			else database_operations_menu();
			}
	}
}

void empty_DB_warning(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "VERiTABANI TAMAMEN SiLiNECEKTiR. DEVAM  ETMEK iSTER MiSiNiZ?", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(0, 80, "A-EVET D-HAYIR", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){empty_DB(); return;}
			if(Key == 'D'){AdminState = DB_OPERATION_SCR; return;}
			if(Key == '#'){
			SystemState = 0;
			AdminState = 0;
			input_password[0] = '\0';
			input_password[1] = '\0';
			input_password[2] = '\0';
			input_password[3] = '\0';
			ST7735_FillScreen(ST7735_BLACK);
			ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
			delay_ms(2000);
			welcome_message(); return;}
			else empty_DB_warning();
		}
	}
}

void change_admin_password_warning(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "ADMiNiSTRATOR SiFRESi DEGiSTiRiLECEKTiR.     DEVAM  ETMEK iSTER MiSiNiZ?", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(0, 80, "A-EVET D-HAYIR", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){AdminState = ADM_PWD_SCR;return;}
			if(Key == 'D'){return;}
			if(Key == '#')
			{
				SystemState = 0;
				AdminState = 0;
				input_password[0] = '\0';
				input_password[1] = '\0';
				input_password[2] = '\0';
				input_password[3] = '\0';
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
				delay_ms(2000);
				welcome_message(); return;
			}else change_admin_password_warning();
		}
	}
}

void change_user_password_warning(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "KULLANICI SiFRESi DEGiSTiRiLECEKTiR. DEVAM  ETMEK iSTER MiSiNiZ?", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(0, 80, "A-EVET D-HAYIR", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){AdminState = USR_PWD_SCR;return;}
			if(Key == 'D'){return;}
			if(Key == '#')
			{
				SystemState = 0;
				AdminState = 0;
				input_password[0] = '\0';
				input_password[1] = '\0';
				input_password[2] = '\0';
				input_password[3] = '\0';
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
				delay_ms(2000);
				welcome_message(); return;
			}else change_user_password_warning();
		}
	}
}

void set_alarm_menu(void)
{
	ST7735_InvertColors(false);
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_DrawString(0, 0, "ALARM SURESi DEGiSTiRiLECEKTiR. DEVAM  ETMEK iSTER MiSiNiZ?", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	ST7735_DrawString(0, 80, "A-EVET D-HAYIR", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	Key = NOT_PRESSED;
	while(Key == NOT_PRESSED)
	{
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'A'){AdminState = SET_ALRM_SCR; ST7735_FillScreen(ST7735_BLACK); return;}
			if(Key == 'D'){admin_profile(); return;}
			if(Key == '#')
			{
				SystemState = 0;
				AdminState = 0;
				input_password[0] = '\0';
				input_password[1] = '\0';
				input_password[2] = '\0';
				input_password[3] = '\0';
				ST7735_FillScreen(ST7735_BLACK);
				ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
				delay_ms(2000);
				welcome_message(); return;
			}else set_alarm_menu();
		}
	}
}

void set_alarm_time(void)
{
	uint8_t save = 0;
	ST7735_InvertColors(false);
	ST7735_DrawString(0, 0, "YENi ALARM SURESiNi TAYiN EDiNiZ[DEFAULT=15] (KAYDETMEK iCiN 'D' TUSUNA BASINIZ)", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	keycheck:
	while(digit_number < 3 && save != 1)
	{
		Key = NOT_PRESSED;
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			typing:
			if(Key == '0'){keypad_input[digit_number] = '0'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "0", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '1'){keypad_input[digit_number] = '1'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "1", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '2'){keypad_input[digit_number] = '2'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "2", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '3'){keypad_input[digit_number] = '3'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "3", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '4'){keypad_input[digit_number] = '4'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "4", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '5'){keypad_input[digit_number] = '5'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "5", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '6'){keypad_input[digit_number] = '6'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "6", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '7'){keypad_input[digit_number] = '7'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "7", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '8'){keypad_input[digit_number] = '8'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "8", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == '9'){keypad_input[digit_number] = '9'; digit_number++; ST7735_DrawString(30+digit_number*27.5, 100, "9", Font_16x26, ST7735_GREEN, ST7735_BLACK); delay_ms(70); continue;}
			if(Key == 'A'){delay_ms(70); continue;}
			if(Key == 'B'){delay_ms(70); continue;}
			if(Key == 'C'){ST7735_DrawString(30+digit_number*27.5, 100, " ", Font_16x26, ST7735_GREEN, ST7735_BLACK); digit_number--; delay_ms(70); continue;}
			if(Key == 'D'){save++; delay_ms(300); continue;}
			if(Key == '*'){delay_ms(70); continue;}
			if(Key == '#'){delay_ms(70); continue;}
		}
	}
	if(save == 1 && digit_number == 1)
	{//admin entered new alarm value
		alarm_time = (int)keypad_input[0]-48;
		digit_number = 0;
		SystemState = 4;
		return;
	}
	if(save == 1 && digit_number == 2)
	{//admin entered new alarm value
		alarm_time = 10*((int)keypad_input[0]-48)+((int)keypad_input[1]-48);
		digit_number = 0;
		SystemState = 4;
		return;
	}
	if(save != 1 && digit_number == 3)
	{
		Key = NOT_PRESSED;
		Key = KEYPAD_Read();
		if(Key != NOT_PRESSED)
		{
			if(Key == 'C')
			{
				ST7735_DrawString(30+digit_number*27.5, 100, " ", Font_16x26, ST7735_GREEN, ST7735_BLACK);
				digit_number--;
				goto keycheck;
			}
			if(Key == 'D')
			{
				save++;
				digit_number = 0;
				alarm_time = 100*((int)keypad_input[0]-48)+10*((int)keypad_input[1]-48)+((int)keypad_input[2]-48);
				SystemState = 4;
				return;
			}
			else
			{
				ST7735_DrawString(30+digit_number*27.5, 100, " ", Font_16x26, ST7735_GREEN, ST7735_BLACK);
				digit_number = 2;
				goto typing;
			}

		}
	}
}

void timeout_message_screen(void)
{

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(25, 35, "SURE DOLDU.   YENIDEN GIRIS     YAPINIZ.", Font_11x18, ST7735_BLUE, ST7735_WHITE);
//	buzzer_alert(errorSound);
	delay_ms(500);
	SystemState = 0;
	welcome_message();
	return;
}

void lock_access_granted(void)
{
	counter_1s = 0;
	if(counter_1s < 20)
	{

		if(user_password[0] == input_password[0] && user_password[1] == input_password[1] && user_password[2] == input_password[2] && user_password[3] == input_password[3])
		{
			ST7735_FillScreen(ST7735_GREEN);
			ST7735_DrawString(15, 50, "SiFRE DOGRU", Font_11x18, ST7735_WHITE, ST7735_GREEN);
			delay_ms(100);
			input_password[0] = '\0';
			input_password[1] = '\0';
			input_password[2] = '\0';
			input_password[3] = '\0';
			open_lock();
		}
		else
		{
			SystemState = 0;
			input_password[0] = '\0';
			input_password[1] = '\0';
			input_password[2] = '\0';
			input_password[3] = '\0';
//			buzzer_alert(alarm);
			welcome_message();
			return;
		}
	}else{timeout_message_screen(); return;} //time is over
}

void admin_profile(void)
{
	counter_1s = 0;
	ST7735_InvertColors(false);
	if(admin_password[0] == input_password[0] && admin_password[1] == input_password[1] && admin_password[2] == input_password[2] && admin_password[3] == input_password[3] && admin_password[4] == input_password[4] && admin_password[5] == input_password[5])
	{
		ST7735_InvertColors(false);
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_DrawString(5, 0, "iSLEM LiSTESi:", Font_11x18, ST7735_GREEN, ST7735_BLACK);
		ST7735_DrawString(0, 30, "A-PARMAK iZi           VERiTABANI iSLEMLERi                         B-SiFRE iSLEMLERi                            C-ALARM iSLEMLERi                             D-RESET", Font_7x10, ST7735_GREEN, ST7735_BLACK);
		Key = NOT_PRESSED;
		while(Key == NOT_PRESSED)
		{
			Key = KEYPAD_Read();
			if(Key != NOT_PRESSED)
			{
				if(Key == 'A'){SystemState = 0; AdminState = DB_OPERATION_SCR; return;}
				if(Key == 'B'){SystemState = 0; AdminState = PWD_OPERATIONS_SCR; return;}
				if(Key == 'C'){SystemState = 0; AdminState = ALRM_MENU_SCR; return;}
				if(Key == 'D')
				{//reboot
					SCB->AIRCR = 0x05FA0000 | (1 << 2);
				}
				if(Key == '#')
				{
					AdminState = 0;
					SystemState = 0;
					input_password[0] = '\0';
					input_password[1] = '\0';
					input_password[2] = '\0';
					input_password[3] = '\0';
					ST7735_FillScreen(ST7735_BLACK);
					ST7735_DrawString(60, 50, "BYE!", Font_11x18, ST7735_GREEN, ST7735_BLACK);
					delay_ms(2000);
					welcome_message(); return;
				}
				else admin_profile();
			}
		}

	}
	else
	{
		SystemState = 0;
		input_password[0] = '\0';
		input_password[1] = '\0';
		input_password[2] = '\0';
		input_password[3] = '\0';
//		buzzer_alert(alarm);
		welcome_message();
		return;
	}
}

void open_lock(void)
{
	GPIO_WritePin(LOCK_GPIO_PORT, LOCK_GPIO_PIN, RESET);	//relay open
//	buzzer_alert(accesGrantedSound);
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 45, "GECMEK iCiN     10 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      9 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      8 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      7 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      6 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      5 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_DrawString(20, 45, "GECMEK iCiN      4 SANiYE", Font_11x18, ST7735_BLUE, ST7735_WHITE);
	delay_ms(1000);
	ST7735_FillScreen(ST7735_CYAN);
	ST7735_DrawString(20, 45, "GECMEK iCiN     3 SANiYE!", Font_11x18, ST7735_WHITE, ST7735_CYAN);
	delay_ms(500);
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 45, "GECMEK iCiN     3 SANiYE!", Font_11x18, ST7735_CYAN, ST7735_WHITE);
	delay_ms(500);
	ST7735_FillScreen(ST7735_GREEN);
	ST7735_DrawString(20, 45, "GECMEK iCiN     2 SANiYE!!", Font_11x18, ST7735_WHITE, ST7735_GREEN);
	delay_ms(500);
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 45, "GECMEK iCiN     2 SANiYE!!", Font_11x18, ST7735_GREEN, ST7735_WHITE);
	delay_ms(500);
	ST7735_FillScreen(ST7735_MAGENTA);
	ST7735_DrawString(20, 45, "GECMEK iCiN   SON 1 SANiYE!", Font_11x18, ST7735_WHITE, ST7735_MAGENTA);
	delay_ms(500);
	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(20, 45, "GECMEK iCiN   SON 1 SANiYE!", Font_11x18, ST7735_MAGENTA, ST7735_WHITE);
	delay_ms(1000);
	ST7735_FillScreen(ST7735_WHITE);
//	buzzer_alert(accesGrantedSound);
	GPIO_WritePin(LOCK_GPIO_PORT, LOCK_GPIO_PIN, SET); //relay closed
	welcome_message();
	SystemState = 0;
	AdminState = 0;
	input_password[0] = '\0';
	input_password[1] = '\0';
	input_password[2] = '\0';
	input_password[3] = '\0';
	return;
}

void empty_DB(void)
{
	uint8_t empty_status = R308_EmptyDatabase(&R308);
	if(empty_status == 0)
	{
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_DrawString(25, 40, "VERiTABANI      SiLiNDi", Font_11x18, ST7735_GREEN, ST7735_BLACK);
		delay_ms(1000);
		EnrollNewFingerPrint();
		database_operations_menu();
	}else
	{
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_DrawString(25, 50, "HATA OLUSTU", Font_11x18, ST7735_RED, ST7735_BLACK);
		delay_ms(500);admin_profile();
	}
}

void buzzer_alert(Sound_t sound)
{
	int k = 0;
	switch(sound)
	{
		case SOUND_ALARM:
				ST7735_InvertColors(false);
				ST7735_FillScreen(ST7735_RED);
				ST7735_DrawString(25, 50, "PARMAGINIZ    KAYITLI DEGiL   VEYA SiFRE       YANLIS!", Font_11x18, ST7735_WHITE, ST7735_RED);
				alert_icon();
				while(k < alarm_time)
				{
					for(int i=0; i<255; i++)
					{
						BUZZER.CCR1 = i;
						delay_ms(1);
					}
					ST7735_FillScreen(ST7735_BLUE);
					ST7735_DrawString(25, 50, "PARMAGINIZ    KAYITLI DEGiL   VEYA SiFRE       YANLIS!", Font_11x18, ST7735_WHITE, ST7735_BLUE);
					alert_icon();
					for(int i=100; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(1);
					}
					ST7735_FillScreen(ST7735_RED);
					ST7735_DrawString(25, 50, "PARMAGINIZ    KAYITLI DEGiL   VEYA SiFRE       YANLIS!", Font_11x18, ST7735_WHITE, ST7735_RED);
					alert_icon();
					k++;
				}
				BUZZER.CCR1 = 0;
				welcome_message();
			break;
		case SOUND_ACCESS_GRANTED:
				while(k < 10)
				{
					for(int i=0; i<10; i++)
					{
						BUZZER.CCR1 = i;
						delay_ms(3);
					}

					for(int i=10; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(3);
					}
					k++;
				}
				BUZZER.CCR1 = 0;
			break;
		case SOUND_ERROR:
				for(int i=5; i>0; i--)
				{
					BUZZER.CCR1 = i;
					delay_ms(45);
				}
				BUZZER.CCR1 = 0;
				delay_ms(15);
				for(int i=5; i>0; i--)
				{
					BUZZER.CCR1 = i;
					delay_ms(30);
				}
				BUZZER.CCR1 = 0;
			break;
		case SOUND_SENSOR_INIT:
					for(int i=5; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(20);
					}
					BUZZER.CCR1 = 0;
					delay_ms(100);
					for(int i=5; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(30);
					}
					BUZZER.CCR1 = 0;
					delay_ms(50);
					for(int i=5; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(60);
					}
					BUZZER.CCR1 = 0;
					delay_ms(1);
					for(int i=5; i>0; i--)
					{
						BUZZER.CCR1 = i;
						delay_ms(15);
					}
					BUZZER.CCR1 = 0;
			break;
			}
}
