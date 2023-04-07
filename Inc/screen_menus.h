#ifndef SCREEN_MENUS_H_
#define SCREEN_MENUS_H_

#include "stm32f407xx.h"
#include "st7735.h"
#include "fingerprint_module.h"
#include "stm32f407xx_keypad_driver.h"
#include "kou.h"

typedef enum
{
	MAIN_MENU,
	USR_PWD_REQUESTED,
	USR_ACCESS_GRANTED,
	ADM_PWD_REQUESTED,
	ADM_ACCESS_GRANTED
}User_AccessStep_t;

typedef enum
{
	PWD_OPERATIONS_SCR,
	DB_OPERATION_SCR,
	DB_WARNING_SCR,
	USR_PWD_SCR,
	ADM_PWD_SCR,
	ALRM_MENU_SCR,
	SET_ALRM_SCR,
	RESET_DB
}Admin_Menu_t;

typedef enum
{
	SOUND_ALARM,
	SOUND_ACCESS_GRANTED,
	SOUND_ERROR,
	SOUND_SENSOR_INIT
}Sound_t;

/*
 * Menu functions prototypes
 */
void welcome_message(void);
void main_menu(void);
void enter_password(void);
void first_step_confirmed(uint16_t fid, uint16_t score);
void change_password_menu(void);
void database_operations_menu(void);
void empty_DB_warning(void);
void change_admin_password_warning(void);
void change_user_password_warning(void);
void set_alarm_menu(void);					//just another warning that you are going to change alarm working time
void set_alarm_time(void);
void timeout_message_screen(void);
void lock_access_granted(void);
void admin_profile(void);
void open_lock(void);
void empty_DB(void);
void buzzer_alert(Sound_t sound);


#endif /* SCREEN_MENUS_H_ */
