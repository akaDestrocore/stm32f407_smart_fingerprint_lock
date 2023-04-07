#ifndef FINGERPRINT_MODULE_H_
#define FINGERPRINT_MODULE_H_

#include "stm32f407xx.h"
#include "r308.h"
#include "r308_uart.h"
#include "st7735.h"


/*function prototypes*/
void EnrollNewFingerPrint(void);
void SearchDatabase(void);
void EnrollFinger(int16_t fid);
uint16_t GetFreeID(int16_t * fid);
void EmptyDB(void);

#endif /* FINGERPRINT_MODULE_H_ */
