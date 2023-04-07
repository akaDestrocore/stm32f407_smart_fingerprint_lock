#ifndef R308_H_
#define R308_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "st7735.h"
#include "simple_delay.h"

#define R308_OK                      0x00
#define R308_HANDSHAKE_OK            0x55
#define R308_PACKETRECIEVEERR        0x01
#define R308_NOFINGER                0x02
#define R308_IMAGEFAIL               0x03
#define R308_IMAGEMESS               0x06
#define R308_FEATUREFAIL             0x07
#define R308_NOMATCH                 0x08
#define R308_NOTFOUND                0x09
#define R308_ENROLLMISMATCH          0x0A
#define R308_BADLOCATION             0x0B
#define R308_DBREADFAIL              0x0C
#define R308_UPLOADFEATUREFAIL       0x0D
#define R308_PACKETRESPONSEFAIL      0x0E
#define R308_UPLOADFAIL              0x0F
#define R308_DELETEFAIL              0x10
#define R308_DBCLEARFAIL             0x11
#define R308_PASSFAIL                0x13
#define R308_INVALIDIMAGE            0x15
#define R308_FLASHERR                0x18
#define R308_INVALIDREG              0x1A
#define R308_ADDRCODE                0x20
#define R308_PASSVERIFY              0x21

/*
 * Signatures and packet IDs
 */
#define R308_STARTCODE               0xEF01

#define R308_COMMANDPACKET           0x1
#define R308_DATAPACKET              0x2
#define R308_ACKPACKET               0x7
#define R308_ENDDATAPACKET           0x8

/*
 * Commands
 */
#define R308_GETIMAGE                0x01
#define R308_IMAGE2TZ                0x02
#define R308_REGMODEL                0x05
#define R308_STORE                   0x06
#define R308_LOAD                    0x07
#define R308_UPCHAR                  0x08
#define R308_DOWNCHAR                0x09
#define R308_IMGUPLOAD               0x0A
#define R308_DELETE                  0x0C
#define R308_EMPTYDATABASE           0x0D
#define R308_SETSYSPARAM             0x0E
#define R308_READSYSPARAM            0x0F
#define R308_VERIFYPASSWORD          0x13
#define R308_SEARCH                  0x04
#define R308_HISPEEDSEARCH           0x1B
#define R308_TEMPLATECOUNT           0x1D
#define R308_READTEMPLATEINDEX       0x1F
#define R308_PAIRMATCH               0x03
#define R308_SETPASSWORD             0x12
#define R308_STANDBY                 0x33

#define R308_HANDSHAKE               0x40

/* returned whenever we time out while reading */
#define R308_TIMEOUT                 -1
/* returned whenever we get an unexpected PID or length */
#define R308_READ_ERROR              -2
/* returned whenever there's no free ID */
#define R308_NOFREEINDEX             -1

#define R308_MAX_PACKET_LEN          256
#define R308_PKT_OVERHEAD_LEN        12

/* 32 is max packet length for ACKed commands, +1 for confirmation code */
#define R308_BUFFER_SZ               (32 + 1)

/* default timeout is 2 seconds */
#define R308_DEFAULT_TIMEOUT         2000
#define R308_TEMPLATES_PER_PAGE      32

#define R308_DEFAULT_PASSWORD        0x00000000
#define R308_DEFAULT_ADDRESS         0xFFFFFFFF

#define R308_GETRANDOM               0x14

/*
 * use these constants when setting system
 * parameters with the setParam() method
 */
typedef enum
{
	R308_SETPARAM_BAUD_RATE = 6,
	R308_SETPARAM_SECURITY_LEVEL = 3,
	R308_SETPARAM_PACKET_LEN = 3
}R308_Param_t;

/* possible values for system parameters that can be set with setParam() */

typedef enum
{
	R308_BAUD_9600,
	R308_BAUD_19200,
	R308_BAUD_28800,
	R308_BAUD_38400,
	R308_BAUD_48000,
	R308_BAUD_57600,
	R308_BAUD_67200,
	R308_BAUD_76800,
	R308_BAUD_86400,
	R308_BAUD_96000,
	R308_BAUD_105600,
	R308_BAUD_115200
}R308_Baudrate_t;

/*
 * security levels
 */
typedef enum
{
	R308_LVL_1,
	R308_LVL_2,
	R308_LVL_3,
	R308_LVL_4,
	R308_LVL_5
}R308_Security_t;

/*
 * packet lengths
 */
typedef enum
{
	R308_PLEN_32,
	R308_PLEN_64,
	R308_PLEN_128,
	R308_PLEN_256,
	R308_PLEN_NONE = 0xff
}R308_PacketLen_t;

/*
 * possible output containers for template/image data read from the module
 */
typedef enum
{
	R308_OUTPUT_TO_STREAM,
	R308_OUTPUT_TO_BUFFER
}R308_Output_t;

typedef struct
{
    uint16_t status_reg;
    uint16_t system_id;
    uint16_t capacity;
    uint16_t security_level;
    uint32_t device_addr;
    uint16_t packet_len;
    uint16_t baud_rate;
}R308_System_Params;

typedef uint16_t (*r308_uart_read_func)(uint8_t * bytes, uint16_t len);
typedef void (*r308_uart_write_func)(uint8_t * bytes, uint16_t len);
typedef uint16_t (*r308_uart_avail_func)(void);
typedef uint32_t (*r308_millis_func)(void);

typedef struct
{
	r308_uart_read_func read_func;
	r308_uart_write_func write_func;
	r308_uart_avail_func avail_func;

    uint32_t password;
    uint32_t address;

    uint8_t manual_settings;

    R308_System_Params sys_params;

    uint8_t buffer[R308_BUFFER_SZ];
}R308_t;

/* Default parameters

   status_reg: 0x0000,
   system_id: 0x0000,
   capacity: <Your-module-capacity>,
   security_level: R308_LVL_5,
   device_addr: 0xFFFFFFFF,
   packet_len: R308_PLEN_128,
   baud_rate: R308_BAUD_57600

*/

/*
 *	R308 initialization
 */
uint8_t R308_Begin(R308_t * pR308, r308_millis_func _millis_func);

/*
 *	R308 image functions
 */
int16_t R308_GetImage(R308_t * pR308);
int16_t R308_Image2Tz(R308_t * pR308, uint8_t slot);
int16_t R308_CreateModel(R308_t * pR308);

/*
 *	R308 database functions
 */
int16_t R308_EmptyDatabase(R308_t * pR308);
int16_t R308_StoreModel(R308_t * pR308, uint16_t id, uint8_t slot);

/*
 * loads template with ID #'id' from the database into buffer #'slot'
 */
int16_t R308_LoadModel(R308_t * pR308, uint16_t id, uint8_t slot);
int16_t R308_SetParam(R308_t * pR308, uint8_t param, uint8_t value);
int16_t R308_ReadParams(R308_t * pR308, R308_System_Params * user_params);
uint8_t R308_ReadRaw(R308_t * pR308, uint8_t outType, void * pOut, uint8_t * pRead_Complete, uint16_t * pRead_Len);
void R308_WriteRaw(R308_t * pR308, uint8_t * data, uint16_t len);

/*
 * initiates the transfer of the template in buffer #'slot' to the MCU
 */
int16_t R308_DownloadModel(R308_t * pR308, uint8_t slot);
int16_t R308_UploadModel(R308_t * pR308, uint8_t slot);
int16_t R308_DeleteModel(R308_t * pR308, uint16_t id, uint16_t how_many);
int16_t R308_SearchDatabase(R308_t * pR308, uint16_t * finger_id, uint16_t * score, uint8_t slot);
int16_t R308_GetTemplateCount(R308_t * pR308, uint16_t * template_cnt);
uint16_t R308_GetFreeIndex(R308_t * pR308, uint8_t page, int16_t * id);
int16_t R308_MatchTemplatePair(R308_t * pR308, uint16_t * score);
int16_t R308_SetPassword(R308_t * pR308, uint32_t pwd);
int16_t R308_GetRandomNumber(R308_t * pR308, uint32_t * number);

int16_t R308_Standby(R308_t * pR308);

uint8_t R308_Handshake(R308_t * pR308);

extern const uint16_t r308_packet_lengths[];

#endif
