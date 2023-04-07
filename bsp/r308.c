#include "r308.h"

/*
 *	helper function prototypes
 */
static void write_packet(R308_t * pR308, uint8_t packettype, uint8_t * packet, uint16_t len);
static int16_t get_reply(R308_t * pR308, uint8_t * replyBuf, uint16_t buflen, uint8_t * pktid, r308_uart_write_func out_stream);
static int16_t read_ack_get_response(R308_t * pR308, uint8_t * rc);

const uint16_t r308_packet_lengths[] = {32, 64, 128, 256};
static r308_millis_func millis_func;

/*
 *	R308 states enumeration
 */
typedef enum
{
    R308_STATE_READ_HEADER,
	R308_STATE_READ_ADDRESS,
	R308_STATE_READ_PID,
	R308_STATE_READ_LENGTH,
	R308_STATE_READ_CONTENTS,
	R308_STATE_READ_CHECKSUM
} R308_State;

/**************************************************************************************************************
 * @function name 		- R308_Begin
 *
 * @brief				- This function initiates the communication with the R308 finger print sensor module.
						  It verifies the password, reads the parameters and prepares the sensor for operation.
 *
 * @parameter[in]		- Pointer to R308_t structure containing the sensor information
 * @parameter[in]		- Pointer to a function that returns the current time in milliseconds
 *
 * @return				- Returns 1 if the initialization is successful, 0 otherwise
 *
 * @Note				- The function sets the manual_settings flag in R308_t structure to false if the
 * 						  parameter reading operation is successful. This flag is used to check whether
 * 						  parameters need to be manually set or read from the sensor. The function waits
 * 						  for 500ms to establish communication with the sensor.
 **************************************************************************************************************/
uint8_t R308_Begin(R308_t * pR308, r308_millis_func _millis_func)
{
    millis_func = _millis_func;

    uint32_t start = millis_func();
    while (millis_func() - start < 1000);   // 500 ms at least according to datasheet

    pR308->buffer[0] = R308_VERIFYPASSWORD;
    pR308->buffer[1] = (pR308->password >> 24) & 0xff; pR308->buffer[2] = (pR308->password >> 16) & 0xff;
    pR308->buffer[3] = (pR308->password >> 8) & 0xff; pR308->buffer[4] = pR308->password & 0xff;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 5);

    uint8_t confirm_code = 0;
    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0 || confirm_code != R308_OK)
        return 0;

    if (!pR308->manual_settings && R308_ReadParams(pR308, NULL) != R308_OK)
        return 0;

		else return 1;
}

/*********************************************************************************************************
 * @function name 		- R308_SetPassword
 *
 * @brief				- This function sets the password for the R308 finger print sensor by writing it
 * 						  to the device's internal memory. The password is a 32-bit integer, which is
 * 						  split into 4 bytes and written to the buffer.
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- password to be set
 *
 * @return				- confirmation code or response code
 *
 * @Note				- This function writes the password to the device's internal memory and returns
 * 						 a confirmation code or response code indicating success or failure. The password
 * 						 is a 32-bit integer and is split into 4 bytes and written to the buffer.
 *********************************************************************************************************/
int16_t R308_SetPassword(R308_t * pR308, uint32_t pwd)
{
    pR308->buffer[0] = R308_SETPASSWORD;
    pR308->buffer[1] = (pwd >> 24) & 0xff; pR308->buffer[2] = (pwd >> 16) & 0xff;
    pR308->buffer[3] = (pwd >> 8) & 0xff; pR308->buffer[4] = pwd & 0xff;

    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 5);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/*******************************************************************************************************
 * @function name 		- R308_GetImage
 *
 * @brief				- This function sends a command to the R308 finger print scanner to capture an
 *  					  image of a finger print and retrieve it from the scanner's buffer.
 *
 * @parameter[in]		- pointer to R308 structure
 *
 * @return				- confirmation code or response code
 *
 * @Note				- This function requires a previous initialization of the R308 structure and
 * 						  communication with the finger print scanner. It returns a confirmation code if
 * 						  the operation was successful, or a response code if there was an error during
 * 						  communication or image capturing.
 ********************************************************************************************************/
int16_t R308_GetImage(R308_t * pR308)
{
    pR308->buffer[0] = R308_GETIMAGE;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/*****************************************************************************************************
 * @function name 		- R308_Standby
 *
 * @brief				- This function puts the R308 finger print sensor module in standby mode
 *
 * @parameter[in]		- pointer to R308 structure
 *
 * @return				- confirmation code or response code; returns negative value in case of error
 *
 * @Note				- none
 ******************************************************************************************************/
int16_t R308_Standby(R308_t * pR308)
{
    pR308->buffer[0] = R308_STANDBY;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/*********************************************************************************************************
 * @function name 		- R308_Image2Tz
 *
 * @brief				- This function converts the image stored in the R308 structure buffer to a
 * 						  character file and stores it in the designated slot in the R308 CharBuffer page.
 *
 * @parameter[in]		- pointer to R308 structure containing the image to be converted
 * @parameter[in]		- designated slot number in the R308 CharBuffer page (R308_BUFFER_SZ)
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the image-to-template conversion process. It then reads the acknowledgment packet
 * 						  and response packet from the sensor to obtain the confirmation code. A negative
 * 						  return value indicates a failure in reading the response packet from the sensor.
 **********************************************************************************************************/
int16_t R308_Image2Tz(R308_t * pR308, uint8_t slot)
{
    pR308->buffer[0] = R308_IMAGE2TZ;
    pR308->buffer[1] = slot;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 2);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/************************************************************************************************************
 * @function name 		- R308_CreateModel
 *
 * @brief				- This function creates a finger print template from the characters stored in the
 * 						  designated slots in the R308 CharBuffer page.
 *
 * @parameter[in]		- pointer to R308 structure containing the designated slot numbers in the CharBuffer
 * 						  page
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the template creation process. It then reads the acknowledgment packet and response
 * 						  packet from the sensor to obtain the confirmation code. A negative return value
 * 						  indicates a failure in reading the response packet from the sensor.
 *************************************************************************************************************/
int16_t R308_CreateModel(R308_t * pR308)
{
    pR308->buffer[0] = R308_REGMODEL;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/*****************************************************************************************************
 * @function name 		- R308_StoreModel
 *
 * @brief				- This function stores a model with the specified ID in the specified slot of the
 * 						  finger print module
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- ID of the model to be stored
 * @parameter[in]		- slot number in which the model will be stored (R308_BUFFER_SZ)
 *
 * @return				- confirmation code or response code
 *
 * @Note				- This function sends a command packet to the finger print module to store the model
 ******************************************************************************************************/
int16_t R308_StoreModel(R308_t * pR308, uint16_t id, uint8_t slot)
{
    pR308->buffer[0] = R308_STORE;
    pR308->buffer[1] = slot;
    pR308->buffer[2] = id >> 8; pR308->buffer[3] = id & 0xFF;

    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 4);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/*****************************************************************************************************
 * @function name 		- R308_LoadModel
 *
 * @brief				- This function loads a pre-existing finger print model from the flash library
 * 						  to the specified character buffer page of the R308 finger print sensor module
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- ID number of the pre-existing finger print model to be loaded
 * @parameter[in]		- designated slot number in CharBuffer page (R308_BUFFER_SZ)
 *
 * @return				- confirmation code or response code
 *
 * @Note				- The function uses the R308_LOAD command to load the finger print model to the
 * 						  specified slot in the R308's character buffer page. The function returns a
 * 						  confirmation code if the model was successfully loaded, or a response code if
 * 						  an error occurred during the loading process.
 ******************************************************************************************************/
int16_t R308_LoadModel(R308_t * pR308, uint16_t id, uint8_t slot)
{
    pR308->buffer[0] = R308_LOAD;
    pR308->buffer[1] = slot;
    pR308->buffer[2] = id >> 8; pR308->buffer[3] = id & 0xFF;

    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 4);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/***************************************************************************************************************
 * @function name 		- R308_SetParam
 *
 * @brief				- This function sets a specified parameter value in the R308 structure
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- parameter to be set
 * @parameter[in]		- value of the parameter to be set
 *
 * @return				- confirmation code or response code
 *
 * @Note				- If the manual settings flag is set in the R308 structure, then the function
 *                        will return with an error code R308_PACKETRECIEVEERR. Otherwise, it will set
 *                        the system parameter and return a confirmation code or response code. It also
 *                        waits for 100ms before reading and updating the parameter values in the R308 structure.
 ***************************************************************************************************************/
int16_t R308_SetParam(R308_t * pR308, uint8_t param, uint8_t value)
{
    if (pR308->manual_settings) {
        return R308_PACKETRECIEVEERR;
    }

	pR308->buffer[0] = R308_SETSYSPARAM;
    pR308->buffer[1] = param; pR308->buffer[2] = value;

	write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 3);
    uint8_t confirm_code = 0;
    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    /* gets weird if you dont wait */
    uint32_t start = millis_func();
    while (millis_func() - start < 100);

    R308_ReadParams(pR308, NULL);
    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- reverse_bytes
 *
 * @brief				- This function takes in a pointer to a memory location and a size in bytes and
 * 						  reverses the byte order of the memory block in place. The memory block is
 * 						  treated as an array of bytes and the function swaps the bytes at the first and
 * 						  last position, then the second and second-to-last position, and so on until the
 * 						  entire block has been reversed.
 *
 * @parameter[in]		- start: a pointer to the memory location to reverse the bytes for
 * @parameter[in]		- size: the size of the memory block in bytes to reverse
 *
 * @return				- none
 *
 * @Note				- This function does not allocate any additional memory for the operation,
 * 						  and the caller must ensure that the memory block is properly allocated and
 * 						  initialized before calling this function. This function is useful for reversing
 * 						  the byte order of data that is stored in a different byte order than what is
 * 						  expected by the current system architecture or data format.
 ***********************************************************************************************************/
static void reverse_bytes(void *start, uint16_t size)
{
    uint8_t *lo = (uint8_t *)start;
    uint8_t *hi = (uint8_t *)start + size - 1;
    uint8_t swap;
    while (lo < hi) {
        swap = *lo;
        *lo++ = *hi;
        *hi-- = swap;
    }
}

/**********************************************************************************************************
 * @function name 		- R308_ReadParams
 *
 * @brief				- This function reads the system parameters from the R308 finger print sensor and
 * 						  stores them in the provided structure or the internal R308 structure buffer.
 *
 * @parameter[in]		- pointer to the R308 finger print sensor structure
 * @parameter[in]		- pointer to the user-defined system parameters structure
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to read the
 * 						  system parameters. It then reads the acknowledgment packet and response packet
 * 						  from the sensor to obtain the parameters. If a user-defined parameter structure
 * 						  is provided, it is populated with the system parameters. Otherwise, the parameters
 * 						  are stored in the internal R308 structure buffer.
 ***********************************************************************************************************/
int16_t R308_ReadParams(R308_t * pR308, R308_System_Params * pUser_Params)
{
    if (pR308->manual_settings)
    {
        if (pUser_Params != NULL)
            memcpy(pUser_Params, &pR308->sys_params, 16);
        return R308_OK;
    }

    pR308->buffer[0] = R308_READSYSPARAM;

	write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    if (len != 16) {
    	ST7735_Print(10, 10, "Unexpected params length:", len, Font_11x18, ST7735_GREEN, ST7735_BLACK);
		delay_ms(1000);
        return R308_READ_ERROR;
    }

    memcpy(&pR308->sys_params, &pR308->buffer[1], 16);
    reverse_bytes(&pR308->sys_params.status_reg, 2);
    reverse_bytes(&pR308->sys_params.system_id, 2);
    reverse_bytes(&pR308->sys_params.capacity, 2);
    reverse_bytes(&pR308->sys_params.security_level, 2);
    reverse_bytes(&pR308->sys_params.device_addr, 4);
    reverse_bytes(&pR308->sys_params.packet_len, 2);
    reverse_bytes(&pR308->sys_params.baud_rate, 2);

    if (pUser_Params != NULL)
        memcpy(pUser_Params, &pR308->sys_params, 16);

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_ReadRaw
 *
 * @brief				- This function reads the raw data output from the R308 finger print sensor and
 * 						  either stores it in a buffer or streams it out to a serial port.
 *
 * @parameter[in]		- pointer to R308 structure containing the initialized sensor object
 * @parameter[in]		- type of output, either R308_OUTPUT_TO_BUFFER to store in buffer or
 * 						  R308_OUTPUT_TO_STREAM to stream out to a serial port
 * @parameter[in]		- pointer to buffer or stream object depending on output type
 * @parameter[in]		- pointer to variable that stores whether the read operation is complete or not
 * @parameter[in]		- pointer to variable that stores the length of the data read from the sensor
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the reading process. It then reads the acknowledgment packet and response packet
 * 						  from the sensor to obtain the confirmation code and raw data. The raw data is
 * 						  either stored in the buffer specified by the caller or streamed out to a serial port.
 * 						  The function returns a confirmation code if the operation was successful, and a
 * 						  response code if it failed. A return value of 0 indicates failure, while a return
 * 						  value of 1 indicates success.
 ***********************************************************************************************************/
uint8_t R308_ReadRaw(R308_t * pR308, uint8_t outType, void * pOut, uint8_t * pRead_Complete, uint16_t * pRead_Len)
{
    r308_uart_write_func out_stream;
    uint8_t * outBuf;

    if (outType == R308_OUTPUT_TO_BUFFER)
        outBuf = (uint8_t *)pOut;
    else if (outType == R308_OUTPUT_TO_STREAM)
        out_stream = (r308_uart_write_func)pOut;
    else
        return 0;

    uint8_t pid;
    int16_t len;

    /* read into a buffer or straight to a serial port */
    if (outType == R308_OUTPUT_TO_BUFFER)
        len = get_reply(pR308, outBuf, *pRead_Len, &pid, NULL);
    else if (outType == R308_OUTPUT_TO_STREAM)
        len = get_reply(pR308, NULL, 0, &pid, out_stream);

    /* check that the length is > 0 */
    if (len <= 0) {
		ST7735_Print(10, 10, "Wrong read length: %.f", len, Font_11x18, ST7735_GREEN, ST7735_BLACK);
		delay_ms(1000);
        return 0;
    }

    *pRead_Complete = 0;

    if (pid == R308_DATAPACKET || pid == R308_ENDDATAPACKET) {
        if (outType == R308_OUTPUT_TO_BUFFER)
            *pRead_Len = len;
        if (pid == R308_ENDDATAPACKET)
            *pRead_Complete = 1;
        return 1;
    }

    return 0;
}

/**********************************************************************************************************
 * @function name 		- R308_WriteRaw
 *
 * @brief				- This function writes raw data to the R308 finger print sensor by breaking it down
 * 						  into smaller chunks and sending them as separate data packets.
 *
 * @parameter[in]		- pointer to R308 structure containing the sensor's system parameters
 * @parameter[in]		- pointer to the data to be written
 * @parameter[in]		- length of the data to be written

 * @return				- none
 *
 * @Note				- This function writes data to the R308 finger print sensor using a protocol that
 * 						  requires data to be sent in packets of a fixed size. The function breaks down
 * 						  the data into smaller chunks and sends each chunk as a separate data packet.
 * 						  It first determines the packet size based on the system parameters stored in
 * 						  the R308 structure. It then sends the data as packets, one at a time, until all
 * 						  the data has been written to the sensor. The final packet is marked as the end of
 * 						  data packet.
 ***********************************************************************************************************/
void R308_WriteRaw(R308_t * pR308, uint8_t * data, uint16_t len)
{
    uint16_t written = 0;
    uint16_t chunk_sz = r308_packet_lengths[pR308->sys_params.packet_len];

    while (len > chunk_sz) {
        write_packet(pR308, R308_DATAPACKET, &data[written], chunk_sz);
        written += chunk_sz;
        len -= chunk_sz;
    }
    write_packet(pR308, R308_ENDDATAPACKET, &data[written], len);
}

/**********************************************************************************************************
 * @function name 		- R308_DownloadModel
 *
 * @brief				- This function is used to download a finger print template from the R308 finger
 * 					      print sensor module's CharBuffer to the host computer
 *
 * @parameter[in]		- pointer to an R308 structure containing the image to be converted.
 * @parameter[in]		- designated slot number in the R308 CharBuffer page (R308_BUFFER_SZ).

 * @return				- confirmation code or response code indicating success or failure of the operation.
 *
 * @Note				- It works by sending a command packet to the sensor module to initiate the download
 * 						  process, then reading the acknowledgement and response packets to obtain the
 * 						  confirmation code. The template is stored in the sensor module's CharBuffer,
 * 						  which is a temporary storage area for finger print templates before they are
 * 						  uploaded to the host computer or stored in the module's flash memory.
 * 						  The slot parameter specifies which slot in the CharBuffer the template is stored in,
 * 						  and the function returns a confirmation code or response code indicating the success
 * 						  or failure of the operation.
 ***********************************************************************************************************/
int16_t R308_DownloadModel(R308_t * pR308, uint8_t slot)
{
    pR308->buffer[0] = R308_UPCHAR;
    pR308->buffer[1] = slot;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 2);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_UploadModel
 *
 * @brief				- This function sends a command packet to the R308 finger print sensor to request
 * 					      the transfer of a finger print template from the designated slot in the R308
 * 					      CharBuffer page to the host computer.
 *
 * @parameter[in]		- pointer to R308 structure containing the image to be converted
 * @parameter[in]		- designated slot number in the R308 CharBuffer page (R308_BUFFER_SZ)

 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the template-to-image transfer process. It then reads the acknowledgment packet
 * 						  and response packet from the sensor to obtain the confirmation code. A negative
 * 						  return value indicates a failure in reading the response packet from the sensor.
 ***********************************************************************************************************/
int16_t R308_UploadModel(R308_t * pR308, uint8_t slot)
{
    pR308->buffer[0] = R308_DOWNCHAR;
    pR308->buffer[1] = slot;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 2);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_DeleteModel
 *
 * @brief				- This function deletes one or more finger print templates from the R308 finger print
 * 					      sensor's internal memory based on the provided ID and the number of templates to
 * 					      delete
 *
 * @parameter[in]		- pointer to R308 structure containing the communication parameters and buffer
 * @parameter[in]		- ID of the first template to delete
 * @parameter[in]		- number of templates to delete
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to delete one
 * 						  or more templates with IDs ranging from the provided ID to ID + how_many. It then
 * 						  reads the acknowledgment packet and response packet from the sensor to obtain the
 * 						  confirmation code. A negative return value indicates a failure in reading the
 * 						  response.
 ***********************************************************************************************************/
int16_t R308_DeleteModel(R308_t * pR308, uint16_t id, uint16_t how_many)
{
    pR308->buffer[0] = R308_DELETE;
    pR308->buffer[1] = id >> 8; pR308->buffer[2] = id & 0xFF;
    pR308->buffer[3] = how_many >> 8; pR308->buffer[4] = how_many & 0xFF;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 5);

    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_EmptyDatabase
 *
 * @brief				- This function empties the database of all registered fingerprints stored in the R308
 * 						  finger print sensor.
 *
 * @parameter[in]		- pointer to R308 structure containing device-specific data
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the database emptying process. It then reads the acknowledgment packet and response
 * 						  packet from the sensor to obtain the confirmation code. A negative return value
 * 						  indicates a failure in reading the response packet from the sensor.
 ***********************************************************************************************************/
int16_t R308_EmptyDatabase(R308_t * pR308)
{
    pR308->buffer[0] = R308_EMPTYDATABASE;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_SearchDatabase
 *
 * @brief				- This function performs a high-speed search of the finger print database stored in
 * 					      the R308 CharBuffer. It searches for a match with the finger print template stored
 * 					      in the designated slot, and returns the ID of the matched finger print and the
 * 					      matching score.
 *
 * @parameter[in]		- pointer to R308 structure containing the finger print database
 * @parameter[in]		- pointer to a variable that will store the ID of the matched finger print
 * @parameter[in]		- pointer to a variable that will store the matching score
 * @parameter[in]		-  designated slot number in the R308 CharBuffer page (R308_BUFFER_SZ)
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the search process. It specifies the slot number of the finger print template to be
 * 						  searched for and the range of the search within the database. It then reads the
 * 						  acknowledgment packet and response packet from the sensor to obtain the confirmation
 * 						  code and the matching finger print ID and score. A negative return value indicates
 * 						  a failure in reading the response packet from the sensor or an unsuccessful search.
 ***********************************************************************************************************/
int16_t R308_SearchDatabase(R308_t * pR308, uint16_t * finger_id, uint16_t * score, uint8_t slot)
{
    // high speed search of slot #1 starting at page 0 to 'capacity'
    pR308->buffer[0] = R308_SEARCH;
    pR308->buffer[1] = slot;
    pR308->buffer[2] = 0x00; pR308->buffer[3] = 0x00;
    pR308->buffer[4] = (uint8_t)(pR308->sys_params.capacity >> 8);
    pR308->buffer[5] = (uint8_t)(pR308->sys_params.capacity & 0xFF);

    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 6);
    uint8_t confirm_code = 0;
    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    if (len != 4)
        return R308_READ_ERROR;

    *finger_id = pR308->buffer[1];
    *finger_id <<= 8;
    *finger_id |= pR308->buffer[2];

    *score = pR308->buffer[3];
    *score <<= 8;
    *score |= pR308->buffer[4];

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_MatchTemplatePair
 *
 * @brief				- This function initiates a finger print matching process between the two templates
 * 					      stored in the R308 CharBuffer 1 and 2, and returns a matching score.
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[out]		- pointer to variable to store the matching score
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  the finger print matching process between CharBuffer 1 and 2. It then reads the
 * 						  acknowledgment packet and response packet from the sensor to obtain the confirmation
 * 						  code and the matching score. If the matching score is successfully obtained, it is
 * 						  stored in the variable passed as a parameter. A negative return value indicates a
 * 						  failure in reading the response packet from the sensor or an error in the matching.
 ***********************************************************************************************************/
int16_t R308_MatchTemplatePair(R308_t * pR308, uint16_t * score)
{
    pR308->buffer[0] = R308_PAIRMATCH;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;

    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    if (len != 2)
        return R308_READ_ERROR;

    *score = pR308->buffer[1];
    *score <<= 8;
    *score |= pR308->buffer[2];

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_GetTemplateCount
 *
 * @brief				- This function sends a command packet to the R308 finger print sensor to retrieve
 * 					      the number of finger print templates stored in the sensor's memory. Upon receiving
 * 					      the response packet from the sensor, the function reads and extracts the template count,
 * 					      and returns a confirmation code indicating the success or failure of the operation.
 *
 * @parameter[in]		- pointer to an R308_t structure containing the configuration and communication details
 * 						  for the sensor
 * @parameter[in]		- pointer to a uint16_t variable that stores the number of templates stored in the
 * 						  sensor's memory.
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to retrieve
 * 						  the template count information. It then reads the acknowledgment packet and response
 * 						  packet from the sensor to obtain the confirmation code and template count.
 * 						  A negative return value indicates a failure in reading the response packet from the
 * 						  sensor, while a confirmation code other than R308_OK indicates a failure in
 * 						  retrieving the template count information.
 ***********************************************************************************************************/
int16_t R308_GetTemplateCount(R308_t * pR308, uint16_t * template_cnt)
{
    pR308->buffer[0] = R308_TEMPLATECOUNT;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;

    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    if (len != 2)
        return R308_READ_ERROR;

    *template_cnt = pR308->buffer[1];
    *template_cnt <<= 8;
    *template_cnt |= pR308->buffer[2];

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_GetFreeIndex
 *
 * @brief				- This function is used to obtain the index of the next available free memory slot
 * 					      in the designated page of the R308 finger print sensor. It reads the template index
 * 					      information from the sensor and searches for the next free slot. The index is
 * 					      returned via a pointer to id.
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- designated page in the R308 template library
 * @parameter[out] 		- pointer to id where the index of the next free slot is stored
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to obtain the
 * 						  template index information from the designated page. It then searches for the next
 * 						  free slot by examining the bits in the template index data. If a free slot is found,
 * 						  the index is returned via the id pointer. If no free slot is available,
 * 						  R308_NOFREEINDEX is returned as the id value.
 ***********************************************************************************************************/
uint16_t R308_GetFreeIndex(R308_t * pR308, uint8_t page, int16_t * id)
{
    pR308->buffer[0] = R308_READTEMPLATEINDEX;
    pR308->buffer[1] = page;

    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 2);
    uint8_t confirm_code = 0;

    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    for (int group_idx = 0; group_idx < len; group_idx++)
    {
        uint8_t group = pR308->buffer[1 + group_idx];
        if (group == 0xff)        /* if group is all occupied */
            continue;

        for (uint8_t bit_mask = 0x01, fid = 0; bit_mask != 0; bit_mask <<= 1, fid++) {
            if ((bit_mask & group) == 0)
            {
                *id = (R308_TEMPLATES_PER_PAGE * page) + (group_idx * 8) + fid;
                return confirm_code;
            }
        }
    }

    *id = R308_NOFREEINDEX;  // no free space found
    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_GetRandomNumber
 *
 * @brief				- This function is used to request a random number from the R308 finger print sensor.
 * 						  It sends a command packet to the sensor requesting a random number, and receives a
 * 						  response packet containing the requested number. The function stores the random
 * 						  number in the memory location specified by the provided pointer to number.
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[out] 		- pointer to the memory location where the random number will be stored
 *
 * @return				- confirmation code or response code indicating success or failure of the operation
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor requesting a
 * 						  random number. Upon receiving the response packet, it checks the confirmation code
 * 						  and the length of the packet to ensure a successful operation. If the operation is
 * 						  successful, the random number is stored in the provided memory location pointed to
 * 						  by the number pointer.
 ***********************************************************************************************************/
int16_t R308_GetRandomNumber(R308_t * pR308, uint32_t * number)
{
    pR308->buffer[0] = R308_GETRANDOM;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;

    int16_t len = read_ack_get_response(pR308, &confirm_code);

    if (len < 0)
        return len;

    if (confirm_code != R308_OK)
        return confirm_code;

    if (len != 4)
        return R308_READ_ERROR;

    *number = pR308->buffer[1];
    *number <<= 8;
    *number |= pR308->buffer[2];
    *number <<= 8;
    *number |= pR308->buffer[3];
    *number <<= 8;
    *number |= pR308->buffer[4];

    return confirm_code;
}

/**********************************************************************************************************
 * @function name 		- R308_Handshake
 *
 * @brief				- This function is used to establish a handshake connection with the R308 finger
 * 						  print sensor. It sends a command packet to the sensor to initiate the handshake,
 * 						  and waits for a response packet containing a confirmation code indicating whether
 * 						  the handshake was successful or not.
 *
 * @parameter[in]		- pointer to R308 structure
 *
 * @return				- confirmation code or response code indicating success or failure of the handshake
 *
 * @Note				- This function sends a command packet to the R308 finger print sensor to initiate
 * 						  a handshake. Upon receiving the response packet, it checks the confirmation code
 * 						  to determine whether the handshake was successful or not. If the confirmation code
 * 						  is R308_HANDSHAKE_OK, the function returns true, indicating a successful handshake.
 * 						  Otherwise, it returns false, indicating a failed handshake.
 ***********************************************************************************************************/
uint8_t R308_Handshake(R308_t * pR308)
{
    pR308->buffer[0] = R308_HANDSHAKE;
    write_packet(pR308, R308_COMMANDPACKET, pR308->buffer, 1);
    uint8_t confirm_code = 0;
    int16_t rc = read_ack_get_response(pR308, &confirm_code);

    if (rc < 0)
        return rc;

    return confirm_code == R308_HANDSHAKE_OK;
}

/**********************************************************************************************************
 * @function name 		- write_packet
 *
 * @brief				- This function is used to write a command packet to the R308 finger print sensor.
 * 						  It constructs the command packet from the provided packet type, packet data, and
 * 						  packet length. The function then calculates a checksum for the packet and writes
 * 						  the packet to the sensor via the write function pointer stored in the provided
 * 						  R308 structure
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- packet type indicating the type of packet being sent (e.g. command, data, etc.)
 * @parameter[in]		- pointer to the packet data being sent
 * @parameter[in]		- length of the packet data being sent
 *
 * @return				- none
 *
 * @Note				- This function constructs the command packet by concatenating the start code,
 * 						  device address, packet type, and packet length with the provided packet data. It
 * 						  then calculates a checksum for the packet by summing the packet length, packet
 * 						  type, and packet data byte by byte. The checksum is then written to the end of
 * 						  the packet, and the entire packet is sent to the R308 finger print sensor via the
 * 						  write function pointer stored in the provided R308 structure.
 ***********************************************************************************************************/
static void write_packet(R308_t * pR308, uint8_t packettype, uint8_t * packet, uint16_t len)
{
    len += 2;

    uint8_t preamble[] = {(uint8_t)(R308_STARTCODE >> 8), (uint8_t)R308_STARTCODE,
                          (uint8_t)(pR308->address >> 24), (uint8_t)(pR308->address >> 16),
                          (uint8_t)(pR308->address >> 8), (uint8_t)(pR308->address),
                          (uint8_t)packettype, (uint8_t)(len >> 8), (uint8_t)(len) };

    pR308->write_func(preamble, sizeof(preamble));

    uint16_t sum = (len >> 8) + (len & 0xFF) + packettype;
    for (uint8_t i = 0; i < len - 2; i++) {
        pR308->write_func(&packet[i], 1);
        sum += packet[i];
    }

    /* assume little-endian mcu */
    pR308->write_func((uint8_t *)(&sum) + 1, 1);
    pR308->write_func((uint8_t *)&sum, 1);
}

/**********************************************************************************************************
 * @function name 		- get_reply
 *
 * @brief				- This function is used to read the response from R308 finger print sensor after a \
 * 						  command is sent to it.
 *
 * @parameter[in]		- pointer to R308 structure containing the configuration of the finger print sensor
 * @parameter[in]		- pointer to a buffer where the response data will be stored
 * @parameter[in]		- length of the buffer pointed to by replyBuf
 * @parameter[in]		- pointer to a variable where the packet ID will be stored
 * @parameter[out]		- function pointer to a UART write function used for outputting data
 * 						  (can be set to NULL if replyBuf is used for output)
 *
 * @return				- If successful, the function returns the length of the response data
 * 						  (excluding the checksum bytes)
 * 						  If a timeout occurs, the function returns R308_TIMEOUT (-1)
 *
 * @Note				- This function uses a state machine to read the response from the R308 finger print
 * 						  sensor after a command is sent to it. It first reads the header of the response
 * 						  packet to ensure that it is valid. It then checks the address in the response
 * 						  packet to make sure that it matches the address of the finger print sensor that
 * 						  sent the response. It reads the packet ID, length, and contents of the response
 * 						  packet, and calculates the checksum to ensure that the packet has been received
 * 						  correctly. If everything is valid, it returns the length of the response data
 * 						  (excluding the checksum bytes). If a timeout occurs while waiting for the response,
 * 						  the function returns R308_TIMEOUT.
 ***********************************************************************************************************/
static int16_t get_reply(R308_t * pR308, uint8_t * replyBuf, uint16_t buflen, uint8_t * pktid, r308_uart_write_func out_stream)
{

	R308_State state = R308_STATE_READ_HEADER;

    uint16_t header = 0;
    uint8_t pid = 0;
    uint16_t length = 0;
    uint16_t chksum = 0;
    uint16_t remn = 0;

    uint32_t last_read = millis_func();

    while ((uint32_t)(millis_func() - last_read) < R308_DEFAULT_TIMEOUT) {
        switch (state) {
            case R308_STATE_READ_HEADER: {
                if (pR308->avail_func() == 0)
                    continue;

                last_read = millis_func();
                uint8_t byte;
                pR308->read_func(&byte, 1);

                header <<= 8; header |= byte;
                if (header != R308_STARTCODE)
                    break;

                state = R308_STATE_READ_ADDRESS;
                header = 0;

                break;
            }
            case R308_STATE_READ_ADDRESS: {
                if (pR308->avail_func() < 4)
                    continue;

                last_read = millis_func();
                pR308->read_func(pR308->buffer, 4);
                uint32_t addr = pR308->buffer[0]; addr <<= 8;
                addr |= pR308->buffer[1]; addr <<= 8;
                addr |= pR308->buffer[2]; addr <<= 8;
                addr |= pR308->buffer[3];

                if (addr != pR308->address) {
                    state = R308_STATE_READ_HEADER;
					ST7735_Print(10, 10, "Wrong address: 0x%.f", addr, Font_11x18, ST7735_GREEN, ST7735_BLACK);
					delay_ms(1000);
                    break;
                }

                state = R308_STATE_READ_PID;

                break;
				}
            case R308_STATE_READ_PID: {
                if (pR308->avail_func() == 0)
                    continue;

                last_read = millis_func();
                pR308->read_func(&pid, 1);
                chksum = pid;
                *pktid = pid;

                state = R308_STATE_READ_LENGTH;

                break;
			}
            case R308_STATE_READ_LENGTH: {
                if (pR308->avail_func() < 2)
                    continue;

                last_read = millis_func();
                pR308->read_func(pR308->buffer, 2);
                length = pR308->buffer[0]; length <<= 8;
                length |= pR308->buffer[1];

                if (length > R308_MAX_PACKET_LEN + 2 || (out_stream == NULL && length > buflen + 2)) {
                    state = R308_STATE_READ_HEADER;
					ST7735_Print(10, 10, "Packet too long: %.f", length, Font_11x18, ST7735_GREEN, ST7735_BLACK);
					delay_ms(1000);
                    continue;
                }

                /* num of bytes left to read */
                remn = length;

                chksum += pR308->buffer[0]; chksum += pR308->buffer[1];
                state = R308_STATE_READ_CONTENTS;
                break;
            }
            case R308_STATE_READ_CONTENTS: {
                if (remn <= 2)
                {
                    state = R308_STATE_READ_CHECKSUM;
                    break;
                }

                if (pR308->avail_func() == 0)
                    continue;

                last_read = millis_func();

                /* we now have to stop using 'fpm->buffer' since
                 * we may be storing data in it now */
                uint8_t byte;
                pR308->read_func(&byte, 1);
                if (out_stream != NULL)
                {
                    out_stream(&byte, 1);
                }
                else {
                    *replyBuf++ = byte;
                }

                chksum += byte;

                remn--;
                break;
            }
            case R308_STATE_READ_CHECKSUM: {
                if (pR308->avail_func() < 2)
                    continue;

                last_read = millis_func();
                uint8_t temp[2];
                pR308->read_func(temp, 2);
                uint16_t to_check = temp[0]; to_check <<= 8;
                to_check |= temp[1];

                if (to_check != chksum) {
                    state = R308_STATE_READ_HEADER;
					ST7735_Print(10, 10, "Wrong cheksum:", to_check, Font_11x18, ST7735_GREEN, ST7735_BLACK);
					delay_ms(1000);
                    continue;
                }

                /* without chksum */
                return length - 2;
            }
        }
    }

	ST7735_FillScreen(ST7735_RED);
	ST7735_DrawString(35, 50, "RESPONSE        TIMEOUT", Font_11x18, ST7735_WHITE, ST7735_RED);
	delay_ms(1000);
    return R308_TIMEOUT;
}

/**********************************************************************************************************
 * @function name 		- read_ack_get_response
 *
 * @brief				- This function reads the standard acknowledgment (ACK) packet from the R308 finger
 * 						  print sensor into the specified buffer, and returns the length of the response packet
 * 						  along with the confirmation code indicating success or failure of the operation.
 *
 * @parameter[in]		- pointer to R308 structure
 * @parameter[in]		- pointer to confirmation code
 *
 * @return				- length of response packet (minus confirmation code) on success, or R308_READ_ERROR
 * 						  indicating failure due to a timeout or wrong packet ID.
 *
 * @Note				- This function reads the response packet from the R308 sensor by calling the get_reply
 * 						  function, which waits for a reply packet from the sensor and returns the length of
 * 						  the packet along with the packet ID. If the packet ID does not match the expected
 * 						  ACK packet ID, R308_READ_ERROR is returned. The confirmation code is extracted from
 * 						  the buffer and stored in the rc pointer. The function returns the length of the
 * 						  response packet (minus the confirmation code) on success.
 ***********************************************************************************************************/
static int16_t read_ack_get_response(R308_t * pR308, uint8_t * rc)
{
    uint8_t pktid = 0;
    int16_t len = get_reply(pR308, pR308->buffer, R308_BUFFER_SZ, &pktid, NULL);

    /* most likely timed out */
    if (len < 0)
        return len;

    /* wrong pkt id */
    if (pktid != R308_ACKPACKET)
    {
		ST7735_Print(10, 10, "Wrong PID: 0x%.f", pktid, Font_11x18, ST7735_GREEN, ST7735_BLACK);
		delay_ms(1000);
        return R308_READ_ERROR;
    }

    *rc = pR308->buffer[0];

    /* minus confirmation code */
    return --len;
}
