#ifndef MODBUS_H_INCLUDED
#define MODBUS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "modbus_config.h"

#ifndef MB_LIMIT_REG
#define MB_LIMIT_REG            0
#endif

#ifndef MB_CALLBACK_REG
#define MB_CALLBACK_REG         0
#endif

#ifndef MB_REG_END_TO_END
#define MB_REG_END_TO_END       0
#endif

#ifndef MB_USER_ARG1_REG
#define MB_USER_ARG1_REG        0
#endif

#ifndef MB_USER_ARG2_REG
#define MB_USER_ARG2_REG        0
#endif

#define MB_FRAME_MIN     		4       /* Minimal size of a Modbus RTU frame	*/
#define MB_FRAME_MAX     		256     /* Maximal size of a Modbus RTU frame	*/
#define MB_ADDRESS_BROADCAST  	00		/* MBBuff[0] analysis					*/

#define MB_ANY_ADDRESS		  	00		/* 0 - any address						*/
#define MB_MAX_REG				120		/*max quantity registers in inquiry. Should be less than MB_FRAME_MAX considering service bytes. Use for 03 function*/

#define MB_FUNC_NONE							00
#define MB_FUNC_READ_COILS						01
#define MB_FUNC_READ_DISCRETE_INPUTS			02
#define MB_FUNC_WRITE_SINGLE_COIL				05
#define MB_FUNC_WRITE_MULTIPLE_COILS			15
#define MB_FUNC_READ_HOLDING_REGISTER			03	/* implemented now	*/
#define MB_FUNC_READ_INPUT_REGISTER				04
#define MB_FUNC_WRITE_REGISTER					06	/* implemented now	*/
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS		16	/* implemented now	*/
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS	23
#define MB_FUNC_ERROR							0x80

typedef enum  			// Actually only 1 variable uses this type: er_frame_bad
{
    EV_NOEVENT,
    EV_HAPPEND
} MBEvents_t;

typedef enum
{
    MBE_NONE 					= 0x00,
    MBE_ILLEGAL_FUNCTION 		= 0x01,
    MBE_ILLEGAL_DATA_ADDRESS	= 0x02,
    MBE_ILLEGAL_DATA_VALUE		= 0x03
} MBExcep_t;

typedef enum
{
    MB_STATE_IDLE,			// Ready to get a frame from Master
    MB_STATE_RCVE,			// Frame is being received
    MB_STATE_WAIT,	        // Frame is wait to parse
    MB_STATE_PARS,			// Frame is being parsed (may take some time)
    MB_STATE_SEND,			// Response frame is being sent
    MB_STATE_SENT			// Last byte sent to shift register. Waiting "Last Bit Sent" interrupt
} MBState_t;

typedef enum
{
    MB_CB_FREE = 0,
    MB_CB_PRESENT = 1
} CBState_t;

typedef struct  					// Main program passes interface data to Modbus stack.
{
    uint16_t    *p_write;			// Pointer to the begin of ParsIn array. Modbus writes data in the array
    uint16_t    *p_read;			// Pointer to the begin of ParsWk array. Modbus takes data from the array
    uint16_t    reg_read_last;		//
    uint16_t    reg_write_last;		//
#if (MB_CALLBACK_REG == 1)
    uint16_t    cb_reg_start;		// function modbus write here data start index
    uint8_t	    cb_index;			// and how many registers are pending validation
    CBState_t   cb_state;			//
#endif
    uint8_t     slave_address;		//
    uint8_t	    mb_index;
    MBState_t   mb_state;
    MBEvents_t	er_frame_bad;		//
    uint8_t     *p_mb_buff;			//
    uint8_t		response_size;		                // Set in frame_parse(), used in transmit
#if (MB_CALLBACK_REG == 1)
    void   		(*wr_callback) ( void *mbb);    //for span of register, use "mb_reg_option_check" in this function for every register
#endif
    void    	(*f_start_trans) ( void *mbb);      //start transmit
    void    	(*f_start_receive) ( void *mbb);    //only if you stop the exchange during parsing, it can be NULL,
                                                    //then when using one buffer, check the "mb_state"
                                                    //so as not to change buffer while parsing it
} MBStruct_t;

void mb_parsing(MBStruct_t *mbb);

#endif /* MODBUS_H_INCLUDED */
