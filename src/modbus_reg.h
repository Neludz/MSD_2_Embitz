#ifndef MODBUS_REG_H_INCLUDED
#define MODBUS_REG_H_INCLUDED

#include <stdint.h>
#include "modbus_config.h"
#include "modbus.h"

typedef enum
{
    MB_ERROR 		= 0x00,
    MB_OK 		    = 0x01
} MBError_t;

typedef struct
{
    uint16_t		Default_Value;
    uint16_t		Min_Level;
    uint16_t		Max_Level_Mask;
    uint32_t		Options;
} RegParameters_t;

#if (MB_REG_END_TO_END == 1)
enum
{
#define X_BUF(a,b,c,d,e,f) b,
    MB_BUF_TABLE
#undef  X_BUF
    MB_NUM_BUF  //Reg count
};
#else
enum
{
#define X_BUF(a,b,c,d,e,f,g) b=a,
    MB_BUF_TABLE
#undef  X_BUF
    MB_NUM_BUF=(REG_END_REGISTER+1)
};
#endif

//-----------------------------------------------------------------------
//  prototype
//-----------------------------------------------------------------------

MBError_t mb_instance_idle_check (MBStruct_t *st_mb);
MBError_t mb_reg_limit_check (uint16_t Number_Reg, uint16_t Value);
MBError_t mb_reg_option_check (uint16_t number, uint16_t option_mask);
RegParameters_t mb_getRegParam (uint16_t number);
const void* mb_getRegUserArg1 (uint16_t number);

MBError_t mb_reg_write_option_check (uint16_t number);
MBError_t mb_reg_CB_option_check (uint16_t number);
#endif /* MODBUS_REG_H_INCLUDED*/
