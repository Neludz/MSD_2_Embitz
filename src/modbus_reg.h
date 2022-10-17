#ifndef MODBUS_REG_H_INCLUDED
#define MODBUS_REG_H_INCLUDED

#include <stdint.h>
#include "modbus_config.h"
#include "modbus.h"

typedef enum
{
    REG_OK 		    = 0x00,
    REG_ERR 		= 0x01
} Reg_Error_Code;

typedef struct
{
    //X_BUF(a,b,c,d,e,f)
    uint16_t		Default_Value;		//c
    uint16_t		Min_Level;		    //d
    uint16_t		Max_Level_Mask;		//e
    uint32_t		Permission;		    //f
} t_default_state;

#if (MODBUS_REG_END_TO_END == 1)
enum
{
#define X_BUF(a,b,c,d,e,f) b,
    MAIN_BUF_TABLE
#undef  X_BUF
    NUM_BUF  //Reg count
};
#else
enum
{
#define X_BUF(a,b,c,d,e,f) b=a,
    MAIN_BUF_TABLE
#undef  X_BUF
    NUM_BUF=(REG_END_REGISTER+1)
};
#endif

//-----------------------------------------------------------------------
//  prototype
//-----------------------------------------------------------------------

Reg_Error_Code EESave_Check (uint16_t Number_Reg);
Reg_Error_Code All_Idle_Check (mb_struct *st_mb);
Reg_Error_Code Write_Check (uint16_t Number_Reg);
Reg_Error_Code Limit_Check (uint16_t Number_Reg, uint16_t Value);

#endif /* MODBUS_REG_H_INCLUDED*/
