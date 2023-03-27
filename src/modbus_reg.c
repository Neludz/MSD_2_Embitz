/*
 * modbus_reg.c
 */
#include "modbus_config.h"
#include "modbus_reg.h"
#include "modbus.h"

//-----------------------------------------------------------------------
//	X macro
//-----------------------------------------------------------------------

#if ((MB_CALLBACK_REG == 1) || (MB_LIMIT_REG == 1))
const RegParameters_t MBRegParam[MB_NUM_BUF] =
{
#define X_BUF(a,b,c,d,e,f,g)	[b]={d,e,f,g},
    MB_BUF_TABLE
#undef X_BUF
};
#endif

#if (MB_USER_ARG1_REG == 1)
const void *MBRegArg1[MB_NUM_BUF] =
{
#define X_BUF(a,b,c,d,e,f,g)	[b]=c,
    MB_BUF_TABLE
#undef X_BUF
};
#endif

//-----------------------------------------------------------------------
// function
//-----------------------------------------------------------------------
#if ((MB_CALLBACK_REG == 1) || (MB_LIMIT_REG == 1))
MBError_t mb_instance_idle_check (MBStruct_t *st_mb)
{
    if ((st_mb->cb_state == MB_CB_FREE) && (st_mb->mb_state == MB_STATE_IDLE))
    {
        return MB_OK;
    }
    return MB_ERROR;
}

MBError_t mb_reg_limit_check (uint16_t number, uint16_t value)
{
    switch(MBRegParam[number].Options & LIM_BIT_MASK)
    {
    case 0:
        break;	//not use limit for this register

    case LIM_MASK:
        if (value & (~(MBRegParam[number].Max_Level_Mask)))
        {
            return MB_ERROR;
        }
        break;

    case LIM_SIGN:
        if ((int16_t)value > (int16_t)MBRegParam[number].Max_Level_Mask ||
                (int16_t)value < (int16_t)MBRegParam[number].Min_Level)
        {
            return MB_ERROR;
        }
        break;

    case LIM_UNSIGN:
        if ((uint16_t)value > (uint16_t)MBRegParam[number].Max_Level_Mask ||
                (uint16_t)value < (uint16_t)MBRegParam[number].Min_Level)
        {
            return MB_ERROR;
        }
        break;

    default:
        break;
    }
    return MB_OK;
}

MBError_t mb_reg_option_check (uint16_t number, uint16_t option_mask)
{
    if (MBRegParam[number].Options & option_mask)
    {
        return MB_OK;
    }
    return MB_ERROR;
}

RegParameters_t mb_getRegParam (uint16_t number)
{
    return MBRegParam[number];
}
#endif

#if (MB_USER_ARG1_REG == 1)
const void* mb_getRegUserArg1 (uint16_t number)
{
    return MBRegArg1[number];
}
#endif

// function need for modbus.c
#if (MB_CALLBACK_REG == 1)
CBState_t cb_check_in_request (uint16_t Start_Reg, uint16_t Count)
{
    if((Count+Start_Reg)>=MB_NUM_BUF)
    {
        return 	MB_CB_FREE;
    }
    for (int32_t i = 0; i < Count; i++)
    {
        if(mb_reg_option_check(Start_Reg+i, CB_WR)==MB_OK)
        {
            return MB_CB_PRESENT;
        }
    }
    return 	MB_CB_FREE;
}
#endif

// function need for modbus.c
// check register limits
#if (MB_LIMIT_REG == 1)
MBExcep_t mb_reg_limit_check_in_request (uint16_t Number_Reg, uint16_t Value)
{
    if (mb_reg_option_check(Number_Reg, WRITE_R)==MB_ERROR)
    {
        return MBE_ILLEGAL_DATA_ADDRESS;
    }

    if (mb_reg_limit_check (Number_Reg, Value)==MB_ERROR)
    {
        return	MBE_ILLEGAL_DATA_VALUE;
    }

    return MBE_NONE;
}
#endif
