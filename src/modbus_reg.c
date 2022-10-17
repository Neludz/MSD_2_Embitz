/*
 * modbus_reg.c
 */
#include "modbus_config.h"
#include "modbus_reg.h"
#include "modbus.h"

//-----------------------------------------------------------------------
//	X macro
//-----------------------------------------------------------------------

#if ((EEPROM_REG == 1) || (LIMIT_REG == 1))
const t_default_state default_state[NUM_BUF] =
{
#define X_BUF(a,b,c,d,e,f)	[b]={c,d,e,f},
    MAIN_BUF_TABLE
#undef X_BUF
};
#endif

//-----------------------------------------------------------------------
// function
//-----------------------------------------------------------------------

#if (EEPROM_REG == 1)
Reg_Error_Code EESave_Check (uint16_t Number_Reg)
{
    if (default_state[Number_Reg].Permission & EESAVE_R)
    {
        return REG_OK;
    }
    return REG_ERR;
}

//
Reg_Error_Code All_Idle_Check (mb_struct *st_mb)
{
    if ((st_mb->eep_state == EEP_FREE) && (st_mb->mb_state == STATE_IDLE))
    {
        return REG_OK;
    }
    return REG_ERR;
}
#endif

//
#if (LIMIT_REG == 1)
Reg_Error_Code Write_Check (uint16_t Number_Reg)
{
    if (default_state[Number_Reg].Permission & WRITE_R)
    {
        return REG_OK;
    }
    return REG_ERR;
}

//
/*
#define LIM_BIT_MASK	(0x0C)	//check limit mask (2&3 bits)----|
#define LIM_SIGN		(0x04)	// 2 bit for limit     		  <--|
#define LIM_UNSIGN		(0x08)  // 3 bit for limit	    	  <--|
#define LIM_MASK	    (0x0C)	// 2 and 3 bit for limit	  <--|
 */
Reg_Error_Code Limit_Check (uint16_t Number_Reg, uint16_t Value)
{
    switch(default_state[Number_Reg].Permission & LIM_BIT_MASK)
    {
    case 0:
        break;	//not use limit for this register

    case LIM_MASK:
        if (Value & (~(default_state[Number_Reg].Max_Level_Mask)))
        {
            return REG_ERR;
        }
        break;

    case LIM_SIGN:
        if ((int16_t)Value > (int16_t)default_state[Number_Reg].Max_Level_Mask ||
                (int16_t)Value < (int16_t)default_state[Number_Reg].Min_Level)
        {
            return REG_ERR;
        }
        break;

    case LIM_UNSIGN:
        if ((uint16_t)Value > (uint16_t)default_state[Number_Reg].Max_Level_Mask ||
                (uint16_t)Value < (uint16_t)default_state[Number_Reg].Min_Level)
        {
            return REG_ERR;
        }
        break;

    default:
        break;
    }
    return REG_OK;
}
#endif
