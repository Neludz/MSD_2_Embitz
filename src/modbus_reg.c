/*
 * modbus_reg.c
 *
 *  Created on: 3 ���. 2021 �.
 *      Author: Neludz
 */
//======================Include========================================
#include "modbus_reg.h"

//======================Variable========================================
uint16_t MBbuf_main[NUM_BUF]={Version_MSD_2};

//======================X macros========================================

const t_default_state default_state[NUM_BUF] =
{
#define X_BUF(a,b,c,d,e,f)	[b]={c,d,e,f},
MAIN_BUF_TABLE
#undef X_BUF
};

//======================Function========================================

Reg_Error_Code EESave_Check (uint16_t Number_Reg)
{
	if (default_state[Number_Reg].Permission & EESAVE)
	{
	return REG_OK;
	}
return REG_ERR;
}
//--------------------------------------------------------------------------------------

Reg_Error_Code Write_Check (uint16_t Number_Reg)
{
	if (default_state[Number_Reg].Permission & WRITE)
	{
	return REG_OK;
	}
return REG_ERR;
}
//--------------------------------------------------------------------------------------
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
