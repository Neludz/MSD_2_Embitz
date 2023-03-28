#ifndef MODBUS_X_H_INCLUDED
#define MODBUS_X_H_INCLUDED

#include <stdint.h>
#include "main.h"
#include "modbus_hard.h"
#include <measure_NTC.h>
//-----------------------------------------------------------------------
// Modbus registers X macros
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// define
//-----------------------------------------------------------------------
#define MB_LIMIT_REG	    1//check limit
#define MB_CALLBACK_REG	    1//use write callback
//#define MB_USER_ARG1_REG	1//use user argument (for example: run user callback after write function)
//#define MB_USER_ARG2_REG	1//not implement now
// If not define "MB_REG_END_TO_END" then number register is determined in "a" field from X-macros
//#define MB_REG_END_TO_END

#define REG_END_REGISTER                Reg_End

//  MAIN_BUF_Start_Table_Mask
#define USER_FUNC		(0x20)
#define USER_ARG		(0x10)
#define READ_R		    (0)
#define WRITE_R		    (0x01)	// 0 bit                        <--|
#define CB_WR		    (0x02)	// 1 bit                        <--|
#define LIM_SIGN		(0x04)	// 2 bit for limit              <--|
#define LIM_UNSIGN	    (0x08)  // 3 bit for limit	            <--|
#define LIM_MASK	    (0x0C)	// 2 and 3 bit for limit        <--|____________
//                                                                              |
#define LIM_BIT_MASK	        LIM_MASK
//	 Number		Name for enum	       Arg1  Default   Min	   Max     __________Options________
//										      Value   Level   Level   |                         |
//														     or Mask  |                         |
#define MB_BUF_TABLE\
	X_BUF(0,	Reg_Start,		        0,		0,		0,      0,      READ_R)\
	X_BUF(1,	Reg_T_0_Channel,	    0,		0,		0, 		0,	 	READ_R)\
	X_BUF(2,	Reg_T_1_Channel,	    0,		0,		0, 		0,	 	READ_R)\
	X_BUF(3,	Reg_T_2_Channel,	    0,		0,		0, 		0,	 	READ_R)\
	X_BUF(4,	Reg_T_3_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(5,	Reg_T_4_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(6,	Reg_T_5_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(7,	Reg_T_6_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(8,	Reg_T_7_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(9,	Reg_T_8_Channel,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(10,	Reg_Cur_RMS_W1,		    0,		0,		0, 		0,		READ_R)\
	X_BUF(11,	Reg_Cur_RMS_W2,		    0,		0,		0, 		0,		READ_R)\
	X_BUF(12,	Reg_T_MSD,			    0,		0,		0, 		0,		READ_R)\
	X_BUF(13,	Reg_T_Max,			    0,		0,		0, 		0,		READ_R)\
	X_BUF(14,	Reserved_3,			    0,      100,	0, 		0xFFFF,	READ_R)\
	X_BUF(15,	Reserved_4,			    0,		123,	0, 		0xFFFF,	READ_R)\
	X_BUF(16,	Reg_16,                 0,		0,		0, 		0,	 	READ_R)\
	X_BUF(17,	Reg_17,	                0,      0,		0, 		0,	 	READ_R)\
	X_BUF(18,	Reg_Status_DI_Bit,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(19,	Reg_Status_DO_Bit,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(20,	Reg_T_Warning_bit,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(21,	Reg_T_Alarm_bit,	    0,		0,		0, 		0,		READ_R)\
	X_BUF(22,	Reg_Cur_N_Measure_W1,	0,		0,		0, 		0,		READ_R)\
	X_BUF(23,	Reg_Cur_N_Measure_W2,	0,		0,		0, 		0,		READ_R)\
	X_BUF(24,	Reg_Cur_Cross_Count,	0,		0,		0, 		0,		READ_R)\
	X_BUF(25,	MB_ERRORor_Count,		0,		0,		0, 		0,		READ_R)\
	X_BUF(26,	Reg_DO_On_Bit,			0,		0,		0, 		0x03,	WRITE_R | 		  LIM_MASK)\
	X_BUF(27,	Reg_DO_Off_Bit,			0,		0,		0, 		0x03,	WRITE_R | 		  LIM_MASK)\
	X_BUF(28,	Reg_Set_Default_Reset,	0,		0,		0, 		0,	 	WRITE_R)\
	X_BUF(29,	Reserved_T,				0,		0,		0, 		0,		READ_R)\
	X_BUF(30,	Reg_Mode_Cur,			0,		0,		0,		0x1,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(31,	Reg_Mode_DO,			0,		0,		0,		0x6,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(32,	Reserv32,               0,		0,		0,		0x1,	READ_R)\
	X_BUF(33,	Reg_DO_1_Delay,			0,		100,	1,		0xFF,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(35,	Reg_T_level_Warning,	0,		TEMPERATURE_OVER,\
                                                        0,		0,	    WRITE_R | CB_WR)\
	X_BUF(36,	Reg_T_level_Alarm,		0,		TEMPERATURE_OVER,\
                                                        0,		0,	    WRITE_R | CB_WR)\
	X_BUF(37,	Reg_T_Number_Sensor_1,	0,		0x1FF,	0,		0x1FF,	WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(38,	Reg_T_Number_Sensor_2,	0,		0x1FF,	0,		0x1FF,	WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(39,	Reserved_5,				0,		0,		0,		0,	    READ_R)\
	X_BUF(40,	Reg_Cur_Sens_Hall_Ratio,0,		132,	5,		1000,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(41,	Reg_Cur_Filter_Ratio,	0,		30,		1,		0xFF,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(42,	Reg_Cur_Time_Measure,	0,		1200,	100,	2000,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(43,	Reg_Cur_Scale,			0,		1,		1,		1000,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(44,	Reg_Cur_Dot,			0,		0,		0,		3,		WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(45,	Reg_Cur_Zero_Level,		0,		200,	0,		0,	    WRITE_R | CB_WR)\
	X_BUF(46,	Reg_Cur_Lev_Warn_W1,    0,		40,		0,		0,	    WRITE_R | CB_WR)\
	X_BUF(47,	Reg_Cur_Lev_Warn_W2,    0,		0,		0,		0,	    WRITE_R | CB_WR)\
	X_BUF(48,	Reg_Cur_Level_Alarm_W1,	0,		45,		0,		0,	    WRITE_R | CB_WR)\
	X_BUF(49,	Reg_Cur_Level_Alarm_W2,	0,		0,		0,		0,	    WRITE_R | CB_WR)\
	X_BUF(50,	Reg_RS485_Baud_Rate,	0,		1,		0,		0x03,	WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(51,	Reg_RS485_Ans_Delay,	0,	    5,		0,      100,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(52,	Reg_RS485_Modbus_Addr,  0,		1,		1,		0xFA,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(53,	Reg_Parity_Stop_Bits,	0,	    0,	    0,		0x03,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(56,	Reg_NTC_R2_Value_W1,    0,	    5000,	0,		0,      WRITE_R | CB_WR)\
	X_BUF(57,	Reg_NTC_R2_Value_W2,    0,	    0,		0,		0,		WRITE_R | CB_WR)\
	X_BUF(58,	Reg_NTC_R_Divider_W1,   0,	    5100,	0,		0,      WRITE_R | CB_WR)\
	X_BUF(59,	Reg_NTC_R_Divider_W2,   0,	    0,		0,		0,      WRITE_R | CB_WR)\
	X_BUF(60,	Reg_NTC_B_Value,        0,	    3984,	0,		0,      WRITE_R | CB_WR)\
	X_BUF(61,	Reg_NTC_T2_Value,       0,	    25,		0,		0,      WRITE_R | CB_WR)\
	X_BUF(62,	Reg_NTC_Start_Temper,   0,	    -10,	-40,	20,     WRITE_R | CB_WR | LIM_SIGN)\
	X_BUF(63,	Reg_NTC_Step_Temper,    0,	    1,		1,		5,      WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(64,	Reg_NTC_Temper_N_Step,  0,	    136,    1,      SIZE_NTC_TABLE,\
                                                                        WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(65,	Reg_End,				0,	   0,      0,      0,       READ_R)\


#endif /* MODBUS_X_H_INCLUDED */
