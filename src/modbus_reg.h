#ifndef MODBUS_REG_H__
#define MODBUS_REG_H__
//======================Include========================================
#include <stdint.h>
#include <stdbool.h>
#include <main.h>

//======================Define========================================
#define LIMIT_REG	//check limit
#define EEPROM_REG	//use eeprom

//=========================   Modbus registers X macros   =========================

// If not define "MODBUS_REG_END_TO_END" then number register is determined in "a" field from X-macros

//#define MODBUS_REG_END_TO_END

#ifndef MODBUS_REG_END_TO_END
#define REG_COUNT                Reg_Version+1
#endif

typedef enum {
    REG_OK 			= 0x00,
    REG_ERR 		= 0x01
} Reg_Error_Code;


#define LIM_BIT_MASK	(0x0C)	//check limit mask (2&3 bits)

//  MAIN_BUF_Start_Table_Mask
#define READ			(0)
#define WRITE			(0x01)	// 0 bit
#define EESAVE			(0x02)	// 1 bit
#define LIM_SIGN		(0x04)	// 2 bit for limit     		  <--|
#define LIM_UNSIGN		(0x08)  // 3 bit for limit	    	  <--|----------------
#define LIM_MASK	    (0x0C)	// 2 and 3 bit for limit	  <--|				  |
//																				  |
//	 Number		Name for enum			Default	   Min	    Max   	__________Permission_______
//										 Value    Level    Level   |  R/W     EEPROM    LIMIT  |
//														  or Mask  | 						   |
#define MAIN_BUF_TABLE\
	X_BUF(0,	Reg_Start,					0,		0, 		0,		 READ)\
	X_BUF(1,	Reg_T_0_Channel,			0,		0, 		0,	 	 READ)\
	X_BUF(2,	Reg_T_1_Channel,			0,		0, 		0,	 	 READ)\
	X_BUF(3,	Reg_T_2_Channel,			0,		0, 		0,	 	 READ)\
	X_BUF(4,	Reg_T_3_Channel,			0,		0, 		0,		 READ)\
	X_BUF(5,	Reg_T_4_Channel,			0,		0, 		0,		 READ)\
	X_BUF(6,	Reg_T_5_Channel,			0,		0, 		0,		 READ)\
	X_BUF(7,	Reg_T_6_Channel,			0,		0, 		0,		 READ)\
	X_BUF(8,	Reg_T_7_Channel,			0,		0, 		0,		 READ)\
	X_BUF(9,	Reg_T_8_Channel,			0,		0, 		0,		 READ)\
	X_BUF(10,	Reg_Cur_RMS_W1,				0,		0, 		0,		 READ)\
	X_BUF(11,	Reg_Cur_RMS_W2,				0,		0, 		0,		 READ)\
	X_BUF(12,	Reg_T_MSD,					0,		0, 		0,		 READ)\
	X_BUF(13,	Reg_T_Max,					0,		0, 		0,		 READ)\
	X_BUF(14,	Reserved_3,					789,	0, 		0xFFFF,	 READ)\
	X_BUF(15,	Reserved_4,					123,	0, 		0xFFFF,	 READ)\
	X_BUF(16,	Reg_DI_1_Trip_Counter,		0,		0, 		0,	 	 WRITE | EESAVE)\
	X_BUF(17,	Reg_DI_2_Trip_Counter,		0,		0, 		0,	 	 WRITE | EESAVE)\
	X_BUF(18,	Reg_Status_DI_Bit,			0,		0, 		0,		 READ)\
	X_BUF(19,	Reg_Status_DO_Bit,			0,		0, 		0,		 READ)\
	X_BUF(20,	Reg_T_Warning_bit,			0,		0, 		0,		 READ)\
	X_BUF(21,	Reg_T_Alarm_bit,			0,		0, 		0,		 READ)\
	X_BUF(22,	Reg_Cur_N_Measure_W1,		0,		0, 		0,		 READ)\
	X_BUF(23,	Reg_Cur_N_Measure_W2,		0,		0, 		0,		 READ)\
	X_BUF(24,	Reg_Cur_Cross_Count,		0,		0, 		0,		 READ)\
	X_BUF(25,	Reg_Error_Count,			0,		0, 		0,		 READ)\
	X_BUF(26,	Reg_DO_On_Bit,				0,		0, 		0x03,	 WRITE | 		  LIM_MASK)\
	X_BUF(27,	Reg_DO_Off_Bit,				0,		0, 		0x03,	 WRITE | 		  LIM_MASK)\
	X_BUF(28,	Reg_Set_Default_Reset,		0,		0, 		0,	 	 WRITE)\
	X_BUF(29,	Reserved_T,					0,		0, 		0,		 READ)\
	X_BUF(30,	Reg_Mode_Cur,				0,		0,		0x1,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(31,	Reg_Mode_DO,				0,		0,		0x6,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(32,	Reg_Mode_DI_Trip_Counter,	0,		0,		0x1,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(33,	Reg_DO_1_Delay,				100,	1,		0xFF,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(34,	Reg_DO_2_Delay,				100,	1,		0xFF,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(35,	Reg_T_level_Warning,		TEMPERATURE_OVER,\
                                                    0,		0,	     WRITE | EESAVE)\
	X_BUF(36,	Reg_T_level_Alarm,			TEMPERATURE_OVER,\
                                                    0,		0,	     WRITE | EESAVE)\
	X_BUF(37,	Reg_T_Number_Sensor_1,		0x1FF,	0,		0x1FF,	 WRITE | EESAVE | LIM_MASK)\
	X_BUF(38,	Reg_T_Number_Sensor_2,		0x1FF,	0,		0x1FF,	 WRITE | EESAVE | LIM_MASK)\
	X_BUF(39,	Reserved_5,					0,		0,		0,	     READ)\
	X_BUF(40,	Reg_Cur_Sensor_Hall_Ratio,	132,	5,		1000,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(41,	Reg_Cur_Filter_Ratio,		30,		1,		0xFF,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(42,	Reg_Cur_Time_Measure,		1200,	100,	2000,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(43,	Reg_Cur_Scale,				1,		1,		1000,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(44,	Reg_Cur_Dot,				0,		0,		3,		 WRITE | EESAVE | LIM_MASK)\
	X_BUF(45,	Reg_Cur_Zero_Level,			200,	0,		0,	     WRITE | EESAVE)\
	X_BUF(46,	Reg_Cur_Level_Warning_W1,	40,		0,		0,	     WRITE | EESAVE)\
	X_BUF(47,	Reg_Cur_Level_Warning_W2,	0,		0,		0,	     WRITE | EESAVE)\
	X_BUF(48,	Reg_Cur_Level_Alarm_W1,		45,		0,		0,	     WRITE | EESAVE)\
	X_BUF(49,	Reg_Cur_Level_Alarm_W2,		0,		0,		0,	     WRITE | EESAVE)\
	X_BUF(50,	Reg_RS485_Baud_Rate,		1,		0,		0x03,	 WRITE | EESAVE | LIM_MASK)\
	X_BUF(51,	Reg_RS485_Ans_Delay,	    5,		MIN_TIME_TO_START_TRANSMIT_MS,\
                                                            100,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(52,	Reg_RS485_Modbus_Address,	1,		1,		0xFA,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(53,	Reg_Parity_Stop_Bits,	    0,	    0,		0x03,	 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(56,	Reg_NTC_R2_Value_W1,        5000,	0,		0,       WRITE | EESAVE)\
	X_BUF(57,	Reg_NTC_R2_Value_W2,        0,		0,		0,		 WRITE | EESAVE)\
	X_BUF(58,	Reg_NTC_R_Divider_W1,       5100,	0,		0,       WRITE | EESAVE)\
	X_BUF(59,	Reg_NTC_R_Divider_W2,       0,		0,		0,       WRITE | EESAVE)\
	X_BUF(60,	Reg_NTC_B_Value,            3984,	0,		0,       WRITE | EESAVE)\
	X_BUF(61,	Reg_NTC_T2_Value,           25,		0,		0,       WRITE | EESAVE)\
	X_BUF(62,	Reg_NTC_Start_Temperature, -10,	   -40,		20,		 WRITE | EESAVE | LIM_SIGN)\
	X_BUF(63,	Reg_NTC_Step_Temperature,   1,		1,		5,       WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(64,	Reg_NTC_Temper_Number_Step, 136,    1,      SIZE_NTC_TABLE,\
	                                                                 WRITE | EESAVE | LIM_UNSIGN)\
	X_BUF(65,	Reg_Version,				Version_MSD_2,\
													Version_MSD_2,\
															Version_MSD_2,\
																	 READ  | EESAVE)\

//------========MAIN_BUF_End_Table========------

typedef struct
{						//X_BUF(a,b,c,d,e,f)
uint16_t		Default_Value;		//c
uint16_t		Min_Level;			//d
uint16_t		Max_Level_Mask;		//e
uint32_t		Permission;			//f
} t_default_state;

#ifdef MODBUS_REG_END_TO_END
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
          NUM_BUF=REG_COUNT
};
#endif

 //=========================   prototype   =========================

 Reg_Error_Code EESave_Check (uint16_t Number_Reg);
 Reg_Error_Code Write_Check (uint16_t Number_Reg);
 Reg_Error_Code Limit_Check (uint16_t Number_Reg, uint16_t Value);

#endif /* MODBUS_REG_H__ */
