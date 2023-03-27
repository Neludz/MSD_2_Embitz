#include "modbus.h"
#include "modbus_config.h"
#include <stdio.h>

#if ((MB_CALLBACK_REG == 1) || (MB_LIMIT_REG == 1))
#include "modbus_reg.h"
#endif
//-----------------------------------------------------------------------
//  CRC
//----------------------------------------------------------------------

/* Table of CRC values for high–order byte */
static const uint16_t wCRCTable[] =
{
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
} ;

static unsigned int mb_CRC16 ( unsigned char *puchMsg, unsigned int usDataLen )
{
    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (usDataLen--)
    {
        nTemp = *puchMsg++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}

//-----------------------------------------------------------------------
//  function
//----------------------------------------------------------------------

#if (MB_CALLBACK_REG == 1)
static CBState_t mb_cb_check_in_request (uint16_t Start_Reg, uint16_t Count)
{
    for (int32_t i = 0; i < Count; i++)
    {
        if(mb_reg_CB_option_check(Start_Reg+i)==MB_OK)
        {
            return MB_CB_PRESENT;
        }
    }
    return 	MB_CB_FREE;
}
#endif

// check register limits
#if (MB_LIMIT_REG == 1)
static MBExcep_t mb_reg_limit_check_in_request (uint16_t Number_Reg, uint16_t Value)
{
    if (mb_reg_write_option_check(Number_Reg)==MB_ERROR)
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

static bool invalid_frame( MBStruct_t *mbb)
{
    uint8_t	PDU_len;
    if( EV_HAPPEND == mbb->er_frame_bad)
    {
        return true;
    }
    if( mbb->mb_index < MB_FRAME_MIN)
    {
        return true;
    }
    if((mbb->slave_address != mbb->p_mb_buff[0])&(mbb->p_mb_buff[0]!=MB_ADDRESS_BROADCAST)&(mbb->slave_address!=0))
    {
        return true;
    }
#if (MB_CALLBACK_REG == 1)
    if(mbb->cb_state!=MB_CB_FREE)
    {
        return true;
    }
#endif
    PDU_len = mbb->mb_index;
    if( mb_CRC16( (uint8_t*)mbb->p_mb_buff, PDU_len))
    {
        return true;
    }
    return false;
}


static bool frame_parse (MBStruct_t *mbb)
{
    // Returns TRUE if a response is needed
#if (MB_LIMIT_REG == 1)
    uint16_t 	Value, RegIndxInter, j;
#endif
    uint16_t 	RegIndx, RegNmb, RegLast,   i;
    uint8_t		BytesN;
    MBExcep_t	Exception;					// If a Modbus exception happens - we put the var in MBBuff[2]

    bool NeedResponse = true;
    if( mbb->p_mb_buff[0] == MB_ADDRESS_BROADCAST)
    {
        NeedResponse = false;				// We parse the request but we don'n give a response
    }

    switch( mbb->p_mb_buff[1])
    {
    // Function code
//--------------------------------------------------
    case MB_FUNC_READ_HOLDING_REGISTER:
        //		  03	0...LASTINDEX-1      0...125
        // addr  func    AddrHi  AddrLo  QuantHi  QuantLo  CRC CRC
        //  0	  1			2		3		4		5		 6	7
        if( 8 == mbb->mb_index)
        {
            // In this function mb_index == 8.
            RegIndx = (mbb->p_mb_buff[2]<<8) | (mbb->p_mb_buff[3]&0xFF);
            RegNmb  = (mbb->p_mb_buff[4]<<8) | (mbb->p_mb_buff[5]&0xFF);
            RegLast = RegIndx + RegNmb;
            if( (RegIndx > mbb->reg_read_last) ||
                    ((RegLast-1) > mbb->reg_read_last) ||
                    (RegNmb>MB_MAX_REG) )	// max quantity registers in inquiry
            {
                Exception = MBE_ILLEGAL_DATA_ADDRESS;
                break;
            }
            // Make response. MBBuff[0] and MBBuff[1] are ready
            mbb->p_mb_buff[2] = RegNmb << 1;
            mbb->mb_index = 3;
            while( RegIndx < RegLast )
            {
                mbb->p_mb_buff[mbb->mb_index++] = (uint8_t)(*(mbb->p_read+RegIndx) >> 8)  ;
                mbb->p_mb_buff[mbb->mb_index++] = (uint8_t)(*(mbb->p_read+RegIndx) & 0xFF);
                ++RegIndx;
            }
            Exception = MBE_NONE;	// OK, make CRC to MBBuff[mb_index] and response is ready
            break;
        }
        Exception = MBE_ILLEGAL_DATA_VALUE;	// PDU length incorrect
        break;
    //--------------------------------------------------
    case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
        //		  16	0...LASTINDEX-1      0...125	 0...250
        // addr  func    AddrHi  AddrLo  QuantHi  QuantLo  Bytes  RG1Hi RG1LO ... CRC CRC
        //  0	  1			2		3		4		5		 6		7	  8	  ...

        if( mbb->mb_index > 10)
        {
            Exception = MBE_NONE;	// ...and mb_index is a length of response
            // The must: mb_index == 11, 13, 15, ...
            RegIndx = (mbb->p_mb_buff[2]<<8) | (mbb->p_mb_buff[3]&0xFF);
            RegNmb  = (mbb->p_mb_buff[4]<<8) | (mbb->p_mb_buff[5]&0xFF);
            RegLast = RegIndx + RegNmb;
            BytesN	= mbb->p_mb_buff[6];
            if(	(RegIndx > mbb->reg_write_last) ||
                    ((RegLast-1) > mbb->reg_write_last) ||
                    (RegNmb>MB_MAX_REG))
            {
                Exception = MBE_ILLEGAL_DATA_ADDRESS;
                break;
            }
            if( BytesN != (RegNmb << 1) ||
                    mbb->mb_index != (9+BytesN) )
            {
                // 1 reg - mb_index=11, 2 regs - 13,... 5 regs - 19, etc.
                Exception = MBE_ILLEGAL_DATA_VALUE;
                break;
            }

            i = 7;	// Registers' values are from MBBuff[7] and more
#if (MB_CALLBACK_REG == 1)
            //=================check EEPROM start==============================
            mbb->cb_state = mb_cb_check_in_request(RegIndx, RegNmb);
            if(mbb->cb_state==MB_CB_PRESENT)
            {
                mbb->cb_reg_start = RegIndx;
                mbb->cb_index = RegNmb;
            }
            //=================check EEPROM end==============================
#endif

#if (MB_LIMIT_REG == 1)
            //=================check data start==============================
            j = i;
            for (RegIndxInter = RegIndx; RegIndxInter < RegLast; RegIndxInter++)
            {
                Value = mbb->p_mb_buff[j++]<<8;
                Value |= mbb->p_mb_buff[j++];
                Exception = mb_reg_limit_check_in_request(RegIndxInter, Value);
                if	(Exception != MBE_NONE)
                {
                    break;
                }
            }
            if(Exception != MBE_NONE)
            {
                break;
            }
            //=================check data end==============================
#endif
            while( RegIndx < RegLast)
            {
                *(mbb->p_write+RegIndx)  = mbb->p_mb_buff[i++]<<8;	// High, then Low byte
                *(mbb->p_write+RegIndx) |= mbb->p_mb_buff[i++]	;	// ... are packed in a WORD
                ++RegIndx;
            }
            mbb->mb_index = 6;	// MBBuff[0] to MBBuff[5] are ready (unchanged)
            break;
        }
        Exception = MBE_ILLEGAL_DATA_VALUE;	// PDU length incorrect
        break;

    //--------------------------------------------------
    case MB_FUNC_WRITE_REGISTER:
        //		  06		0...250     	 0...255
        // addr  func    AddrHi  AddrLo   RG1Hi  	RG1LO   CRC		 CRC
        //  0	  1			2		3		4		  5	     6	 	  7

        if( mbb->mb_index == 8)
        {
            Exception = MBE_NONE;	// ...and mb_index is a length of response
            RegIndx = (mbb->p_mb_buff[2]<<8) | (mbb->p_mb_buff[3]&0xFF);

            if((RegIndx > mbb->reg_write_last))
            {
                Exception = MBE_ILLEGAL_DATA_ADDRESS;
                break;
            }

            i = 4;	// Registers' value are from MBBuff[4]-[5]
#if (MB_CALLBACK_REG == 1)
            //=================check EEPROM start==============================
            mbb->cb_state = mb_cb_check_in_request(RegIndx, 1);
            if(mbb->cb_state==MB_CB_PRESENT)
            {
                mbb->cb_reg_start = RegIndx;
                mbb->cb_index = 1;
            }
            //=================check EEPROM end==============================
#endif

#if (MB_LIMIT_REG  == 1)
            //=================check data start==============================
            RegIndxInter = RegIndx;
            j = i;

            Value = mbb->p_mb_buff[j++]<<8;
            Value |= mbb->p_mb_buff[j++];
            Exception = mb_reg_limit_check_in_request(RegIndxInter, Value);
            if	(Exception != MBE_NONE)
            {
                //mbb->cb_state = MB_CB_FREE;
                break;
            }
            //=================check data end==============================
#endif
            *(mbb->p_write+RegIndx)  = mbb->p_mb_buff[i++]<<8;	// High, then Low byte
            *(mbb->p_write+RegIndx) |= mbb->p_mb_buff[i++]	;	// ... are packed in a WORD

            mbb->mb_index = 6;	// MBBuff[0] to MBBuff[5] are ready (unchanged)

            break;
        }
        Exception = MBE_ILLEGAL_DATA_VALUE;	// PDU length incorrect
        break;
    //--------------------------------------------------
    default:
        Exception = MBE_ILLEGAL_FUNCTION;
        break;
    }

    /*
     * At this point the "mb_index" is a length of the response (w/o CRC bytes)
     * mb_index is variable if MB_FUNC_READ_HOLDING_REGISTER, 6 if MB_FUNC_WRITE_MULTIPLE_REGISTERS)
     * but if there's some exception - it will be shortened to 3 in both cases
     */

    if( Exception != MBE_NONE)
    {
        // Any exception?
        mbb->p_mb_buff[1] |= MB_FUNC_ERROR;						// Add 0x80 to function code
        mbb->p_mb_buff[2] =  Exception;							// Exception code
        mbb->mb_index = 3;										// Length of response is fixed if exception
#if (MB_CALLBACK_REG == 1)
        mbb->cb_state = MB_CB_FREE;
#endif
    }
    i = mb_CRC16( (uint8_t*)mbb->p_mb_buff, mbb->mb_index);				// MBBuff is a pointer, mb_index is a size
    mbb->p_mb_buff[mbb->mb_index++] = i & 0xFF;						// CRC: Lo then Hi
    mbb->p_mb_buff[mbb->mb_index  ] = i >> 8;
    mbb->response_size = mbb->mb_index+1;
    return NeedResponse? true:false;
}

void mb_parsing(MBStruct_t *mbb)
{
    if( invalid_frame(mbb))
    {
        mbb->mb_state = MB_STATE_IDLE;
        if (mbb->f_start_receive != NULL)
        {
            mbb->f_start_receive(mbb);
        }
        return;
    }

    if( frame_parse(mbb)) // Returns TRUE if a response is needed
    {
        mbb->mb_state = MB_STATE_SEND;
        mbb->mb_index = 0;
        mbb->f_start_trans(mbb);
    }
    else
    {
        mbb->mb_state = MB_STATE_IDLE;
        if (mbb->f_start_receive != NULL)
        {
            mbb->f_start_receive(mbb);
        }
    }
#if (MB_CALLBACK_REG == 1)
    if  (mbb->cb_state == MB_CB_PRESENT)
    {
        mbb->wr_callback(mbb);
        mbb->cb_state = MB_CB_FREE;
    }
#endif
}
