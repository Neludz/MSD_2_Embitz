/**
  ******************************************************************************
  * @file    usb_prop.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   All processing related to Virtual Com Port Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "hw_config.h"
#include "usb_bot.h"
#include "memory.h"
#include "mass_mal.h"
//#include "virtualComPort.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Request = 0;

LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
  };

/* -------------------------------------------------------------------------- */
/*  Structures initializations */
/* -------------------------------------------------------------------------- */

DEVICE Device_Table =
  {
    EP_NUM,
    1
  };

DEVICE_PROP Device_Property =
  {
    Copmosite_init,
	Copmosite_Reset,
	Copmosite_Status_In,
	Copmosite_Status_Out,
	Copmosite_Data_Setup,
	Copmosite_NoData_Setup,
	Copmosite_Get_Interface_Setting,
	Copmosite_GetDeviceDescriptor,
	Copmosite_GetConfigDescriptor,
	Copmosite_GetStringDescriptor,
    0,
    0x40 /*MAX PACKET SIZE*/
  };

USER_STANDARD_REQUESTS User_Standard_Requests =
  {
	Copmosite_GetConfiguration,
	Copmosite_SetConfiguration,
	Copmosite_GetInterface,
	Copmosite_SetInterface,
	Copmosite_GetStatus,
	Copmosite_ClearFeature,
	Copmosite_SetEndPointFeature,
	Copmosite_SetDeviceFeature,
	Copmosite_SetDeviceAddress
  };

ONE_DESCRIPTOR Device_Descriptor =
  {
	(uint8_t*)Composite_DeviceDescriptor,
	Composite_SIZ_DEVICE_DESC
  };

ONE_DESCRIPTOR Config_Descriptor =
  {
  (uint8_t*)Composite_ConfigDescriptor,
  Composite_SIZ_CONFIG_DESC
  };

ONE_DESCRIPTOR String_Descriptor[5] =
  {
		    {(uint8_t*)Composite_StringLangID, Composite_SIZ_STRING_LANGID},
		    {(uint8_t*)Composite_StringVendor, Composite_SIZ_STRING_VENDOR},
		    {(uint8_t*)Composite_StringProduct, Composite_SIZ_STRING_PRODUCT},
		    {(uint8_t*)Composite_StringSerial, Composite_SIZ_STRING_SERIAL},
			{(uint8_t*)MASS_StringInterface, MASS_SIZ_STRING_INTERFACE},
  };

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern unsigned char Bot_State;
extern Bulk_Only_CBW CBW;
uint32_t Max_Lun = 0;

/*******************************************************************************
* Function Name  : Get_Max_Lun
* Description    : Handle the Get Max Lun request.
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *Get_Max_Lun(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = LUN_DATA_LENGTH;
    return 0;
  }
  else
  {
    return((uint8_t*)(&Max_Lun));
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_init.
* Description    : Virtual COM Port Mouse init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_init(void)
{

  /* Update the serial number string descriptor with the data from the unique
  ID*/
//  Get_SerialNum();

  pInformation->Current_Configuration = 0;

  /* Connect the device */
  PowerOn();

  /* Perform basic device initialization operations */
  USB_SIL_Init();

  /* configure the USART to the default settings */
  //USART_Config_Default();

  bDeviceState = UNCONNECTED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_Reset(void)
{
  /* Set Virtual_Com_Port DEVICE as not configured */
  pInformation->Current_Configuration = 0;

  /* Current Feature initialization */
  pInformation->Current_Feature = Composite_ConfigDescriptor[7];

  /* Set Virtual_Com_Port DEVICE with the default Interface*/
  pInformation->Current_Interface = 0;

  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
  SetEPTxStatus(ENDP0, EP_TX_STALL);
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  Clear_Status_Out(ENDP0);
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
  SetEPRxValid(ENDP0);

  /* Initialize Endpoint 1 IN */
  SetEPType(ENDP1, EP_BULK);
  SetEPTxCount(ENDP1, 64);
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);
  SetEPTxStatus(ENDP1, EP_TX_NAK);

  /* Initialize Endpoint 1 OUT */
  SetEPType(ENDP1, EP_BULK);
  SetEPRxAddr(ENDP1, ENDP1_RXADDR);
  SetEPRxCount(ENDP1, 64);
  SetEPRxStatus(ENDP1, EP_RX_VALID);

  /* Initialize Endpoint 2 */
  SetEPType(ENDP2, EP_INTERRUPT);
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);
  SetEPRxStatus(ENDP2, EP_RX_DIS);
  SetEPTxStatus(ENDP2, EP_TX_NAK);

  /* Initialize Endpoint 3 */
  SetEPType(ENDP3, EP_BULK);
  SetEPTxAddr(ENDP3, ENDP3_TXADDR);
  SetEPTxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
  SetEPTxStatus(ENDP3, EP_TX_NAK);
  //SetEPRxStatus(ENDP3, EP_RX_DIS);

  /* Initialize Endpoint 3 */
  SetEPType(ENDP3, EP_BULK);
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
  SetEPRxStatus(ENDP3, EP_RX_VALID);
  //SetEPTxStatus(ENDP3, EP_TX_DIS);



  /* Set this device to response on default address */
  SetDeviceAddress(0);
  
  bDeviceState = ATTACHED;

  CBW.dSignature = BOT_CBW_SIGNATURE;
  Bot_State = BOT_IDLE;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to configured.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_SetConfiguration(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;

    ClearDTOG_TX(ENDP1);
    ClearDTOG_RX(ENDP1);
    Bot_State = BOT_IDLE; /* set the Bot state machine to the IDLE state */
  }
}

/*******************************************************************************
* Function Name  : Mass_Storage_ClearFeature
* Description    : Handle the ClearFeature request.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_ClearFeature(void)
{
  /* when the host send a CBW with invalid signature or invalid length the two
     Endpoints (IN & OUT) shall stall until receiving a Mass Storage Reset     */
  if (CBW.dSignature != BOT_CBW_SIGNATURE)
    Bot_Abort(BOTH_DIR);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_SetDeviceAddress (void)
{
  bDeviceState = ADDRESSED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_Status_In(void)
{
  if (Request == SET_LINE_CODING)
  {
   // USART_Config();
    Request = 0;
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_Out
* Description    : Virtual COM Port Status OUT Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Copmosite_Status_Out(void)
{}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Copmosite_Data_Setup(uint8_t RequestNo)
{
  uint8_t    *(*CopyRoutine)(uint16_t);

  CopyRoutine = NULL;

  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
      && (RequestNo == GET_MAX_LUN) && (pInformation->USBwValue == 0)
      && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x01))
  {
    CopyRoutine = Get_Max_Lun;
  }

  else if (RequestNo == GET_LINE_CODING)
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
      CopyRoutine = Copmosite_GetLineCoding;
    }
  }
  else if (RequestNo == SET_LINE_CODING)
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
      CopyRoutine = Copmosite_SetLineCoding;
    }
    Request = SET_LINE_CODING;
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Copmosite_NoData_Setup(uint8_t RequestNo)
{

	  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
	        && (RequestNo == MASS_STORAGE_RESET) && (pInformation->USBwValue == 0)
	        && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x00))
	    {
	      /* Initialize Endpoint 1 */
	      ClearDTOG_TX(ENDP1);

	      /* Initialize Endpoint 2 */
	      ClearDTOG_RX(ENDP1);

	      /*initialize the CBW signature to enable the clear feature*/
	      CBW.dSignature = BOT_CBW_SIGNATURE;
	      Bot_State = BOT_IDLE;

	      return USB_SUCCESS;
	    }

	  else if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
  {
    if (RequestNo == SET_COMM_FEATURE)
    {
      return USB_SUCCESS;
    }
    else if (RequestNo == SET_CONTROL_LINE_STATE)
    {
      return USB_SUCCESS;
    }
  }
  return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
uint8_t *Copmosite_GetDeviceDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetConfigDescriptor.
* Description    : get the configuration descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *Copmosite_GetConfigDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetStringDescriptor
* Description    : Gets the string descriptors according to the needed index
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
uint8_t *Copmosite_GetStringDescriptor(uint16_t Length)
{
  uint8_t wValue0 = pInformation->USBwValue0;
  if (wValue0 >= 5)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
  }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.
* Input1         : uint8_t: Interface : interface number.
* Input2         : uint8_t: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT Copmosite_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
  if (AlternateSetting > 0)
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 3)
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetLineCoding.
* Description    : send the linecoding structure to the PC host.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Copmosite_GetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetLineCoding.
* Description    : Set the linecoding structure fields.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Copmosite_SetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

