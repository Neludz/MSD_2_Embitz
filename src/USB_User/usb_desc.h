/**
  ******************************************************************************
  * @file    usb_desc.h
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Descriptor Header for Virtual COM Port Device
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

// Interface numbers
#define MSC_INTERFACE_IDX 0x0			// Index of MSC interface
#define CDC_INTERFACE_IDX 0x1			// Index of CDC interface
#define USBD_MAX_NUM_INTERFACES     3
#define MSC_EP_IDX                      0x01
#define CDC_CMD_EP_IDX                  0x02
#define CDC_EP_IDX                      0x03
#define MSC_INTERFACE_IDX 0x0			// Index of MSC interface
#define CDC_INTERFACE_IDX 0x1			// Index of CDC interface

#define  USBD_IDX_LANGID_STR                            0x00
#define  USBD_IDX_MFC_STR                               0x01
#define  USBD_IDX_PRODUCT_STR                           0x02
#define  USBD_IDX_SERIAL_STR                            0x03
#define  USBD_IDX_CONFIG_STR                            0x04
#define  USBD_IDX_INTERFACE_STR                         0x05

#define VIRTUAL_COM_PORT_DATA_SIZE              64
#define VIRTUAL_COM_PORT_INT_SIZE               8

/* Exported functions ------------------------------------------------------- */

#define Composite_SIZ_DEVICE_DESC               18
#define Composite_SIZ_CONFIG_DESC               98
#define Composite_SIZ_STRING_LANGID             4
#define Composite_SIZ_STRING_VENDOR             12
#define Composite_SIZ_STRING_PRODUCT            48
#define Composite_SIZ_STRING_SERIAL             12

#define STANDARD_ENDPOINT_DESC_SIZE             0x09
#define MASS_SIZ_STRING_INTERFACE        		 22

/* Exported functions ------------------------------------------------------- */
extern const uint8_t Composite_DeviceDescriptor[Composite_SIZ_DEVICE_DESC];
extern const uint8_t Composite_ConfigDescriptor[Composite_SIZ_CONFIG_DESC];
extern const uint8_t Composite_StringLangID[Composite_SIZ_STRING_LANGID];
extern const uint8_t Composite_StringVendor[Composite_SIZ_STRING_VENDOR];
extern const uint8_t Composite_StringProduct[Composite_SIZ_STRING_PRODUCT];
extern  uint8_t Composite_StringSerial[Composite_SIZ_STRING_SERIAL];
extern const uint8_t MASS_StringInterface[MASS_SIZ_STRING_INTERFACE];

#endif /* __USB_DESC_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
