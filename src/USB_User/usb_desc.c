/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Descriptors for Virtual Com Port Demo
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
#include "usb_desc.h"

/* USB Standard Device Descriptor */
//**********************************************************************
const uint8_t Composite_DeviceDescriptor[] =
  {
    0x12,   /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
    0x00,
    0x02,   /* bcdUSB = 2.00 */
    0xEF,   /* bDeviceClass*/
    0x02,   /* bDeviceSubClass */
    0x01,   /* bDeviceProtocol */
    0x40,   /* bMaxPacketSize0 */
    0xC0,
    0x16,   /* idVendor = 0x0483 */
    0xE1,
    0x27,   /* idProduct = 0x5741 */
    0x00,
    0x02,   /* bcdDevice = 2.00 */
    1,              /* Index of string descriptor describing manufacturer */
    2,              /* Index of string descriptor describing product */
    3,              /* Index of string descriptor describing the device's serial number */
    0x01    /* bNumConfigurations */
  };

const uint8_t Composite_ConfigDescriptor[Composite_SIZ_CONFIG_DESC] =
  {
    /*Configuration Descriptor*/
    0x09,   /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
	Composite_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
    0x00,
    0x03,   /* bNumInterfaces: 3 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
    0xC0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 0 mA */
	/******************** Descriptor of Mass Storage interface ********************/
   /* 09 */
	0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    /*      Interface descriptor type */
	MSC_INTERFACE_IDX,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints*/
    0x08,   /* bInterfaceClass: MASS STORAGE Class */
    0x06,   /* bInterfaceSubClass : SCSI transparent*/
    0x50,   /* nInterfaceProtocol */
    4,          /* iInterface: */
    /* 09 */
	/********************  Mass Storage Endpoints ********************/
    0x07,   /*Endpoint descriptor length = 7*/
    0x05,   /*Endpoint descriptor type */
    0x81,   /*Endpoint address (IN, address 1) */
    0x02,   /*Bulk endpoint type */
    0x40,   /*Maximum packet size (64 bytes) */
    0x00,
	0x00,   /*Polling interval in milliseconds */
    /* 07 */
    0x07,   /*Endpoint descriptor length = 7 */
    0x05,   /*Endpoint descriptor type */
    0x01,   /*Endpoint address (OUT, address 1) */
    0x02,   /*Bulk endpoint type */
    0x40,   /*Maximum packet size (64 bytes) */
    0x00,
    0x00,     /*Polling interval in milliseconds*/
    /*07*/

	/******** IAD should be positioned just before the CDC interfaces ******
			 IAD to associate the two CDC interfaces */

	0x08, /* bLength */
	0x0B, /* bDescriptorType */
	CDC_INTERFACE_IDX, /* bFirstInterface */
	0x02, /* bInterfaceCount */
	0x02, /* bFunctionClass */
	0x02, /* bFunctionSubClass */
	0x00, /* bFunctionProtocol */
	0x00, /* iFunction (Index of string descriptor describing this function) */
	/* 08 bytes */
	/********************  CDC interfaces ********************/

	/*Interface Descriptor*/
    0x09,   /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
    /* Interface descriptor type */
	CDC_INTERFACE_IDX,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */
	/* 09 bytes */

	/*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,
	/* 05 bytes */

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
	CDC_INTERFACE_IDX + 1,   /* bDataInterface: 1 */
	/* 05 bytes */

	/*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */
	/* 04 bytes */

	/*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
	CDC_INTERFACE_IDX,   /* bMasterInterface: Communication class interface */
	CDC_INTERFACE_IDX + 1,   /* bSlaveInterface0: Data Class Interface */
	/* 05 bytes */

    /*Endpoint 2 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x82,   /* bEndpointAddress: (IN2) */
    0x03,   /* bmAttributes: Interrupt */
    VIRTUAL_COM_PORT_INT_SIZE,      /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: */
	/* 07 bytes */

    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
	CDC_INTERFACE_IDX + 1,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
	/* 09 bytes */

    /*Endpoint 3 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x03,   /* bEndpointAddress: (OUT3) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,   /* bInterval: ignore for Bulk transfer */
	/* 07 bytes */

    /*Endpoint 3 Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
    0x83,   /* bEndpointAddress: (IN3) */
    0x02,   /* bmAttributes: Bulk */
    VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
    0x00,
    0x00,    /* bInterval */
	/* 07 bytes */

  };

/* USB String Descriptors */
const uint8_t Composite_StringLangID[Composite_SIZ_STRING_LANGID] =
  {
    Composite_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t Composite_StringVendor[Composite_SIZ_STRING_VENDOR] =
  {
    Composite_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer:  */
    'M', 0, 'S', 0, 'D', 0, '_', 0, '2', 0
  };

const uint8_t Composite_StringProduct[Composite_SIZ_STRING_PRODUCT] =
  {
    Composite_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'M', 0, 'S', 0, 'D', 0, '_', 0, '2', 0, ' ', 0, 'C', 0,
    'o', 0, 'm', 0, 'p', 0, 'o', 0, 's', 0, 'i', 0, 't', 0,
    'e', 0,' ',0, 'M', 0, 'S', 0, 'C', 0, '+', 0, 'C', 0, 'D', 0, 'C', 0
  };
uint8_t Composite_StringSerial[Composite_SIZ_STRING_SERIAL] =
  {
    Composite_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'M', 0, 'S', 0, 'D', 0,'_', 0,'2', 0
  };

const uint8_t MASS_StringInterface[MASS_SIZ_STRING_INTERFACE] =
  {
    MASS_SIZ_STRING_INTERFACE,
    0x03,
    /* Interface 0: */
    'M', 0, 'S', 0, 'D', 0, '_', 0, '2', 0, ' ', 0, 'M', 0, 'a', 0, 's', 0, 's', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
