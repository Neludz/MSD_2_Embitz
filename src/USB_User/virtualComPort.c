/**
  ******************************************************************************
  * @file    virtualComPort.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Virtual Com Port Configuration
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
#include <string.h>
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART_RX_DATA_SIZE   256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//USART_InitTypeDef USART_InitStructure;
//extern DMA_InitTypeDef  DMA_InitStructure;

#ifdef VCP_RX_BY_DMA
/* Use a double buffer for DMA/USB switch */
uint8_t  USART_Rx_Buffer[2][USART_RX_DATA_SIZE/2]; 

/* Id (0 or 1) of buffer being transferred to the USB. The DMA can not be restarted
   on this buffer until the previous transfer is finished (buffer empty). The DMA
   may be filling this buffer, while the USB is emptying it. Once the DMA transfer
   is completed, another DMA transfer may be initiated on the other buffer, if it
   is empty. Otherwise, there is an overflow, and one must wait for the end of USB
   before restarting the DMA */
static int bufId_To_Usb=0;

/* Id (0 or 1) of buffer being filled by the DMA */
static int bufId_To_Dma=0;

/* Amount of data copied from RAM buffer to USB buffers but still not sent over
   the USB. sizeReady_For_Usb must not exceed VIRTUAL_COM_PORT_DATA_SIZE */
static unsigned short sizeReady_For_Usb=0;

/* Amount of data that must be sent to the USB */
static unsigned short sizeNewDataRemainingToSend=0;

/* Amount of data from USART_Rx_Buffer that have already been sent to the USB.
  When sizeTransferredByUsb[bufId_To_Usb]>=USART_RX_DATA_SIZE/2,
  bufId_To_Usb is becoming empty again (ready for new DMA), and the other
  buffer must start being transferred to the USB (switch bufId_To_Usb) */
static unsigned short sizeTransferredByUsb[2]={0,0};

/* Flag managing the DMA restarting after a buffer overflow */
static bool bDelayed_Dma=FALSE;

#else /* !VCP_RX_BY_DMA */
uint8_t  USART_Rx_Buffer[USART_RX_DATA_SIZE]; 
extern uint32_t USART_Rx_ptr_in ;
extern uint32_t USART_Rx_ptr_out ;
extern uint32_t USART_Rx_length ;
extern uint8_t  USB_Tx_State ;
uint16_t USB_Tx_length;
uint16_t USB_Tx_ptr;
#endif /* VCP_RX_BY_DMA */

/* Extern variables ----------------------------------------------------------*/
//extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef VCP_RX_BY_DMA
/*******************************************************************************
* Function Name  : VCP_StartDMA.
* Description    : Start the DMA for Virtual Com Port Rx
* Input          : None.
* Return         : None.
*******************************************************************************/
void VCP_StartDMA(void)
{
/* Code partially imported from stm32f10x_usart.c and stm32f10x_dma.c for
  better performance (no call) */
#define CR1_UE_Set              ((uint16_t)0x2000)  /* USART Enable Mask */
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFE)
#define CCR_ENABLE_Set          ((uint32_t)0x00000001)
  
/* DMA has to be disabled in order to be able to write into CNDTR */
  VCP_RX_DMA_CHANNEL->CCR &= CCR_ENABLE_Reset;
  VCP_RX_DMA_CHANNEL->CNDTR = USART_RX_DATA_SIZE/2;
  VCP_RX_DMA_CHANNEL->CMAR = (uint32_t)USART_Rx_Buffer[bufId_To_Dma];
  
/* Restart all
   Enable RX DMA channel */
  VCP_RX_DMA_CHANNEL->CCR |= CCR_ENABLE_Set;
  
/* Enable the VCP_USART */
  VCP_USART->CR1 |= CR1_UE_Set;
}

/*******************************************************************************
* Function Name  : VCP_RX_DMA_Channel_ISR.
* Description    : Interrupt handler called from stm32f10x_it.c, after DMA end
*                : of transfer. If the previous buffer was totally sent, a new
*                : DMA transfer is restarted. Otherwise (overflow state), the
*                : new DMA transfer will be initiated when the buffer is becoming
*                : empty.
* Input          : None.
* Return         : None.
*******************************************************************************/
void VCP_RX_DMA_Channel_ISR(void) 
{
  if (DMA1->ISR & VCP_RX_DMA_IT_TC) 
  {
    /* Transfer complete: clear interrupt flag */
    DMA1->IFCR |= VCP_RX_DMA_FLAG_GL | VCP_RX_DMA_FLAG_TC;
    
    /* Restart a new DMA transfer if possible */
    if( sizeTransferredByUsb[1-bufId_To_Dma] == USART_RX_DATA_SIZE/2 ) 
    {
      /* The other buffer is free: launch a new DMA on it */
      bufId_To_Dma = 1 - bufId_To_Dma;
      sizeTransferredByUsb[bufId_To_Dma] = 0;
      
      VCP_StartDMA();
    } 
    else 
    {
      /* No buffer is free yet => overflow. The DMA will have to be restarted later */
      bDelayed_Dma = TRUE;
    }
  }
}

/*******************************************************************************
* Function Name  : VCP_GetSizeOfNewData.
* Description    : Get the size of newly received data (since last call). This
*                : triggers the first transfer from RAM buffer to USB buffer.
*                : Further transfers (if any) will be initiated by the USB
*                : transfer complete interrupt (EPx_IN_Callback).
* Input          : None.
* Return         : The size in bytes of pending data.
*******************************************************************************/
unsigned short VCP_GetSizeOfNewData(void)
{
  if( sizeNewDataRemainingToSend > 0 ) 
  {
    /* The previous buffer was not completely sent to the USB => must wait for
    the end before preparing new data;
    Or the trace was not activated */
    return 0;
  }
 /* Prepare a new block of data to send over the USB */
  if( bufId_To_Usb == bufId_To_Dma )
  {
    /* The buffer to transmit is currently being filled by the DMA */
    sizeNewDataRemainingToSend = USART_RX_DATA_SIZE/2 - DMA_GetCurrDataCounter(VCP_RX_DMA_CHANNEL) 
      /* he nb of data received by the DMA */
      - sizeTransferredByUsb[bufId_To_Usb];  /* Minus the data already sent in a previous sequence */
  } 
  
  else 
  {
    /* The DMA switched to the other buffer, which means the buffer for USB is full */
    sizeNewDataRemainingToSend = USART_RX_DATA_SIZE/2
      - sizeTransferredByUsb[bufId_To_Usb]; /* Minus the data already sent in a previous sequence */
  }
  
  VCP_SendRxBufPacketToUsb();
  
  return sizeNewDataRemainingToSend;
}
#endif /* VCP_RX_BY_DMA */

/*******************************************************************************
* Function Name  : VCP_Data_InISR.
* Description    : EPxIN USB transfer complete ISR. Send pending data if any.
* Input          : None.
* Return         : none.
*******************************************************************************/

void VCP_Data_InISR(void)
{
#ifdef VCP_RX_BY_DMA
   /* Previous USB transfer completed. Update counters and prepare next transfer
   if required */
  sizeTransferredByUsb[bufId_To_Usb] += sizeReady_For_Usb;
  sizeNewDataRemainingToSend -= sizeReady_For_Usb;
  sizeReady_For_Usb = 0;
  
  if( sizeNewDataRemainingToSend != 0 ) 
  {
    VCP_SendRxBufPacketToUsb();
  }
  if( sizeTransferredByUsb[bufId_To_Usb] >= USART_RX_DATA_SIZE/2 ) 
  {
    /* The whole RAM buffer was sent over the USB */
    if( bDelayed_Dma==TRUE ) 
    {
      bDelayed_Dma=FALSE;
      /* A new DMA transfer must be initiated: DMA has to be disabled in order to
      be able to write into CNDTR */
      bufId_To_Dma = 1-bufId_To_Dma;
      sizeTransferredByUsb[bufId_To_Dma] = 0;
      VCP_StartDMA();
    }
    /* Switch the buffer */
    bufId_To_Usb = 1-bufId_To_Usb;
  }
#else
  VCP_SendRxBufPacketToUsb();
#endif
}


/*******************************************************************************
* Function Name  : VCP_SendRxBufPacketToUsb.
* Description    : send data from USART_Rx_Buffer to the USB. Manage the segmentation
*                  into USB FIFO buffer. Commit one packet to the USB at each call.
* Input          : globals:
*                  - USB_Tx_State: transmit state variable
*                  - USART_Rx_Buffer: buffer of data to be sent
*                  - USART_Rx_length: amount of data (in bytes) ready to be sent
*                  - USART_Rx_ptr_out: index in USART_Rx_Buffer of the first data
*                    to send
* Return         : none.
*******************************************************************************/
void VCP_SendRxBufPacketToUsb(void) {
#ifdef VCP_RX_BY_DMA
  unsigned short sizeToSend;
  
  if( sizeNewDataRemainingToSend != 0 ) 
  {
    /* There is something to send: prepare the USB buffer */
    if( sizeNewDataRemainingToSend > VIRTUAL_COM_PORT_DATA_SIZE ) 
    {
      sizeToSend = VIRTUAL_COM_PORT_DATA_SIZE;
    }
    else {
      sizeToSend = sizeNewDataRemainingToSend;
    }
 
    UserToPMABufferCopy((uint8_t*)&(USART_Rx_Buffer[bufId_To_Usb][sizeTransferredByUsb[bufId_To_Usb]]),
                          ENDP1_TXADDR, sizeToSend);
    sizeReady_For_Usb = sizeToSend;
    SetEPTxCount(ENDP1, sizeReady_For_Usb);
    SetEPTxValid(ENDP1);
  }
#else
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
  if (USB_Tx_State == 1)
  {
    if (USART_Rx_length == 0) 
    {
      USB_Tx_State = 0;
    }
    else 
    {
      if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE){
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
        
        USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
        USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;    
      }
      else 
      {
        USB_Tx_ptr = USART_Rx_ptr_out;
        USB_Tx_length = USART_Rx_length;
        
        USART_Rx_ptr_out += USART_Rx_length;
        USART_Rx_length = 0;
      }
      
      UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP3_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP3, USB_Tx_length);
      SetEPTxValid(ENDP3);
    }
  }
#endif
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (void)
{
#ifdef VCP_RX_BY_DMA
  VCP_GetSizeOfNewData();
#else

  if(USB_Tx_State != 1)
  {
    if (USART_Rx_ptr_out == USART_RX_DATA_SIZE)
    {
      USART_Rx_ptr_out = 0;
    }
    
    if(USART_Rx_ptr_out == USART_Rx_ptr_in) 
    {
      USB_Tx_State = 0; 
      return;
    }
    
    if(USART_Rx_ptr_out > USART_Rx_ptr_in) /* rollback */
    { 
      USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
    }
    else 
    {
      USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
    }
    
    if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE)
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
      
      USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;	
      USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;	
    }
    else
    {
      USB_Tx_ptr = USART_Rx_ptr_out;
      USB_Tx_length = USART_Rx_length;
      
      USART_Rx_ptr_out += USART_Rx_length;
      USART_Rx_length = 0;
    }
    USB_Tx_State = 1; 
    
    UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP3_TXADDR, USB_Tx_length);
    SetEPTxCount(ENDP3, USB_Tx_length);
    SetEPTxValid(ENDP3);
  }
#endif
}

/*******************************************************************************
* Function Name  : UART_To_USB_Send_Data.
* Description    : send the received data from UART 0 to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void USB_Send_Data(uint8_t data)
{
  USART_Rx_Buffer[USART_Rx_ptr_in] = data;
  USART_Rx_ptr_in++;

  /* To avoid buffer overflow */
  if(USART_Rx_ptr_in >= USART_RX_DATA_SIZE)
  {
    USART_Rx_ptr_in = 0;
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
