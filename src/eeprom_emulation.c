
/*
 * EEPROM on flash
 */
/*-----------------------------------------------------------------------*/
// include
/*-----------------------------------------------------------------------*/
#include <eeprom_emulation.h>
#include <stm32f1xx.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"

/*-----------------------------------------------------------------------*/
// Data from linker script
/*-----------------------------------------------------------------------*/
extern uint32_t _eeprom_addr;
uint32_t const *data_eeprom_p = &_eeprom_addr;
extern uint32_t _eeprom_len;
uint32_t const data_eeprom_len = (uint32_t)&_eeprom_len;

/*-----------------------------------------------------------------------*/
//Prototype
/*-----------------------------------------------------------------------*/
static EEPRESULT EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static EEPRESULT EE_ProgramHalfWord(uint32_t Address, uint16_t Data);
static EEPRESULT EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static void EE_WaitBusyFlaf(void);

/*-----------------------------------------------------------------------*/
// Read from ee_flash
/*-----------------------------------------------------------------------*/
EEPRESULT EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data)
{
    uint16_t AddressValue;
    EEPRESULT ReadStatus = RES_NOTFND;
    uint32_t Address;

    /* Get the end Address */
    Address = (uint32_t)(((uint32_t)EEPROM_START_ADDRESS - 2) + (uint32_t)PAGE_SIZE);

    /* Check each active page address starting from end */
    while (Address >= ((uint32_t)EEPROM_START_ADDRESS + 2))
    {
        /* Get the current location content to be compared with virtual address */
        AddressValue = (*(__IO uint16_t*)Address);

        /* Compare the read address with the virtual address */
        if (AddressValue == VirtAddress)
        {
            /* Get content of Address-2 which is variable value */
            *Data = (*(__IO uint16_t*)(Address - 2));

            /* In case variable value is read, reset ReadStatus flag */
            ReadStatus = RES_OK;

            break;
        }
        else
        {
            /* Next address location */
            Address = Address - 4;
        }
    }
    return ReadStatus;
}

/*-----------------------------------------------------------------------*/
// Write to ee_flash
/*-----------------------------------------------------------------------*/
EEPRESULT EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
    EEPRESULT Status = RES_ERROR;
    // unlock flash
    if((FLASH->CR & FLASH_CR_LOCK) != 0)
    {
        /* Authorize the FLASH Registers access */
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        /* Verify Flash is unlocked */
        if((FLASH->CR & FLASH_CR_LOCK) != 0)
        {
            return RES_ERROR;
        }
    }

    /* Write the variable virtual address and value in the EEPROM */
    Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

    /* In case the EEPROM active page is full */
    if (Status == RES_PAGEFULL)
    {
        /* Perform Page transfer */
        Status = EE_PageTransfer(VirtAddress, Data);
    }
    // lock flash
    FLASH->CR |= FLASH_CR_LOCK;
    /* Return last operation status */
    return Status;
}

/*-----------------------------------------------------------------------*/
// Update to ee_flash
/*-----------------------------------------------------------------------*/
EEPRESULT EE_UpdateVariable(uint16_t VirtAddress, uint16_t Data)
{
    uint16_t read_data=0;
     EEPRESULT Status = EE_ReadVariable(VirtAddress, &read_data);
    if ((read_data!=Data) || (Status != RES_OK))
    {
       EE_WriteVariable (VirtAddress,  Data);
    }
}


/*-----------------------------------------------------------------------*/
//
/*-----------------------------------------------------------------------*/
static EEPRESULT EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
    EEPRESULT Status = RES_OK;
    uint32_t Address, PageEndAddress;

    /* Get the valid Page start Address */
    Address = (uint32_t)((uint32_t)EEPROM_START_ADDRESS);

    /* Get the valid Page end Address */
    PageEndAddress = (uint32_t)(((uint32_t)EEPROM_START_ADDRESS - 2) + PAGE_SIZE);

    /* Check each active page address starting from begining */
    while (Address < PageEndAddress)
    {
        /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
        if ((*(__IO uint32_t*)Address) == 0xFFFFFFFF)
        {
            /* Set variable data */

            Status = EE_ProgramHalfWord(Address, Data);
            /* If program operation was failed, a Flash error code is returned */
            if (Status != RES_OK)
            {
                return Status;
            }
            /* Set variable virtual address */
            Status = EE_ProgramHalfWord(Address + 2, VirtAddress);

            /* Return program operation status */
            return Status;
        }
        else
        {
            /* Next address location */
            Address = Address + 4;
        }
    }
    /* Return PAGE_FULL in case the valid page is full */
    return RES_PAGEFULL;
}

/*-----------------------------------------------------------------------*/
//
/*-----------------------------------------------------------------------*/
static EEPRESULT EE_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
    EEPRESULT status = RES_OK;
    /* Wait for last operation to be completed */
    EE_WaitBusyFlaf();

    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint16_t*)Address = Data;
    /* Wait for last operation to be completed */
    EE_WaitBusyFlaf();
    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= ~(FLASH_CR_PG);
    /* Return the Program Status */
    return status;
}

/*-----------------------------------------------------------------------*/
//
/*-----------------------------------------------------------------------*/
static EEPRESULT EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
    uint16_t  VirtAddressNext, VirtAddressLast, NextData;
    EE_WaitBusyFlaf();

    /* Proceed to erase the page */
    FLASH->CR |= (FLASH_CR_PER);
    FLASH->AR = (uint32_t)EEPROM_START_ADDRESS;
    FLASH->CR |= (FLASH_CR_STRT);

    EE_WaitBusyFlaf();

    FLASH->CR &= ~(FLASH_CR_PER);
    VirtAddressLast=0;
    while (GetNextVirtAddrData(VirtAddressLast, &VirtAddressNext, &NextData)==RES_OK)
    {
        EE_WriteVariable(VirtAddressNext, NextData);
        VirtAddressLast = VirtAddressNext;
    }
     return RES_OK;
}

/*-----------------------------------------------------------------------*/
//
/*-----------------------------------------------------------------------*/
static void EE_WaitBusyFlaf(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0 )
    {
        #if defined xPortSysTickHandler
        if (xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)
        {
           taskYIELD();
        }
        #endif
    }
}
