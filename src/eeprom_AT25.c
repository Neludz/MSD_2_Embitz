
#include <eeprom_AT25.h>

/*
 * SPI
 */

#define AT25_select() IO_SetLine(io_Eeprom_CS,OFF)
#define AT25_release() IO_SetLine(io_Eeprom_CS,ON)


uint16_t AT25_rxtx(uint16_t data)
{
  SPI2->DR = (data);

  while(!(SPI2->SR & SPI_SR_RXNE))
    ;
  return SPI2->DR;
}


#define AT25_rx() AT25_rxtx(0xFF)
#define AT25_tx(data) AT25_rxtx(data)



uint8_t AT25_get_status_bit(void)
{
	uint8_t data;

    AT25_select();
	AT25_tx(AT25_read_status_com);
	data = AT25_rx();
	AT25_release();

	return ((uint8_t) data);
}


void AT25_read_byte(uint16_t adr, uint8_t *data_out, uint16_t len)
{


        while ((AT25_get_status_bit())&(AT_25_RDY_bit))
        ;
	AT25_select();
	AT25_tx(AT25_read_com);
	AT25_tx((adr>>8));
	AT25_tx((adr & 0xFF));
        for (uint32_t i=0; i<len; i++)
        {
        data_out[i] = AT25_rx();
        }
	AT25_release();
	return;
}


void AT25_write_byte(uint16_t adr, uint8_t *data_in, uint16_t len)
{
    for (uint32_t i=0; i<len; i++)
    {
    while ((AT25_get_status_bit())&(AT_25_RDY_bit))
        ;
    AT25_select();
    AT25_rxtx(AT25_set_write_com);
    AT25_release();

    AT25_select();
    AT25_tx(AT25_write_com);
    AT25_tx(adr>>8);
    AT25_tx((adr & 0xFF));
    AT25_tx(data_in[i]);
    AT25_release();

    adr++;
    }
}


void AT25_update_byte(uint16_t adr, uint8_t *data_in, uint16_t len)
{

uint8_t check;

for (uint32_t i=0; i<len; i++)
    {
    while ((AT25_get_status_bit())&(AT_25_RDY_bit))
        ;
	AT25_select();
	AT25_tx(AT25_read_com);
	AT25_tx((adr>>8));
	AT25_tx((adr & 0xFF));

    check = AT25_rx();

	AT25_release();

     if (check !=data_in[i])
        {
        while ((AT25_get_status_bit())&(AT_25_RDY_bit))
            ;
        AT25_select();
        AT25_rxtx(AT25_set_write_com);
        AT25_release();

        AT25_select();
        AT25_tx(AT25_write_com);
        AT25_tx(adr>>8);
        AT25_tx((adr & 0xFF));
        AT25_tx(data_in[i]);
        AT25_release();
        }
    adr++;
    }
}


void AT_25_Init (void)
{

    while ((AT25_get_status_bit())&(AT_25_RDY_bit))
        ;
    AT25_select();
    AT25_rxtx(AT25_set_write_com);
    IO_SetLine(io_Eeprom_CS,ON);

    AT25_select();
    AT25_tx(AT25_write_status_com);
    AT25_tx(0x00);
    AT25_release();


}
