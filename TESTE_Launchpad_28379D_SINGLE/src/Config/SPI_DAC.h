#ifndef SPI_DAC_H_
#define SPI_DAC_H_

#include "F28x_Project.h"   // Device Headerfile and Examples Include File


void spi_init();

void spi_xmit(Uint16 a);

void spi_fifo_init();

void enviar_dac_spi_uni(Uint16 A,Uint16 sdatai);

void enviar_dac_spi_4Canais(Uint16 sdata0,Uint16 sdata1,Uint16 sdata2,Uint16 sdata3);


#endif /* SPI_DAC_H_ */
