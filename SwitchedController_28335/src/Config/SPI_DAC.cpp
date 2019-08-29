#include <src/Config/SPI_DAC.h>

long DelaySPI;

void spi_init()
{
	SpiaRegs.SPICCR.all = 0x000F;           // Reset on, rising edge, 12-bit char bits
	SpiaRegs.SPICTL.all = 0x0006;           // Enable master mode, normal phase,
	SpiaRegs.SPICTL.bit.CLK_PHASE = 0;      // enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR = 0x0001;               // Set SPI Clock
    SpiaRegs.SPICCR.all = 0x00CF;           // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPIPRI.bit.FREE = 1;           // Set so breakpoints don't disturb xmission
}

void spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

void spi_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x204f;
    SpiaRegs.SPIFFCT.all=0x0;
}


void enviar_dac_spi_uni(Uint16 A,Uint16 sdatai)
{
    int i;
	sdatai=((0x0008+A)<<12)+sdatai;
	spi_xmit(sdatai);
	for (i = 0; i < 50; i++) {              // DELAY
	    DelaySPI=sdatai+i;
	}

}


void enviar_dac_spi_4Canais(Uint16 sdata0,Uint16 sdata1,Uint16 sdata2,Uint16 sdata3)
{
	    enviar_dac_spi_uni(0,sdata0);
		enviar_dac_spi_uni(1,sdata1);
		enviar_dac_spi_uni(2,sdata2);
		enviar_dac_spi_uni(3,sdata3);
}

