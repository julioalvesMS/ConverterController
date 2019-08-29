
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File



//Protótipo das funções básicas para configuração e realização dos testes colocar no programa principal
//extern spi_fifo_init();  //  colocar no início
//extern   spi_init();     // colocar no início
//extern enviar_dac_spi_uni(int ,Uint16);// Quando for enviar para um canal apenas (canal(0-3), dados)
//extern enviar_dac_spi_4Canais(Uint16,Uint16,Uint16,Uint16);//Envia para os 4 canais

extern  long DelaySPI;

void spi_init()
{
	SpiaRegs.SPICCR.all =0x000F;	             // Reset on, rising edge, 12-bit char bits
	SpiaRegs.SPICTL.all =0x0006;    		     // Enable master mode, normal phase,
	SpiaRegs.SPICTL.bit.CLK_PHASE=0;              // enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x0001;				   	// ajusta o clock do SPI
    SpiaRegs.SPICCR.all =0x00CF;		         // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.CLKPOLARITY=0;
    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
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


void enviar_dac_spi_uni(Uint16 A,Uint16 sdatai){
long i;
	sdatai=((0x0008+A)<<12)+sdatai;
	 spi_xmit(sdatai);
	   for (i = 0; i < 50; i++) {            // DELAY
		   DelaySPI=sdatai+i;
	   }

}


void enviar_dac_spi_4Canais(Uint16 sdata0,Uint16 sdata1,Uint16 sdata2,Uint16 sdata3){
	    enviar_dac_spi_uni(0,sdata0);
		enviar_dac_spi_uni(1,sdata1);
		enviar_dac_spi_uni(2,sdata2);
		enviar_dac_spi_uni(3,sdata3);
}





