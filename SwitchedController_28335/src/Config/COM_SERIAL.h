
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <string.h>
#include <stdio.h>

extern char message[8];                             //serial
extern char message2[4];                                       //serial
extern int index;			                                //serial
extern Uint16 buffer;
//#######################################TRANSMITE PELA SERIAL######################################
void SCIA_TX(void)
{
	int index =0;

	 	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;

		while(message[index] != '\0')
	    {
			SciaRegs.SCITXBUF= message[index];
			index++;

	    }
			while(SciaRegs.SCIFFTX.bit.TXFFST != 0);

		SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 0;
}

//#######################################RECEBE PELA SERIAL#########################################
void SCIA_RX(void)
{
	if(SciaRegs.SCIFFRX.bit.RXFFST>=1){

	buffer = SciaRegs.SCIRXBUF.bit.RXDT;

///////////////////////////////////////////CLR//////////////////////////////////////////////////////


	//	if (buffer == 'T')
	//	{
	//	}

	SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;	// reset RX-FIFO pointer
	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX-operation
	SciaRegs.SCIRXST.bit.RXERROR = 1;
	}
}

//###########################################FIM DO CÓDIGO##########################################
