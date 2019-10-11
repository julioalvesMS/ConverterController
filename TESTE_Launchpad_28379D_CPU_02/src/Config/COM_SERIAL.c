#include <src/Config/COM_SERIAL.h>


//
// TRANSMITE PELA SERIAL - RS232 B (DIREITA)
//
void SCIB_TX(void)
{
	int index =0;

    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;

    while(message[index] != '\0')
    {
        ScibRegs.SCITXBUF.bit.TXDT = message[index];
        index++;
    }

    while(ScibRegs.SCIFFTX.bit.TXFFST != 0);

    ScibRegs.SCIFFTX.bit.TXFIFORESET = 0;
}

//
// RECEBE PELA SERIAL - RS232 B (DIREITA)
//
void SCIB_RX(void)
{
	if(ScibRegs.SCIFFRX.bit.RXFFST>=1)
	{

        buffer = ScibRegs.SCIRXBUF.bit.SAR;

        //	if (buffer == 'T')
        //	{
        //	}

        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;	// reset RX-FIFO pointer
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;	// enable RX-operation
        ScibRegs.SCIRXST.bit.RXERROR = 1;
	}
}


//
// TRANSMITE PELA SERIAL - RS232 C (ESQUERDA)
//
void SCIC_TX(void)
{
    int index =0;

    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;

    while(message[index] != '\0')
    {
        ScicRegs.SCITXBUF.bit.TXDT = message[index];
        index++;
    }

    while(ScicRegs.SCIFFTX.bit.TXFFST != 0);

    ScicRegs.SCIFFTX.bit.TXFIFORESET = 0;
}

//
// RECEBE PELA SERIAL - RS232 C (ESQUERDA)
//
void SCIC_RX(void)
{
    if(ScicRegs.SCIFFRX.bit.RXFFST>=1)
    {

        buffer = ScicRegs.SCIRXBUF.bit.SAR;

        //  if (buffer == 'T')
        //  {
        //  }

        ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;   // reset RX-FIFO pointer
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;   // enable RX-operation
        ScicRegs.SCIRXST.bit.RXERROR = 1;
    }
}

