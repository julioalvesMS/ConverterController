#include <src/Config/COM_SERIAL.h>


//
// TRANSMITE PELA SERIAL - RS232 B (DIREITA)
//
void SCIB_TX(void)
{
	int index =0;

    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;

    while(RS232B_message[index] != '\0')
    {
        ScibRegs.SCITXBUF.bit.TXDT = RS232B_message[index];
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

	    RS232B_buffer = ScibRegs.SCIRXBUF.bit.SAR;

        switch(RS232B_buffer)
        {
        case 'A':
            break;
        default:
            break;
        }

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

    while(RS232C_message[index] != '\0')
    {
        ScicRegs.SCITXBUF.bit.TXDT = RS232C_message[index];
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

        RS232C_buffer = ScicRegs.SCIRXBUF.bit.SAR;

        switch(RS232C_buffer)
        {
        case 'A':
            break;
        default:
            break;
        }

        ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;   // reset RX-FIFO pointer
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;   // enable RX-operation
        ScicRegs.SCIRXST.bit.RXERROR = 1;
    }
}

