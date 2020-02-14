#include <src/Serial/serial.h>


extern int grupo_dac;

extern CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;


void Serial_Configure(void)
{
    SCIB_init();        // Initializa e configura  SCIA- RS232
}

void Serial_GenerateProtocol()
{
    switch(data_index)
    {
    case 0:
        sprintf(protocol_message, "D%1d", (int) grupo_dac);
        break;
    default:
        break;
    }

    data_index = (data_index+1)%MESSAGES_COUNT;
}

void Serial_SendMessage(void)
{
    int index;

    Serial_GenerateProtocol();

    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;

    for(index = 0; protocol_message[index] != '\0'; index++)
    {
        ScibRegs.SCITXBUF.bit.TXDT = protocol_message[index];
    }

    while(ScibRegs.SCIFFTX.bit.TXFFST != 0);

    ScibRegs.SCIFFTX.bit.TXFIFORESET = 0;
}

void Serial_ReceiveMessage(void)
{
    if(ScibRegs.SCIFFRX.bit.RXFFST>=1)
    {
        CommunicationCommand command;
        int grupo;
        char buffer = ScibRegs.SCIRXBUF.bit.SAR;

        switch(buffer)
        {
        case 'D':
            grupo_dac = (grupo_dac+1)%DAC_CHANNEL_COUNT;
            command = IncreaseDacGroup;
            break;
        case 'd':
            grupo = grupo_dac - 1;
            if(grupo < 0) grupo = grupo + DAC_CHANNEL_COUNT;
            grupo_dac = grupo;
            command = DecreaseDacGroup;
            break;
        default:
            command = None;
            break;
        }

        IPC_CommandBuffer[IPC_BufferIndex++] = command;

        if (IPC_BufferIndex >= IPC_COMMAND_BUFFER_SIZE)
            IPC_BufferIndex -= IPC_COMMAND_BUFFER_SIZE;

        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;   // reset RX-FIFO pointer
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   // enable RX-operation
        ScibRegs.SCIRXST.bit.RXERROR = 1;
    }
}
