#include <src/Serial/communication.h>

extern double *Vin, *Vout, *IL, *Iout;
extern double Vref;
extern Protection::Problem protection;
extern bool ConverterEnabled;
extern DAC_SPI::Channel DacChannel;

namespace Communication
{
    void Configure(void)
    {
        SCIA_init();        // Initializa e configura  SCIA- RS232
    }

    void GenerateProtocol()
    {
        switch(data_index)
        {
        case 0:
            sprintf(protocol_message, "U%02d", (int) (*Vin));
            break;
        case 1:
            sprintf(protocol_message, "Y%03d", (int) (10*(*Vout)));
            break;
        case 2:
            sprintf(protocol_message, "R%03d", (int) (10*Vref));
            break;
        case 3:
            sprintf(protocol_message, "I%02d", (int) (100*(*Iout)));
            break;
        case 4:
            sprintf(protocol_message, "L%04d", (int) (100*(*IL)));
            break;
        case 5:
            sprintf(protocol_message, "#%1d", (int) ConverterEnabled);
            break;
        case 6:
            sprintf(protocol_message, "D%1d", (int) DacChannel);
            break;
        case 7:
            sprintf(protocol_message, "$%1d", (int) protection);
            break;
        default:
            break;
        }

        data_index = (data_index+1)%8;
    }

    void SendMessage(void)
    {
        int index;

        GenerateProtocol();

        SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;

        for(index = 0; protocol_message[index] != '\0'; index++)
        {
            SciaRegs.SCITXBUF = protocol_message[index];
        }

        while(SciaRegs.SCIFFTX.bit.TXFFST != 0);

        SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 0;
    }

    void ReceiveMessage(void)
    {
        if(SciaRegs.SCIFFRX.bit.RXFFST>=1)
        {
            int channel;

            char buffer = SciaRegs.SCIRXBUF.bit.RXDT;

            switch(buffer)
            {
            case 'E':
                ConverterEnabled = true;
                break;
            case 'e':
                ConverterEnabled = false;
                break;
            case 'D':
                DacChannel = (DAC_SPI::Channel) ((((int) DacChannel)+1)%DAC_CHANNEL_COUNT);
                break;
            case 'd':
                channel = ((int) DacChannel) - 1;
                if(channel < 0) channel = channel + DAC_CHANNEL_COUNT;
                DacChannel = (DAC_SPI::Channel) channel;
                break;
            case 'R':
                Vref += 0.01;
                break;
            case 'r':
                Vref -= 0.01;
                if(Vref < 0) Vref = 0;
                break;
            case 'S':
                Vref += 10;
                break;
            case 's':
                Vref -= 10;
                if(Vref < 0) Vref = 0;
                break;
            case 'P':
                protection = Protection::FAULT_EMERGENCY_STOP;
                break;
            case 'p':
                protection = Protection::NONE;
                break;
            default:
                break;
            }

            SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;   // reset RX-FIFO pointer
            SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;   // enable RX-operation
            SciaRegs.SCIRXST.bit.RXERROR = 1;
        }
    }
}
