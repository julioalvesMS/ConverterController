#include <src/Serial/communication.h>

extern double *Vin, *Vout, *IL, *Iout;
extern double Vref;
extern int SwitchingFrequency;
extern Protection::Problem protection;
extern bool ConverterEnabled;
extern DAC_SPI::Channel DacChannel;
extern ConverterID activeConverter;
extern bool CapacitorPreLoad;


namespace Communication
{
    void Configure(void)
    {
        SCIB_init();        // Initializa e configura  SCIA- RS232
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
        case 8:
            sprintf(protocol_message, "F%03d", (int) (SwitchingFrequency/100));
            break;
        case 9:
            sprintf(protocol_message, "@%1d", (int) activeConverter);
            break;
        default:
            break;
        }

        data_index = (data_index+1)%MESSAGES_COUNT;
    }

    void SendMessage(void)
    {
        int index;

        GenerateProtocol();

        ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;

        for(index = 0; protocol_message[index] != '\0'; index++)
        {
            ScibRegs.SCITXBUF.bit.TXDT = protocol_message[index];
        }

        while(ScibRegs.SCIFFTX.bit.TXFFST != 0);

        ScibRegs.SCIFFTX.bit.TXFIFORESET = 0;
    }

    void ReceiveMessage(void)
    {
        if(ScibRegs.SCIFFRX.bit.RXFFST>=1)
        {
            int channel;

            char buffer = ScibRegs.SCIRXBUF.bit.SAR;

            switch(buffer)
            {
            case 'E':
                Manager::EnableOperation();
                break;
            case 'e':
                Manager::DisableOperation();
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
                Vref += 0.002;
                if(Vref > PROTECTION_VOUT_MAX) Vref = PROTECTION_VOUT_MAX;
                break;
            case 'r':
                Vref -= 0.002;
                if(Vref < 0) Vref = 0;
                break;
            case 'S':
                Vref += 10;
                if(Vref > PROTECTION_VOUT_MAX) Vref = PROTECTION_VOUT_MAX;
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
            case '0':
                Manager::ChangeConverter(ID_Buck);
                break;
            case '1':
                Manager::ChangeConverter(ID_Boost);
                break;
            case '2':
                Manager::ChangeConverter(ID_BuckBoost);
                break;
            default:
                break;
            }

            ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;   // reset RX-FIFO pointer
            ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   // enable RX-operation
            ScibRegs.SCIRXST.bit.RXERROR = 1;
        }
    }
}
