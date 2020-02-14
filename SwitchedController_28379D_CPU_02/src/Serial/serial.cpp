#include <src/Serial/serial.h>

extern double Vin, Vout, IL, Iout;
extern double Vref;
extern double loadResistance;
extern int SwitchingFrequency;
extern bool ConverterEnabled;
extern int CorrectionMethod;
extern bool OutputLoadStep;
extern double stateDutyCycle[4];

//
// Enums
//
extern int protection;
extern int DacChannel;
extern int activeConverter;
extern int controlStrategy;

extern Protocol::CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;

namespace Serial
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
            sprintf(protocol_message, "U%02d", (int) (Vin));
            break;
        case 1:
            sprintf(protocol_message, "Y%04d", (int) (10*Vout));
            break;
        case 2:
            sprintf(protocol_message, "R%04d", (int) (10*Vref));
            break;
        case 3:
            sprintf(protocol_message, "I%03d", (int) (100*Iout));
            break;
        case 4:
            sprintf(protocol_message, "L%04d", (int) (100*IL));
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
        case 10:
            sprintf(protocol_message, "&%1d", (int) controlStrategy);
            break;
        case 11:
            sprintf(protocol_message, "E%1d", (int) CorrectionMethod);
            break;
        case 12:
            sprintf(protocol_message, "C%1d", (int) OutputLoadStep);
            break;
        case 13:
            sprintf(protocol_message, "Z%04d", (int) loadResistance*10);
            break;
        case 14:
            sprintf(protocol_message, "0%04d", (int) (stateDutyCycle[0]*10));
            break;
        case 15:
            sprintf(protocol_message, "1%04d", (int) (stateDutyCycle[1]*10));
            break;
        case 16:
            sprintf(protocol_message, "2%04d", (int) (stateDutyCycle[2]*10));
            break;
        case 17:
            sprintf(protocol_message, "3%04d", (int) (stateDutyCycle[3]*10));
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
            Protocol::CommunicationCommand command;
            int channel;
            char buffer = ScibRegs.SCIRXBUF.bit.SAR;

            switch(buffer)
            {
            case 'E':
                command = Protocol::EnableOperation;
                break;
            case 'e':
                command = Protocol::DisableOperation;
                break;
            case 'D':
                DacChannel = (DAC_SPI::Channel) ((((int) DacChannel)+1)%DAC_CHANNEL_COUNT);
                command = Protocol::IncreaseDacChannel;
                break;
            case 'd':
                channel = ((int) DacChannel) - 1;
                if(channel < 0) channel = channel + DAC_CHANNEL_COUNT;
                DacChannel = (DAC_SPI::Channel) channel;
                command = Protocol::DecreaseDacChannel;
                break;
            case 'R':
                command = Protocol::RampIncreaseReference;
                break;
            case 'r':
                command = Protocol::RampDecreaseReference;
                break;
            case 'S':
                command = Protocol::StepIncreaseReference;
                break;
            case 's':
                command = Protocol::StepDecreaseReference;
                break;
            case 'P':
                command = Protocol::EmergencyButtonProtection;
                break;
            case 'p':
                command = Protocol::ResetProtection;
                break;

            case '0':
                command = Protocol::ConverterBuck;
                break;
            case '1':
                command = Protocol::ConverterBoost;
                break;
            case '2':
                command = Protocol::ConverterBuckBoost;
                break;
            case '3':
                command = Protocol::ConverterBuckBoost3;
                break;

            case '!':
                command = Protocol::ControllerClassic;
                break;
            case '@':
                command = Protocol::ControllerContinuous1;
                break;
            case '#':
                command = Protocol::ControllerContinuous2;
                break;
            case '$':
                command = Protocol::ControllerDiscrete1;
                break;
            case '%':
                command = Protocol::ControllerClassicVC;
                break;
            case '¨':
                command = Protocol::ControllerLimitCycleCost;
                break;
            case '&':
                command = Protocol::ControllerLimitCycleH2;
                break;
            case '*':
                command = Protocol::ControllerLimitCycleHinf;
                break;
            case '(':
                command = Protocol::ControllerStateFeedbackH2;
                break;

            case ',':
                command = Protocol::EquilibriumNone;
                break;
            case '.':
                command = Protocol::EquilibriumReferenceController;
                break;
            case ';':
                command = Protocol::EquilibriumPartialInformation;
                break;
            case '<':
                command = Protocol::EquilibriumCurrentCorrection;
                break;

            case 'L':
                command = Protocol::EngageParallelLoad;
                break;
            case 'l':
                command = Protocol::DisengageParallelLoad;
                break;
            default:
                command = Protocol::None;
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
}
