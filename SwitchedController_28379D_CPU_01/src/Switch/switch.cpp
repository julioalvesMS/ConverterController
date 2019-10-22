#include <src/Switch/switch.h>

namespace Switch
{

    static short s_switch_state = DISBALE_SWITCHES;

    void Configure()
    {
        EALLOW;

        // PULL DOWN
        GpioCtrlRegs.GPAPUD.bit.S1 = 1;
        GpioCtrlRegs.GPAPUD.bit.S2 = 1;
        GpioCtrlRegs.GPAPUD.bit.S3 = 1;
        GpioCtrlRegs.GPAPUD.bit.S4 = 1;

        // UTILIZAR PORTA COMO GPIO
        GpioCtrlRegs.GPAMUX1.bit.S1 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S2 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S3 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S4 = 0;

        // CONFIGURA COMO SAÍDA
        GpioCtrlRegs.GPADIR.bit.S1 = 1;
        GpioCtrlRegs.GPADIR.bit.S2 = 1;
        GpioCtrlRegs.GPADIR.bit.S3 = 1;
        GpioCtrlRegs.GPADIR.bit.S4 = 1;

        EDIS;

    }

    bool SetState(int state)
    {
        bool switched = false;

        // If the switch is already ate the desired state, do nothing
        if (state == s_switch_state && s_switch_state!=DISBALE_SWITCHES)
            return switched;

        switched = true;
        switch(state)
        {
        case 0:
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            GpioDataRegs.GPASET.bit.S1 = 1;
            GpioDataRegs.GPASET.bit.S4 = 1;
            break;
        case 1:
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            GpioDataRegs.GPASET.bit.S2 = 1;
            GpioDataRegs.GPASET.bit.S4 = 1;
            break;
        case 2:
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S4 = 1;
            GpioDataRegs.GPASET.bit.S1 = 1;
            GpioDataRegs.GPASET.bit.S3 = 1;
            break;
        case DISBALE_SWITCHES:
        default:
            switched = false;
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            GpioDataRegs.GPACLEAR.bit.S4 = 1;
            break;
        }

        // Register new state
        s_switch_state = state;

        return switched;
    }
}
