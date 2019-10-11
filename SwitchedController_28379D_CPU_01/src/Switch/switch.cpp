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
        GpioCtrlRegs.GPAPUD.bit.MF = 1;
        GpioCtrlRegs.GPAPUD.bit.AF = 1;

        // UTILIZAR PORTA COMO GPIO
        GpioCtrlRegs.GPAMUX1.bit.S1 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S2 = 0;
        GpioCtrlRegs.GPAMUX1.bit.MF = 0;
        GpioCtrlRegs.GPAMUX1.bit.AF = 0;

        // CONFIGURA COMO SAÍDA
        GpioCtrlRegs.GPADIR.bit.S1 = 1;
        GpioCtrlRegs.GPADIR.bit.S2 = 1;
        GpioCtrlRegs.GPADIR.bit.MF = 1;
        GpioCtrlRegs.GPADIR.bit.AF = 1;

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
            GpioDataRegs.GPASET.bit.S1 = 1;
            break;
        case 1:
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPASET.bit.S2 = 1;
            break;
        case DISBALE_SWITCHES:
        default:
            switched = false;
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            break;
        }

        // Register new state
        s_switch_state = state;

        return switched;
    }
}
