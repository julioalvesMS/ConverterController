#include <src/Switch/switch.h>

namespace Switch
{

    static short s_switch_state = -1;

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

    void SetState(int state)
    {
        // Test GPIO. Used to enable frequency measurement
        GpioDataRegs.GPATOGGLE.bit.MF = 1;

        // If the switch is already ate the desired state, do nothing
        if (state == s_switch_state)
            return;

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
        default:
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            break;
        }

        // Register new state
        s_switch_state = state;

    }
}
