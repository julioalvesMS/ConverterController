#include "DSP2833x_Device.h"

namespace Switch
{

    static short s_switch_state = -1;

    void init()
    {
        //
        // Configure switches GPIO
        //
        EALLOW;

        // S1
        GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
        GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;

        // S2
        GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
        GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;

        EDIS;

    }

    void updateState(short state)
    {
        //
        // If the switch is already ate the desired state, do nothing
        //
        if (state == s_switch_state)
            return;

        //
        // Toggle switch and register new state
        //
        s_switch_state = state;

        switch(state)
        {
        case 0:
            GpioDataRegs.GPASET.bit.GPIO9 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
            break;
        case 1:
            GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
            GpioDataRegs.GPASET.bit.GPIO11 = 1;
            break;
        default:
            GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
            break;
        }
    }
}
