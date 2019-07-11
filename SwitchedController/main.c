#include "DSP28x_Project.h"


void main(void)
{
    Uint32 delay;
    InitSysCtrl();

    //
    // Configure GPIO34 as a GPIO output pin
    //
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    EDIS;


    while(1)
    {
        //
        // Toggle LED
        //
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

        //
        // Delay for a bit
        //
        for(delay = 0; delay < 2000000; delay++)
        {
        }

        //
        // Toggle LED
        //
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

        //
        // Delay for a bit
        //
        for(delay = 0; delay < 2000000; delay++)
        {
        }
    }
}
