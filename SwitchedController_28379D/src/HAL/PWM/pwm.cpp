#include <src/HAL/PWM/pwm.h>

namespace PWM
{
    //
    // Configure - Configure EPWM SOC and compare values
    //
    void Configure(void)
    {
        EALLOW;
        //
        // Assumes ePWM clock is already enabled
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
        EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
        EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
        EPwm1Regs.CMPA.bit.CMPA = 0x0800;   // Set compare A value to 2048 counts
        EPwm1Regs.TBPRD = 0x1000;           // Set period to 4096 counts
        EPwm1Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
        EDIS;
    }


    //
    // Start - Start ePWM
    //
    void Start(void)
    {
        //
        // sync ePWM
        //
        EALLOW;
        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

        //
        // start ePWM
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
        EDIS;
    }
}
