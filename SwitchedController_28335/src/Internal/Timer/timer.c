#include "DSP2833x_Device.h"

void timer_Init(long period)
{
    // Configure timer 100ms interruption
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 150, period);
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER |= 1;

    EINT;
    ERTM;
}

void timer_Start()
{
    CpuTimer0Regs.TCR.bit.TSS = 0;
}
