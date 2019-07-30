#include <src/HAL/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        // Configure timer 100ms interruption
        InitCpuTimers();

        ConfigCpuTimer(&CpuTimer0, 200, MAIN_PERIOD);

        CpuTimer0Regs.TCR.all = 0x4000;

        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    }

    void MainLoop_Start()
    {
        CpuTimer0Regs.TCR.bit.TSS = 0;
    }
}
