#include <src/HAL/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        // Configure timer 100ms interruption
        InitCpuTimers();

        ConfigCpuTimer(&CpuTimer1, 200, 100000);
        ConfigCpuTimer(&CpuTimer2, 200, 30000);

        CpuTimer1Regs.TCR.all = 0x4000;
        CpuTimer2Regs.TCR.all = 0x4000;

        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    }

    void MainLoop_Start()
    {
        CpuTimer1Regs.TCR.bit.TSS = 0;
    }

    void Switch_Start()
    {
        CpuTimer2Regs.TCR.bit.TSS = 0;
    }

    void Switch_Stop()
    {
        CpuTimer2Regs.TCR.bit.TSS = 1;
    }
}
