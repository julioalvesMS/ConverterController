#include <src/HAL/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        // Configure timer 100ms interruption
        InitCpuTimers();

        ConfigCpuTimer(&CpuTimer0, 200, MAIN_PERIOD);
        ConfigCpuTimer(&CpuTimer1, 200, SWITCH_ON_DELAY);
        ConfigCpuTimer(&CpuTimer2, 200, REFERENCE_CONTROLLER_PERIOD);

        CpuTimer0Regs.TCR.all = 0x4000;
        CpuTimer1Regs.TCR.all = 0x4000;
        CpuTimer2Regs.TCR.all = 0x4000;

        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    }

    void MainLoop_Start()
    {
        CpuTimer0Regs.TCR.bit.TSS = 0;
    }

    void Switch_Start()
    {
        CpuTimer1Regs.TCR.bit.TSS = 0;
    }

    void Switch_Stop()
    {
        CpuTimer1Regs.TCR.bit.TSS = 1;
    }

    void ReferenceUpdate_Start()
    {
        CpuTimer2Regs.TCR.bit.TSS = 0;
    }
}
