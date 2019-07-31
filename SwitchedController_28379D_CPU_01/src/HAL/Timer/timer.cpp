#include <src/HAL/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        InitCpuTimers();

        ConfigCpuTimer(&CpuTimer2, 200, REFERENCE_CONTROLLER_PERIOD);

        CpuTimer2Regs.TCR.all = 0x4000;

        // Disable Timer
        CpuTimer2Regs.TCR.bit.TSS = 1;
    }

    void ReferenceUpdate_Start()
    {
        //Enable Timer
        CpuTimer2Regs.TCR.bit.TSS = 0;
    }
}
