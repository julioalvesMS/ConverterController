#include <src/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        InitCpuTimers();

        #if (CPU_FRQ_150MHZ)
        ConfigCpuTimer(&CpuTimer2, 150, REFERENCE_CONTROLLER_PERIOD);
        #endif

        #if (CPU_FRQ_100MHZ)
        ConfigCpuTimer(&CpuTimer2, 100, REFERENCE_CONTROLLER_PERIOD);
        #endif

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
