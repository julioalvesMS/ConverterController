#include <src/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        InitCpuTimers();

        #if (CPU_FRQ_150MHZ)
        ConfigCpuTimer(&CpuTimer1, 150, SYSTEM_EVALUATION_PERIOD);
        #endif

        #if (CPU_FRQ_200MHZ)
        ConfigCpuTimer(&CpuTimer1, 200, SYSTEM_EVALUATION_PERIOD);
        #endif

        CpuTimer1Regs.TCR.all = 0x4000;

        // Disable Timer
        CpuTimer1Regs.TCR.bit.TSS = 1;
    }

    void CommunicationTimer_Start()
    {
        //Enable Timer
        CpuTimer1Regs.TCR.bit.TSS = 0;
    }
}
