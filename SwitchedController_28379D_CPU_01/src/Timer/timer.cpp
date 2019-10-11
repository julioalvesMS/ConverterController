#include <src/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        InitCpuTimers();

        #if (CPU_FRQ_150MHZ)
        ConfigCpuTimer(&CpuTimer1, 150, SYSTEM_EVALUATION_PERIOD);
        ConfigCpuTimer(&CpuTimer2, 150, REFERENCE_CONTROLLER_PERIOD);
        #endif

        #if (CPU_FRQ_200MHZ)
        ConfigCpuTimer(&CpuTimer1, 200, SYSTEM_EVALUATION_PERIOD);
        ConfigCpuTimer(&CpuTimer2, 200, REFERENCE_CONTROLLER_PERIOD);
        #endif

        CpuTimer1Regs.TCR.all = 0x4000;
        CpuTimer2Regs.TCR.all = 0x4000;

        // Disable Timer
        CpuTimer1Regs.TCR.bit.TSS = 1;
        CpuTimer2Regs.TCR.bit.TSS = 1;
    }

    void SystemEvaluation_Start()
    {
        //Enable Timer
        CpuTimer1Regs.TCR.bit.TSS = 0;
    }

    void ReferenceUpdate_Start()
    {
        //Enable Timer
        CpuTimer2Regs.TCR.bit.TSS = 0;
    }
}
