#include <src/Timer/timer.h>

namespace Timer
{
    void Configure()
    {
        InitCpuTimers();

        #if (CPU_FRQ_150MHZ)
        ConfigCpuTimer(&CpuTimer1, 150, RS232_SEND_PERIOD);
        ConfigCpuTimer(&CpuTimer2, 150, REFERENCE_CONTROLLER_PERIOD);
        #endif

        #if (CPU_FRQ_100MHZ)
        ConfigCpuTimer(&CpuTimer1, 100, RS232_SEND_PERIOD);
        ConfigCpuTimer(&CpuTimer2, 100, REFERENCE_CONTROLLER_PERIOD);
        #endif

        CpuTimer1Regs.TCR.all = 0x4000;
        CpuTimer2Regs.TCR.all = 0x4000;

        // Disable Timer
        CpuTimer1Regs.TCR.bit.TSS = 1;
        CpuTimer2Regs.TCR.bit.TSS = 1;
    }

    void Communication_Start()
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
