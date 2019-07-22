#include <src/HAL/WatchdogTimer/watchDog.h>

namespace WatchDog
{

    void enable()
    {
        //Enable Watchdog timer
//        EALLOW;
//        SysCtrlRegs.WDCR = 0x00AF;
//        EDIS;
    }

    void step1()
    {
//        EALLOW;
//        SysCtrlRegs.WDKEY = 0x55;
//        EDIS;
    }

    void step2()
    {
//        EALLOW;
//        SysCtrlRegs.WDKEY = 0xAA;
//        EDIS;
    }
}

