#include "DSP2833x_Device.h"

void watchDog_Init()
{
    //Enable Watchdog timer
    EALLOW;
    SysCtrlRegs.WDCR = 0x00AF;
    EDIS;
}

void watchDog_Step1()
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x55;
    EDIS;
}

void watchDog_Step2()
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}

