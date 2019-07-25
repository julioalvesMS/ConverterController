
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

#include <src/Util/Math/matrix.h>
#include <src/Core/SwitchedSystem/switched_system.h>
#include <src/Core/SwitchingRule/rule2.h>
#include <src/Core/Converter/buck.h>
#include <src/Core/Controller/switched_controller.h>

using namespace Math;
using namespace SwitchedSystem;

//
// Global Variables
//
#pragma DATA_SECTION("SHARERAMGS0");
static Vector *X, *Xe;

#pragma DATA_SECTION("SHARERAMGS0");
static double *u;

#pragma DATA_SECTION("SHARERAMGS1");
volatile static int BestSubsystem;


void main(void)
{
    Matrix *P;
    System* sys;

    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    //
    InitSysCtrl();

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Control Parameters and variables
    //
    sys = Buck::GetSys();

    P = Controller::GetP();


    //
    // Wait until Shared RAM is available.
    //
   while(!( MemCfgRegs.GSxMSEL.bit.MSEL_GS0 &
            MemCfgRegs.GSxMSEL.bit.MSEL_GS14 ));

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    while(1)
    {

        if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
        {
            BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u);

            IPCRtoLFlagAcknowledge (IPC_FLAG10);
         }
    }
}
