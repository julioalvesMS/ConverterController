
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

#include <src/Core/SwitchedSystem/switched_system.h>
#include <src/Core/SwitchingRule/rule2.h>
#include <src/Core/Converter/buck.h>
#include <src/Core/Controller/switched_controller.h>

using namespace SwitchedSystem;

//
// Global Variables
//
#pragma DATA_SECTION("SHARERAMGS0");
volatile static double shared_X[SYSTEM_ORDER];
#pragma DATA_SECTION("SHARERAMGS0");
volatile static double shared_Xe[SYSTEM_ORDER];

#pragma DATA_SECTION("SHARERAMGS0");
volatile static double shared_u;

#pragma DATA_SECTION("SHARERAMGS1");
volatile static int shared_BestSubsystem;

static double X[SYSTEM_ORDER], Xe[SYSTEM_ORDER];
static double u;
static int BestSubsystem;


void main(void)
{
    double P[SYSTEM_ORDER][SYSTEM_ORDER];
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

    Controller::GetP(P);


    //
    // Wait until Shared RAM is available.
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1));

//    EINT;  // Enable Global interrupt INTM
//    ERTM;  // Enable Global realtime interrupt DBGM

    while(1)
    {
        BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, u);

        if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
        {
            //
            // Read data from CPU 1
            //
            X[0] = shared_X[0];
            X[1] = shared_X[1];

            Xe[0] = shared_Xe[0];
            Xe[1] = shared_Xe[1];

            u = shared_u;

            //
            // Write data to CPU 1
            //
            shared_BestSubsystem = BestSubsystem;
//            shared_BestSubsystem = !shared_BestSubsystem;

            IPCRtoLFlagAcknowledge (IPC_FLAG10);
        }
    }
}
