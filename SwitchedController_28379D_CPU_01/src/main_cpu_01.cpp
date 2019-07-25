
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

#include <src/Util/Math/matrix.h>
#include <src/Core/Switch/switch.h>
#include <src/Core/Sensor/sensor.h>
#include <src/Core/SwitchedSystem/switched_system.h>
#include <src/Core/Equilibrium/reference_update.h>
#include <src/Core/SwitchingRule/rule2.h>
#include <src/Core/Converter/buck.h>
#include <src/Core/Controller/switched_controller.h>

using namespace Math;
using namespace SwitchedSystem;


__interrupt void Interruption_MainLoopPeriod(void);
__interrupt void Interruption_ReferenceUpdate(void);

volatile static bool main_loop_wait;

//
// Global Variables
//
#pragma DATA_SECTION("SHARERAMGS0");
static Vector *X, *Xe;

#pragma DATA_SECTION("SHARERAMGS0");
static double *u;

#pragma DATA_SECTION("SHARERAMGS1");
volatile static int BestSubsystem;


static double Vref;


void main(void)
{
    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    //
    InitSysCtrl();

    //
    // Set the GPIO to it's default state.
    //
    InitGpio();

    InitIpc();


    InitCpuTimers();

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
    // enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //
    // Specific devices initializations
    //
    Sensor::Configure();
    Switch::Configure();

    Timer::Configure();


    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &Interruption_MainLoopPeriod;
    PieVectTable.TIMER2_INT = &Interruption_ReferenceUpdate;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    Equilibrium::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Xe = Equilibrium::GetReference();

    /* =============== */
    Vref = 5;
    Xe->data[0] = 0.1143;
    Xe->data[1] = 5.0000;
    /* =============== */


    //
    // Give Memory Access to GS0/ GS1 SARAM to CPU02
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS0 &
             MemCfgRegs.GSxMSEL.bit.MSEL_GS14))
    {
        EALLOW;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;
        EDIS;
    }

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1; //Enable group 1 interrupts
    IER |= M_INT13;
    IER |= M_INT14;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    Sensor::Start();
    Timer::MainLoop_Start();
//    Timer::ReferenceUpdate_Start();

    while(1)
    {

        //
        // If there is no pending flag
        //
        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
        {
            //
            // Set a flag to notify CPU02 that data is available
            //
            IPCLtoRFlagSet(IPC_FLAG10);
        }

        main_loop_wait = true;

        Switch::SetState(BestSubsystem);

        while(main_loop_wait);
    }
}

//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_MainLoopPeriod(void)
{
   CpuTimer0.InterruptCount++;

   main_loop_wait = false;

   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_ReferenceUpdate(void)
{
    CpuTimer2.InterruptCount++;

//    Equilibrium::UpdateReference(Vref, X, *u);
}

