
#include "F28x_Project.h"
#include "F2837xD_device.h"        // F2837xD Headerfile Include File

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


__interrupt void cpu_timer1_isr(void);

volatile static bool main_loop_wait;

//
// Global Variables
//
Vector *X, *Xe;
double *u;
double Vref;

void main(void)
{
    Matrix *P;
    int k;
    System* sys;

    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    //
    InitSysCtrl();

    //
    // Set the GPIO to it's default state.
    //
    InitGpio();


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

    //
    // Control Parameters and variables
    //
    sys = Buck::GetSys();

    Equilibrium::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Xe = Equilibrium::GetReference();

    P = Controller::GetP();

    /* =============== */
    Vref = 3;
    Xe->data[0] = 1.0331;
    Xe->data[1] = 100;
    /* =============== */

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
    Timer::ReferenceUpdate_Start();

    while(1)
    {
        main_loop_wait = true;

        Sensor::UpdateInput();
        Sensor::UpdateState();


        k = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u);


        Switch::SetState(k);

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

    Equilibrium::UpdateReference(Vref, X, *u);
}

