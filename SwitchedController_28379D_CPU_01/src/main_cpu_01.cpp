
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

#include <src/Core/Switch/switch.h>
#include <src/Core/Sensor/sensor.h>
#include <src/Core/Equilibrium/reference_update.h>


__interrupt void Interruption_MainLoopPeriod(void);
__interrupt void Interruption_ReferenceUpdate(void);

volatile static bool main_loop_wait;

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

static double *X, *Xe;
static double *u;
static int BestSubsystem;


static double Vref;


void main(void)
{
    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    //
    InitSysCtrl();
    InitSysPll(XTAL_OSC,IMULT_40,FMULT_0,PLLCLK_BY_2);

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
    Vref = 4;
    Xe[0] = 0.5333;
    Xe[1] = 4.0000;
    /* =============== */


    //
    // Give Memory Access to GS1 SARAM to CPU02
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1))
    {
        EALLOW;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 1;
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

        main_loop_wait = true;

        //
        // If there is no pending flag
        //
        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
        {
            //
            // Read data from CPU 2
            //
            BestSubsystem = shared_BestSubsystem;

            //
            // Write data to CPU 2
            //
            shared_X[0] = READ_IL(X[0]);
            shared_X[1] = READ_VOUT(X[1]);

            shared_Xe[0] = Xe[0];
            shared_Xe[1] = Xe[1];

            shared_u = READ_VIN(*u);

            //
            // Set a flag to notify CPU02 that data is available
            //
            IPCLtoRFlagSet(IPC_FLAG10);
        }
//        BestSubsystem = !BestSubsystem;

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

