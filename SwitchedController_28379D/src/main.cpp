
#include "F28x_Project.h"

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

void init(void);
void initInterruption(void);


__interrupt void cpu_timer1_isr(void);

static int main_loop_wait = 0;

void main(void)
{
    Vector *X, *Xe;
    Matrix *P;
    double *u;
    int k;
    double Vref = 60;
    System* sys;



    //
    // General Board initializations
    //
    init();

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    initInterruption();

    //
    // Specific devices initializations
    //
    Timer::Configure();
    Sensor::Configure();
    Switch::Configure();


    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Control Parameters and variables
    //
    sys = Buck::getSys();

    Equilibrium::init();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Xe = Equilibrium::getReference();

    P = Controller::getP();

    /* =============== */
    *u = 400;
    Vref = 100;
    Xe->data[0] = 1.0331;
    Xe->data[1] = 100;
    /* =============== */


    Timer::MainLoop_Start();
    while(1)
    {
        main_loop_wait = 1;

//        Equilibrium::referenceUpdate(Vref, X, *u);
        Sensor::UpdateInput();
        Sensor::UpdateState();

        k = SwitchingRule2::switchingRule(sys, P, X, Xe, *u);


        Switch::SetState(k);

        while(main_loop_wait){
            if (Vref==0)
                main_loop_wait = 1;
        }
    }
}


void init(void)
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

    InitCpuTimers();

}

void initInterruption(void)
{

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
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1; //Enable group 1 interrupts
    IER |= M_INT13;
    IER |= M_INT14;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
}


//
// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter
//
__interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;


   main_loop_wait = 0;
}
