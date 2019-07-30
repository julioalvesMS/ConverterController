
#include "F28x_Project.h"

#include <src/HAL/Timer/timer.h>
#include <src/HAL/ADC/adc.h>
#include <src/Core/Switch/switch.h>
#include <src/Core/Sensor/sensor.h>
#include <src/Core/Controller/pid.h>

void DebugVariables(void);

__interrupt void Interruption_MainLoopPeriod(void);
__interrupt void Interruption_ADC(void);
__interrupt void Interruption_ReferenceUpdate(void);

volatile static bool main_loop_wait;

//
// Global Variables
//

static double *X;
static double *u;

static double *Vin, *Vout, *IL;
static double Vref;
static double DutyCycle;


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

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &(Interruption_MainLoopPeriod);
    PieVectTable.ADCA1_INT = &(Interruption_ADC);
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1; //Enable group 1 interrupts


    //
    // Specific devices initializations
    //
    Sensor::Configure();
    Switch::Configure();

    Timer::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();

    /* =============== */
    Vref = 2;
    /* =============== */

    DebugVariables();

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    Sensor::Start();
    Timer::MainLoop_Start();

    //
    // sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    while(1)
    {
        main_loop_wait = true;

        while(main_loop_wait);
    }
}

void DebugVariables(void)
{
    Vin = u;
    Vout = X+1;
    IL = X;
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
// Interruption_ADC - CPU Timer1 ISR with interrupt counter
//
__interrupt void Interruption_ADC(void)
{
    ADC_HAL::ReadResult();

    DutyCycle = PID::Update(Vref, *Vout, *Vin);

    PWM::Switch_UpdateDutyCycle(DutyCycle);

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


