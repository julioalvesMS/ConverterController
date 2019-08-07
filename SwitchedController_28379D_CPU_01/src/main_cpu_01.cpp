#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"

#include <src/settings_cpu_01.h>
#include <src/HAL/ADC/adc.h>
#include <src/HAL/DAC/dac.h>
#include <src/HAL/Timer/timer.h>
#include <src/Core/Sensor/sensor.h>
#include <src/Core/Equilibrium/reference_update.h>

void DebugVariables(void);
void ConfigureCPU02(void);
__interrupt void Interruption_MainLoopPeriod(void);
__interrupt void Interruption_ReferenceUpdate(void);

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
static double *Vout_mean;

static double Vref;

//
// Debug Variables
//
__attribute__((unused)) static int BestSubsystem;
__attribute__((unused)) static double *Vin, *Vout, *IL;



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

    //
    // Initializations to enable CPUs communication
    //
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
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    EALLOW;
    PieVectTable.ADCA1_INT = &(ADC_HAL::Interruption);
    PieVectTable.TIMER2_INT = &Interruption_ReferenceUpdate;
    EDIS;

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;
    IER |= M_INT14;

    DAC_HAL::Configure();

    //
    // Specific devices initializations
    //
    Sensor::Configure();
    Timer::Configure();
    Equilibrium::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Vout_mean = Sensor::GetOutput();
    Xe = Equilibrium::GetReference();

    ConfigureCPU02();

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    DebugVariables();

    /* =============== */
    Vref = 2;
    Xe[0] = 0.5333;
    Xe[1] = 4.0000;
    /* =============== */

#if REFERENCE_UPDATE_ENABLED
    Timer::ReferenceUpdate_Start();
#endif

    Sensor::Start();

    while(1)
    {
        // If there is no pending flag, transfer data between the CPUs
        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
        {
            //
            // Read data from CPU 2
            //
            BestSubsystem = shared_BestSubsystem;

            //
            // Write data to CPU 2
            //
            shared_X[0] = X[0];
            shared_X[1] = X[1];
            shared_Xe[0] = Xe[0];
            shared_Xe[1] = Xe[1];
            shared_u = *u;

            //
            // Set a flag to notify CPU02 that data is available
            //
            IPCLtoRFlagSet(IPC_FLAG10);
        }
    }
}


void ConfigureCPU02(void)
{

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
    // Configure GPIOS used in CPU2
    //
    GPIO_SetupPinMux(GPIO_S1, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinMux(GPIO_S2, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinMux(GPIO_A1, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(GPIO_S1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(GPIO_S2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinOptions(GPIO_A1, GPIO_OUTPUT, GPIO_PUSHPULL);
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
__interrupt void Interruption_ReferenceUpdate(void)
{
    CpuTimer2.InterruptCount++;

    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);
}

