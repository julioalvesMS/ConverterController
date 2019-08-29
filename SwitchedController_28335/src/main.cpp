
//
// Includes
//
#include "DSP28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/CONFIG_GPIO_V4_CCARDV2.h>
#include <src/Config/DEFINESCONTROLCARDV2.h>
#include <src/Controller/switched_controller.h>
#include <src/Converter/buck.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Sensor/sensor.h>
#include <src/Switch/switch.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/SwitchingRule/rule2.h>
#include <src/Timer/timer.h>

//
// Namespaces in use
//
using namespace SwitchedSystem;


//
// Functions in main
//
void DebugVariables(void);

//
// Interruptions defined in main
//
__interrupt void Interruption_ReferenceUpdate(void);

//
// Global static variables
//
static double *X, *Xe;
static double *u;
static double *Vout_mean;
static double Vref = INITIAL_REFERENCE_VOLTAGE;
static int BestSubsystem;

//
// Variables used during debug
//
__attribute__((unused)) static double *Vin, *Vout, *IL;

void main(void)
{
    double P[SYSTEM_ORDER][SYSTEM_ORDER];
    System* sys;

    InitSysCtrl();                              // Basic Core Init from DSP2833x_SysCtrl.c
    Gpio_setup1();                              // Configura os pinos de GPIO de acordo com a ligaçaõ na placa
    DINT;                                       // Disable all interrupts

    InitPieCtrl();                              // basic setup of PIE table

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();                         // default ISR's in PIE

    EALLOW;
    PieVectTable.ADCINT = &(Sensor::Interruption);
    PieVectTable.TINT2 = &(Interruption_ReferenceUpdate);
    EDIS;

    IER |= M_INT1;      // Enable CPU Interrupt 1
    IER |= M_INT2;      // Enable CPU Interrupt 1
    IER |= M_INT14;     // Timer02

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  // ADC
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer

    Timer::Configure();
    Sensor::Configure();
    Switch::Configure();
    Equilibrium::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Vout_mean = Sensor::GetOutput();

    Xe = Equilibrium::GetReference();
    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);

    sys = Buck::GetSys();
    Controller::GetP(P);

    DebugVariables();

    Timer::ReferenceUpdate_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    while(1)
    {
#if DAC_ENABLED
        enviar_dac_spi_uni(0, Sensor::ADC_RESULT_IL);
        enviar_dac_spi_uni(1, Sensor::ADC_RESULT_VOUT);
        enviar_dac_spi_uni(2, Sensor::ADC_RESULT_VIN);
#endif

#if SWITCHING_RULE == 1
        BestSubsystem = SwitchingRule1::SwitchingRule(sys, P, X, Xe, *u);
#elif SWITCHING_RULE == 2
        BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u);
#endif

        Switch::SetState(BestSubsystem);
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
__interrupt void Interruption_ReferenceUpdate(void)
{
    CpuTimer2.InterruptCount++;

    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);
}
