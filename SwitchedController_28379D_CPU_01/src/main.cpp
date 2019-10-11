
//
// Includes
//
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/CONFIG_GPIO_V1_LP28379D.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Controller/switched_controller.h>
#include <src/Converter/buck.h>
#include <src/DAC/dac.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Protection/protection.h>
#include <src/Sensor/sensor.h>
#include <src/Serial/communication.h>
#include <src/Switch/switch.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/SwitchingRule/discrete_rule1.h>
#include <src/SwitchingRule/rule1.h>
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
__interrupt void Interruption_SystemEvaluation(void);
__interrupt void Interruption_ReferenceUpdate(void);
__interrupt void Interruption_Sensor(void);

//
// Global static variables
//
double *X, *Xe;
double *u;
double *Vout_mean;
double Vref = INITIAL_REFERENCE_VOLTAGE;
int BestSubsystem;


double P[SYSTEM_ORDER][SYSTEM_ORDER];
System* sys;

Protection::Problem protection = Protection::NONE;
bool ConverterEnabled = false;

DAC_SPI::Channel DacChannel = DAC_SPI::CH_CONTROLE;

int InterruptionCounter = 0;
int iterations_since_switch = 0;
int SwitchCounter = 0;
int SwitchingFrequency, ADCFrequency;

bool OpenCommunication = false;

//
// Variables used during debug
//
double *Vin, *Vout, *IL, *Iout;

void main(void)
{
    int loop_count;

    InitSysCtrl();          // Inicializa��es b�sicas
    InitGpio();             // Inicializa��es padr�o para as GPIOs
    InitIpc();              // Inicializa��es para comunica��o entre as CPUs
    InitCpuTimers();        // Inicializa��es do timer 0, 1 e 2

    DINT;                   // Disable all interrupts

    Gpio_setup();           // Configura os pinos de GPIO de acordo com a liga�a� na placa


    InitPieCtrl();          // basic setup of PIE table

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();                         // default ISR's in PIE

    EALLOW;
    PieVectTable.ADCA1_INT = &(Interruption_Sensor);
    PieVectTable.TIMER1_INT = &(Interruption_SystemEvaluation);
    PieVectTable.TIMER2_INT = &(Interruption_ReferenceUpdate);
    EDIS;

    IER |= M_INT1;      // Enable CPU Interrupt 1
    IER |= M_INT2;      // Enable CPU Interrupt 1
    IER |= M_INT13;     // Timer01
    IER |= M_INT14;     // Timer02

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // ADC
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer

    Timer::Configure();
    Sensor::Configure();
    Switch::Configure();
    Equilibrium::Configure();
    Communication::Configure();
    DAC_SPI::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Vout_mean = Sensor::GetOutput();
    Iout = Sensor::GetOutputCurrent();

    Xe = Equilibrium::GetReference();
    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);

    sys = Buck::GetSys();
    Controller::GetP(P);

    DebugVariables();

    Timer::SystemEvaluation_Start();
    Timer::ReferenceUpdate_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM


    loop_count = 0;
    for(;;)
    {
        Communication::ReceiveMessage();

        //
        // Send to DAC
        //
        DAC_SPI::SendData(DacChannel);

        if(OpenCommunication)
        {
            loop_count++;
            //
            // Send to PC
            //
            Communication::SendMessage();

            if (loop_count >= MESSAGES_COUNT)
            {
                OpenCommunication = false;
                loop_count = 0;
            }
        }
    }
}


void DebugVariables(void)
{
    Vin = u;
    Vout = X+1;
    IL = X;
}


//
// Interrup��o executada a cada 100ms para medi��es temporais
//
__interrupt void Interruption_SystemEvaluation(void)
{
    CpuTimer1.InterruptCount++;

    // Medi��es v�o ser a cada 100ms
    SwitchingFrequency = (10*SwitchCounter);
    ADCFrequency = (10*InterruptionCounter);
    InterruptionCounter = 0;
    SwitchCounter = 0;

    OpenCommunication = true;
}

//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_ReferenceUpdate(void)
{
    CpuTimer2.InterruptCount++;

    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);
}

//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_Sensor(void)
{
    InterruptionCounter++;

    // Test GPIO. Used to enable frequency measurement
    GpioDataRegs.GPATOGGLE.bit.AF = 1;
    // Test GPIO. Used to enable frequency measurement
    GpioDataRegs.GPATOGGLE.bit.MF = 1;

    //
    // Read Sensors
    //
    Sensor::ReadADCResult();

    // If no protection was already enabled, check protections
    if (protection ==  Protection::NONE)
        protection = Protection::CheckProtections(*Vin, *Vout, *IL, *Iout);

    // If necessary, protect system
    if(protection != Protection::NONE || !ConverterEnabled)
    {
        Protection::ProtectSystem();
    }
    else
    {

        //
        // Run Switching Rule
        //
#if SWITCHING_RULE == CONTINUOUS_RULE_1
        BestSubsystem = SwitchingRule1::SwitchingRule(sys, P, X, Xe, *u);
#elif SWITCHING_RULE == CONTINUOUS_RULE_2
        BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u);
#elif SWITCHING_RULE == DISCRETE_RULE_1
        BestSubsystem = DiscreteSwitchingRule1::SwitchingRule(sys, P, X, Xe, *u);
#endif

        if(iterations_since_switch < 5)
            iterations_since_switch++;
        else
        {
            //
            // Signal to the gate
            //
            SwitchCounter += Switch::SetState(BestSubsystem);

            iterations_since_switch = 0;
        }
    }


    //
    // Reinitialize for next ADC sequence
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    // Check if overflow has occurred
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    return;
}
