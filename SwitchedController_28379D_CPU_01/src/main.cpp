
//
// Includes
//
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/CONFIG_GPIO_V1_LP28379D.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Converter/base_converter.h>
#include <src/Converter/boost.h>
#include <src/Converter/buck.h>
#include <src/Converter/buck_boost.h>
#include <src/DAC/dac.h>
#include <src/Equilibrium/reference_update.h>
#include <src/OperationManagement/manager.h>
#include <src/Protection/protection.h>
#include <src/Relay/relay.h>
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
using namespace BaseConverter;
using namespace ConverterBuck;
using namespace ConverterBoost;
using namespace ConverterBuckBoost;


//
// Functions in main
//
void DebugVariables(void);
void LoadConverterController(void);

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
int SwitchState;


double P[SYSTEM_ORDER][SYSTEM_ORDER];
double h[SYSTEM_ORDER];
double d;
System* sys;
System* dsys;
ConverterID activeConverter = ID_BuckBoost;

Protection::Problem protection = Protection::NONE;
bool ConverterEnabled = false;
Manager::OperationState *CurrentOperationState;

DAC_SPI::Channel DacChannel = DAC_SPI::CH_CONTROLE;

int InterruptionCounter = 0;
int iterations_since_switch = 0;
int SwitchCounter = 0;
int SwitchingFrequency, ADCFrequency;

bool OpenCommunication = false;

bool CapacitorPreLoad;
bool CapacitorPreLoadEngaged = false;

//
// Variables used during debug
//
double *Vin, *Vout, *IL, *Iout;

void main(void)
{
    int loop_count;

    InitSysCtrl();          // Inicializações básicas
    InitGpio();             // Inicializações padrão para as GPIOs
    InitIpc();              // Inicializações para comunicação entre as CPUs
    InitCpuTimers();        // Inicializações do timer 0, 1 e 2

    DINT;                   // Disable all interrupts

    Gpio_setup();           // Configura os pinos de GPIO de acordo com a ligaçaõ na placa


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

    CurrentOperationState = Manager::GetCurrentState();

    LoadConverterController();

    DebugVariables();

    Timer::SystemEvaluation_Start();
    Timer::ReferenceUpdate_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM


    loop_count = 0;
    for(;;)
    {
        if (*CurrentOperationState == Manager::OS_CHANGING_CONVERTER)
        {
            LoadConverterController();
            Manager::CompleteConverterChange();
        }

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

void LoadConverterController(void)
{
    switch(activeConverter)
    {
    case ID_Buck:
        sys = Buck::GetSys();
        dsys = Buck::GetDiscreteSys();
        Buck::GetP(P);
        Buck::GetH(h);
        d = Buck::GetD(P,h);
        break;
    case ID_Boost:
        sys = Boost::GetSys();
        dsys = Boost::GetDiscreteSys();
        Boost::GetP(P);
        Boost::GetH(h);
        d = Boost::GetD(P,h);
        break;
    case ID_BuckBoost:
        sys = BuckBoost::GetSys();
        dsys = BuckBoost::GetDiscreteSys();
        BuckBoost::GetP(P);
        BuckBoost::GetH(h);
        d = BuckBoost::GetD(P,h);
        break;
    }
}

//
// Interrupção executada a cada 100ms para medições temporais
//
__interrupt void Interruption_SystemEvaluation(void)
{
    static int i = 0;
    CpuTimer1.InterruptCount++;

    // Medições vão ser a cada 100ms
    SwitchingFrequency = (10*SwitchCounter);
    ADCFrequency = (10*InterruptionCounter);
    InterruptionCounter = 0;
    SwitchCounter = 0;

    if(i%10==0)
        Manager::ContinuePreLoad();

    i++;
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
        BestSubsystem = DiscreteSwitchingRule1::SwitchingRule(dsys, P, h, d, X, Xe, *u);
#endif

        switch(activeConverter)
        {
        case ID_Buck:
            SwitchState = Buck::SubSystem2SwitchState(BestSubsystem);
            break;
        case ID_Boost:
            SwitchState = Boost::SubSystem2SwitchState(BestSubsystem);
            break;
        case ID_BuckBoost:
            SwitchState = BuckBoost::SubSystem2SwitchState(BestSubsystem);
            break;
        default:
            SwitchState = DISBALE_SWITCHES;
            break;
        }


        switch(*CurrentOperationState)
        {
        case Manager::OS_RUNNING:
            break;

        case Manager::OS_STARTING_PRE_LOAD:
            SwitchState = DISBALE_SWITCHES;
            break;

        case Manager::OS_PRE_LOAD:
        case Manager::OS_ENDING_PRE_LOAD:
            SwitchState = 0;
            break;

        case Manager::OS_OFF:
        default:
            SwitchState = DISBALE_SWITCHES;
            break;
        }

        if(iterations_since_switch < 10 && *CurrentOperationState == Manager::OS_RUNNING)
            iterations_since_switch++;
        else
        {
            //
            // Signal to the gate
            //
            SwitchCounter += Switch::SetState(SwitchState);

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
