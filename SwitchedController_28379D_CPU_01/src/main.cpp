
//
// Includes
//
#include "F28x_Project.h"
#include <src/settings.h>
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/CONFIG_GPIO_V1_LP28379D.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Controller/SwitchingRule/discrete_rule1.h>
#include <src/Controller/SwitchingRule/rule1.h>
#include <src/Controller/SwitchingRule/rule2.h>
#include <src/Controller/controller.h>
#include <src/Controller/ClassicController/state_feedback_h2_controller.h>
#include <src/Controller/ClassicController/voltage_controller.h>
#include <src/Controller/ClassicController/voltage_current_controller.h>
#include <src/Controller/LimitCycle/cycle_rule_cost.h>
#include <src/Controller/LimitCycle/cycle_rule_hinf.h>
#include <src/Converter/base_converter.h>
#include <src/Converter/boost.h>
#include <src/Converter/buck.h>
#include <src/Converter/buck_boost.h>
#include <src/Converter/buck_boost_3.h>
#include <src/Equilibrium/equilibrium.h>
#include <src/Equilibrium/current_correction.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Equilibrium/partial_information.h>
#include <src/IPC/ipc.h>
#include <src/OperationManagement/manager.h>
#include <src/Protection/protection.h>
#include <src/Relay/relay.h>
#include <src/Sensor/sensor.h>
#include <src/Switch/switch.h>
#include <src/SwitchedSystem/cycle_sequence.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/Timer/timer.h>
#include <src/Filtros/filtro_1.c>

//
// Namespaces in use
//
using namespace SwitchedSystem;
using namespace CycleSequence;
using namespace BaseConverter;
using namespace ConverterBuck;
using namespace ConverterBoost;
using namespace ConverterBuckBoost;
using namespace ConverterBuckBoost3;
using namespace Controller;


extern float filtro_1(float vP);

//
// Functions in main
//
void ConfigureCPU02(void);
void LoadConverterController(void);
void SwitchedControl(void);
void ClassicControl(void);


//
// Interruptions defined in main
//
__interrupt void Interruption_SystemEvaluation(void);
__interrupt void Interruption_ReferenceUpdate(void);
__interrupt void Interruption_Sensor(void);


//
// IPC Communication
//
Protocol::CommunicationCommand IPC_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];
int IPC_BufferIndex = 0;


//
// Converter State Variables
//
double *X, *Xe, *Xe_o;
double *u;
double *Vout_mean;
double *loadResistance;
double Vref = INITIAL_REFERENCE_VOLTAGE;
double VrefH = Vref;


//
// Switched Control Variables
//
double P[SYSTEM_ORDER][SYSTEM_ORDER];
System* sys;
System* dsys;
Cycle* limitCycle;
int BestSubsystem;
int SwitchState;


//
// System Management and Mode of Operation
//
bool ConverterEnabled = false;
bool CapacitorPreLoad;
bool CapacitorPreLoadEngaged = false;
bool ModeHoppingEnabled = false;
bool LoadEstimationEnabled = true;
bool VoltageHolderEnabled = false;
bool Synchronous = true;
bool RecentReference = true;
Protection::Problem protection = Protection::NONE;
ConverterID activeConverter = ID_BuckBoost;
ControlStrategy controlStrategy = CS_DISCRETE_THEOREM_1;
Manager::OperationState *CurrentOperationState;
Equilibrium::EquilibriumMethod CorrectionMethod = Equilibrium::NONE;
DAC_PWM::Channel dacChannel = DAC_PWM::CH_CONTROLE;


//
// Measurement and Statistics Variables
//
Uint32 InterruptionCounter = 0;
Uint32 iterations_since_switch = 0;
Uint32 SwitchCounter = 0;
Uint32 SwitchingFrequency, ADCFrequency;
double VoltageMinimum, VoltageMaximum, VoltageRipple;
Uint32 stateCounter[4] = {0, 0, 0, 0};
double stateDutyCycle[4] = {0, 0, 0, 0};

double DutyCycle = 0;

bool OutputLoadStep = false;

//
// Variables used during debug
//
double *Vin, *Vout, *IL, *Iout;

void main(void)
{
    InitSysCtrl();          // Inicializa��es b�sicas
    InitGpio();             // Inicializa��es padr�o para as GPIOs
    InitIpc();              // Inicializa��es para comunica��o entre as CPUs
    InitCpuTimers();        // Inicializa��es do timer 0, 1 e 2

    //
    // Step 2. Give control of SPI-A to CPU2
    //
    EALLOW;
    DevCfgRegs.CPUSEL6.bit.SPI_B = 1;
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
    CpuSysRegs.SECMSEL.bit.PF1SEL = 1;      // Set EPWM Secondary Master to DMA
    EDIS;

    //
    // Step 3. Send IPC to CPU2 telling it to proceed with configuring the SPI
    //
    IpcRegs.IPCSET.bit.IPC0 = 1;

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
//    PieVectTable.EPWM = &ePWM1_TZ_isr;
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
    Switch::ConfigureGPIO();
    Equilibrium::Configure();
    ReferenceUpdate::Configure();
    IPC::Configure();
    DAC_PWM::Configure();


    EALLOW;
    IpcRegs.IPCSET.bit.IPC1 = 1;
    EDIS;

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Vout_mean = Sensor::GetOutput();
    Iout = Sensor::GetOutputCurrent();
    loadResistance = Sensor::GetLoadResistance();

    Vin = u;
    Vout = X+1;
    IL = X;

    Xe = Equilibrium::GetEquilibrium();
    Xe_o = Equilibrium::GetOriginalEquilibrium();
    Equilibrium::UpdateEquilibrium(*u, *loadResistance);
    Equilibrium::UpdateOriginalEquilibrium(*u, *loadResistance);

    CurrentOperationState = Manager::GetCurrentState();
    Manager::ChangeController(controlStrategy);

    LoadConverterController();

    ConfigureCPU02();

    Timer::SystemEvaluation_Start();
    Timer::ReferenceUpdate_Start();

    DESLISGATZSENSORES;

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    for(;;)
    {
        IPC::CPUCommunication();

        Manager::ExecuteCommand();

        if (*CurrentOperationState == Manager::OS_CHANGING_CONVERTER_CONTROLLER)
        {
            LoadConverterController();
            Manager::CompleteConverterControllerChange();
        }
        else if (*CurrentOperationState == Manager::OS_STARTING_PRE_LOAD ||
                 *CurrentOperationState == Manager::OS_ENDING_PRE_LOAD)
        {
            LoadConverterController();
            Manager::ContinuePreLoad(true);
        }

        Relay::StepOutputLoad(OutputLoadStep);
    }
}


void ConfigureCPU02(void)
{

    //
    // Give Memory Access to GS1 SARAM to CPU02
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1 &
            MemCfgRegs.GSxMSEL.bit.MSEL_GS12 &
            MemCfgRegs.GSxMSEL.bit.MSEL_GS13))
    {
        EALLOW;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
        EDIS;
    }
}


void LoadConverterController(void)
{
    switch(activeConverter)
    {
    case ID_Buck:
        sys = Buck::GetSys();
        dsys = Buck::GetDiscreteSys();
        limitCycle = Buck::GetLimitCycle();
        Buck::GetP(P);
        break;
    case ID_Boost:
        sys = Boost::GetSys();
        dsys = Boost::GetDiscreteSys();
        limitCycle = Boost::GetLimitCycle();
        Boost::GetP(P);
        break;
    case ID_BuckBoost:
        sys = BuckBoost::GetSys();
        dsys = BuckBoost::GetDiscreteSys();
        limitCycle = BuckBoost::GetLimitCycle();
        BuckBoost::GetP(P);
        break;
    case ID_BuckBoost3:
        sys = BuckBoost3::GetSys();
        dsys = BuckBoost3::GetDiscreteSys();
        limitCycle = BuckBoost3::GetLimitCycle();
        BuckBoost3::GetP(P);
        break;
    }

    if (Controller::isClassicControl(controlStrategy))
    {
        VoltageController::LoadController();
        VoltageCurrentController::LoadController();
        StateFeedbackH2Controller::LoadController();
        Switch::ConfigurePWM();
    }

    if (Controller::isSwitchedControl(controlStrategy))
    {
        ReferenceUpdate::LoadController();
        PartialInformation::Configure();
        CurrentCorrection::Configure();
    }
}


//
// Interrup��o executada a cada 100ms para medi��es temporais
//
void CalculateDutyCycle(void)
{
    int i;

    if (Controller::isClassicControl(controlStrategy))
    {
        switch(activeConverter)
        {
        case ID_Buck:
            stateDutyCycle[0] = 0;
            stateDutyCycle[1] = 100*DutyCycle;
            stateDutyCycle[2] = 100*(1-DutyCycle);
            stateDutyCycle[3] = 0;
            break;
        case ID_Boost:
            stateDutyCycle[0] = 0;
            stateDutyCycle[1] = 100*(1-DutyCycle);
            stateDutyCycle[2] = 0;
            stateDutyCycle[3] = 100*DutyCycle;
            break;
        case ID_BuckBoost:
            stateDutyCycle[0] = 0;
            stateDutyCycle[1] = 0;
            stateDutyCycle[2] = 100*(1-DutyCycle);
            stateDutyCycle[3] = 100*DutyCycle;
            break;
        default:
            break;

        }
    }

    else if (Controller::isSwitchedControl(controlStrategy))
    {
        for (i=0;i<4;i++)
        {
            stateDutyCycle[i] = (double) (100*stateCounter[i])/InterruptionCounter;
            stateCounter[i] = 0;
        }
    }
}


//
// Interrup��o executada a cada 100ms para medi��es temporais
//
__interrupt void Interruption_SystemEvaluation(void)
{
    static Uint32 i = 0;
    CpuTimer1.InterruptCount++;

    // Medi��es v�o ser a cada 100ms
    if (Controller::isSwitchedControl(controlStrategy))
        SwitchingFrequency = (10*SwitchCounter)/2;
    else if (Controller::isClassicControl(controlStrategy) && *CurrentOperationState == Manager::OS_RUNNING)
        SwitchingFrequency = (int) (25000000/SWITCH_PWM_TBPRD);
    else
        SwitchingFrequency = 0;

    CalculateDutyCycle();

    VoltageRipple = VoltageMaximum - VoltageMinimum;
    VoltageMaximum = -1e7;
    VoltageMinimum = 1e7;

    ADCFrequency = (10*InterruptionCounter);
    InterruptionCounter = 0;
    SwitchCounter = 0;

    if(i%10==0)
        Manager::ContinuePreLoad(false);

    i++;
}


//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_ReferenceUpdate(void)
{
    static Uint32 i = 0;
    CpuTimer2.InterruptCount++;

    if (RecentReference)
    {
        if (i < 1000)
            i++;
        else
        {
            RecentReference = false;
            i = 0;
        }
    }

    if (*CurrentOperationState == Manager::OS_PRE_LOAD
            && Vref <= *Vin)
    {
        Vref += 0.01;
    }

    switch(CorrectionMethod)
    {
    case Equilibrium::REFERENCE_UPDATE:
        ReferenceUpdate::UpdateReference(*Vout_mean, *u, *loadResistance, !RecentReference);
        break;
    case Equilibrium::PARTIAL_INFORMATION:
        PartialInformation::UpdateReference(*IL);
        break;
    case Equilibrium::CURRENT_CORRECTION:
        CurrentCorrection::UpdateReference(*Vout_mean, *u, *loadResistance, !RecentReference);
        break;
    case Equilibrium::NONE:
    default:
        Equilibrium::UpdateEquilibrium(*u, *loadResistance);
        break;
    }
    Equilibrium::UpdateOriginalEquilibrium(*u, *loadResistance);
}


//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_Sensor(void)
{
    TOGGLEBKR;
    LIGABKR2;
    GpioDataRegs.GPASET.bit.TST = 1;
    InterruptionCounter++;

    //
    // Read Sensors
    //
    Sensor::ReadADCResult();

    // If no protection was already enabled, check protections
    if (protection ==  Protection::NONE)
        protection = Protection::CheckProtections(*Vin, *Vout, *IL, *Iout);

    // If necessary, protect system
    if(protection != Protection::NONE ||
        (!ConverterEnabled &&
        (*CurrentOperationState!=Manager::OS_STARTING_PRE_LOAD && *CurrentOperationState!=Manager::OS_ENDING_PRE_LOAD)))
    {
        Protection::ProtectSystem();
    }
    else
    {
        if (isSwitchedControl(controlStrategy))
            SwitchedControl();
        else if (isClassicControl(controlStrategy))
            ClassicControl();

    }

    stateCounter[Switch::GetState() + 1]++;
    if(*Vout < VoltageMinimum) VoltageMinimum = *Vout;
    if(*Vout > VoltageMaximum) VoltageMaximum = *Vout;

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
    GpioDataRegs.GPACLEAR.bit.TST = 1;

    DAC_PWM::SendData(dacChannel);

    DESLIGABKR2;
    return;
}


void SwitchedControl(void)
{
    //
    // Run Switching Rule
    //
    switch(controlStrategy)
    {
    case CS_CONTINUOUS_THEOREM_1:
        BestSubsystem = SwitchingRule1::SwitchingRule(sys, P, X, Xe, *u, *loadResistance);
        break;
    case CS_CONTINUOUS_THEOREM_2:
        BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u, *loadResistance);
        break;
    case CS_DISCRETE_THEOREM_1:
        BestSubsystem = DiscreteSwitchingRule1::SwitchingRule(sys, dsys, P, X, Xe, Xe_o, *u, *loadResistance);
        break;
    case CS_LIMIT_CYCLE_COST:
    case CS_LIMIT_CYCLE_H2:
        BestSubsystem = LimitCycleRuleCost::SwitchingRule(dsys, limitCycle, X);
        break;
    case CS_LIMIT_CYCLE_Hinf:
        BestSubsystem = LimitCycleRuleHinf::SwitchingRule(dsys, limitCycle, X);
        break;
    default:
        break;
    }

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
    case ID_BuckBoost3:
        SwitchState = BuckBoost3::SubSystem2SwitchState(BestSubsystem);
        break;
    default:
        SwitchState = DISBALE_SWITCHES;
        break;
    }


    switch(*CurrentOperationState)
    {
    case Manager::OS_RUNNING:
    case Manager::OS_PRE_LOAD:
        break;

    case Manager::OS_STARTING_PRE_LOAD:
        SwitchState = DISBALE_SWITCHES;
        break;

    case Manager::OS_ENDING_PRE_LOAD:
        SwitchState = 0;
        break;

    case Manager::OS_OFF:
    default:
        SwitchState = DISBALE_SWITCHES;
        break;
    }

    Synchronous = Manager::SynchronousOperation(*IL);

    //
    // Signal to the gate
    //
    SwitchCounter += Switch::SetState(SwitchState, Synchronous);
}


void ClassicControl(void)
{
    //
    // Run Switching Rule
    //
    switch(controlStrategy)
    {
    case CS_CLASSIC_PWM:
        DutyCycle = VoltageController::Update(Vref, *Vout, *Vin);
        break;
    case CS_CLASSIC_VC_PWM:
        DutyCycle = VoltageCurrentController::Update(Vref, *Vout, *IL, *Vin);
        break;
    case CS_STATE_H2_PWM:
        DutyCycle = StateFeedbackH2Controller::Update(Xe, X);
        break;
    default:
        break;
    }

    switch(*CurrentOperationState)
    {
    case Manager::OS_RUNNING:
    case Manager::OS_PRE_LOAD:
        Switch::UpdateDutyCycle(DutyCycle);
        break;

    case Manager::OS_ENDING_PRE_LOAD:
        DutyCycle = 0;
        Switch::UpdateDutyCycle(DutyCycle);
        break;

    case Manager::OS_STARTING_PRE_LOAD:
    case Manager::OS_OFF:
    default:
        DutyCycle = 0;
        break;
    }
}
