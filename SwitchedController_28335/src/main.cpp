
//
// Includes
//
#include "DSP28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/CONFIG_GPIO_V4_CCARDV2.h>
#include <src/Config/DEFINESCONTROLCARDV2.h>
#include <src/Controller/switched_controller.h>
#include <src/Converter/buck.h>
#include <src/DAC/dac.h>
#include <src/Equilibrium/reference_update.h>
#include <src/Protection/protection.h>
#include <src/Sensor/sensor.h>
#include <src/Serial/communication.h>
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
__interrupt void Interruption_SendCommunication(void);
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
bool ConverterEnabled = true;

DAC_SPI::Channel DacChannel = DAC_SPI::CH_ADC;

//
// Variables used during debug
//
double *Vin, *Vout, *IL, *Iout;

void main(void)
{
    int loop_count;

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
    PieVectTable.ADCINT = &(Interruption_Sensor);
    PieVectTable.XINT13 = &(Interruption_SendCommunication);
    PieVectTable.TINT2 = &(Interruption_ReferenceUpdate);
    EDIS;

    IER |= M_INT1;      // Enable CPU Interrupt 1
    IER |= M_INT2;      // Enable CPU Interrupt 1
    IER |= M_INT13;     // Timer01
    IER |= M_INT14;     // Timer02

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  // ADC
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer

    Timer::Configure();
    Sensor::Configure();
    Switch::Configure();
    Equilibrium::Configure();
    Communication::Configure();

    X = Sensor::GetState();
    u = Sensor::GetInput();
    Vout_mean = Sensor::GetOutput();
    Iout = Sensor::GetOutputCurrent();

    Xe = Equilibrium::GetReference();
    Equilibrium::UpdateReference(Vref, *Vout_mean, *u);

    sys = Buck::GetSys();
    Controller::GetP(P);

    DebugVariables();

    //Timer::Communication_Start();
    Timer::ReferenceUpdate_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM


    for(loop_count=0;;loop_count++)
    {
        Communication::ReceiveMessage();

        //
        // Send to DAC
        //
        DAC_SPI::SendData(DacChannel);

        if(loop_count > 1500)
        {
            //
            // Send to PC
            //
            Communication::SendMessage();

            if (loop_count >= 1500 + MESSAGES_COUNT)
                loop_count = 0;
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
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_SendCommunication(void)
{
    CpuTimer1.InterruptCount++;

    Communication::SendMessage();
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
#if SWITCHING_RULE == 1
        BestSubsystem = SwitchingRule1::SwitchingRule(sys, P, X, Xe, *u);
#elif SWITCHING_RULE == 2
        BestSubsystem = SwitchingRule2::SwitchingRule(sys, P, X, Xe, *u);
#endif

        //
        // Signal to the gate
        //
        Switch::SetState(BestSubsystem);
    }


    //
    // Reinitialize for next ADC sequence
    //
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}
