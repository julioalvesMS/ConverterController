
#include "DSP28x_Project.h"
#include "DSP2833x_Device.h"

#include <src/Math/matrix.h>
#include <src/Sensor/sensor.h>
#include <src/Switch/switch.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/Equilibrium/reference_update.h>
#include <src/SwitchingRule/rule2.h>
#include <src/Converter/buck.h>
#include <src/Controller/switched_controller.h>

using namespace Math;
using namespace SwitchedSystem;

extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);


void init(void);
void initInterruption(void);

void main(void)
{
    Vector *X, *Xe;
    Matrix *P;
    double *u;
    int k;

    double Vref = 60;

    System* sys = Buck::getSys();

    //
    // General Board initializations
    //
    init();
    initInterruption();

    //
    // Specific devices initializations
    //
    Sensor::init();
    Switch::init();

    Equilibrium::init();

    X = Sensor::getState();
    u = Sensor::getInput();
    Xe = Equilibrium::getReference();

    P = Controller::getP();

    /* =============== */
    *u = 400;
    Vref = 100;
    Xe->data[0] = 1.0331;
    Xe->data[1] = 100;
    /* =============== */

    EINT;

    while(1)
    {
//        Equilibrium::referenceUpdate(Vref, X, *u);

        k = SwitchingRule2::switchingRule(sys, P, X, Xe, *u);

        Switch::updateState(k);
    }
}


void init(void)
{
    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    //
    InitSysCtrl();

    EALLOW;
    #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
        //
        // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
        //
        #define ADC_MODCLK 0x3
    #endif
    #if (CPU_FRQ_100MHZ)
        //
        // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
        //
        #define ADC_MODCLK 0x2
    #endif
    EDIS;

    //
    // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
    // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
    //
    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;

}

void initInterruption(void)
{

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
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
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    // This function is found in DSP2833x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT = &(Sensor::isr_interruption);
    EDIS;    // This is needed to disable write to EALLOW protected registers
}

