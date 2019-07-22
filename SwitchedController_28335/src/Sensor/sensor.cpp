#include <src/Sensor/sensor.h>

extern void InitAdc(void);

using namespace Math;

namespace Sensor
{
    static Vector s_state;

    static double input_voltage = 0;

    void init(void)
    {
        InitAdc();

        //
        // Enable ADCINT in PIE
        //
        PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
        IER |= M_INT1;      // Enable CPU Interrupt 1
        EINT;               // Enable Global interrupt INTM
        ERTM;               // Enable Global realtime interrupt DBGM

        //
        // Configure ADC
        //
        AdcRegs.ADCMAXCONV.all = 0x0002;       // Setup 3 conv's on SEQ1
        AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA0 as 1st SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA1 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA2 as 3nd SEQ1 conv.

        //
        // Enable SOCA from ePWM to start SEQ1
        //
        AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;

        AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

        //
        // Assumes ePWM1 clock is already enabled in InitSysCtrl();
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
        EPwm1Regs.ETSEL.bit.SOCASEL = 2;    // Generate SOCA on first event
        EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
        EPwm1Regs.TBPRD = 0x0BB7;           // Set period for ePWM1
        EPwm1Regs.TBCTL.bit.CTRMODE = 0;    // count up and start

        //
        // Assume the initial state as zero
        //
        s_state.data[0] = 0;
        s_state.data[1] = 0;

    }


    __interrupt
    void isr_interruption(void)
    {
        //
        // Read ADC result
        //
        s_state.data[0] = AdcRegs.ADCRESULT0; // Current
        s_state.data[1] = AdcRegs.ADCRESULT1; // Voltage

        //
        // Reinitialize for next ADC sequence
        //
        AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

        return;
    }

    Vector* getState(void)
    {
        return &s_state;
    }

    double* getInput(void)
    {
        return &input_voltage;
    }
}
