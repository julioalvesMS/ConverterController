#include <src/HAL/PWM/pwm.h>

namespace PWM
{
    //
    // Configure - Configure EPWM SOC and compare values
    //
    void ADC_Configure(void)
    {
        EALLOW;
        //
        // PWM for ADC
        //
        EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
        EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
        EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
        EPwm1Regs.CMPA.bit.CMPA = 128;   // Set compare A value to 2048 counts
        EPwm1Regs.TBPRD = 256;           // Set period to 4096 counts

        EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
        EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
        EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;
        EPwm1Regs.TBCTL.bit.CTRMODE = 3;    // freeze counter
        EDIS;
    }

    //
    // Configure - Configure EPWM SOC and compare values
    //
    void Switch_Configure(void)
    {
        EALLOW;
        //
        // PWM for switchs
        //
        InitEPwm4Gpio();

        EPwm4Regs.TBPRD = SWITCH_PWM_TBPRD;           // Set timer period
        EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
        EPwm4Regs.TBCTR = 0x0000;                     // Clear counter

        //
        // Setup TBCLK
        //
        EPwm4Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;       // set Immediate load
        EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;     // Count up
        EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
        EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
        EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;

        EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
        EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
        EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

        //
        // Setup compare
        //
        EPwm4Regs.CMPA.bit.CMPA = 8;

        //
        // Set actions
        //
        EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM4A on Zero
        EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;

        EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;          // Set PWM4B on Zero
        EPwm4Regs.AQCTLB.bit.CAU = AQ_SET;

        //
        // Active Low PWMs - Setup Deadband
        //
        EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
        EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
        EPwm4Regs.DBRED.bit.DBRED = 16;
        EPwm4Regs.DBFED.bit.DBFED = 16;
        EDIS;
    }


    //
    // Start - Start ePWM
    //
    void ADC_Start(void)
    {
        //
        // start ePWM
        //
        EALLOW;
        EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
        EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
        EDIS;
    }

    void Switch_UpdateDutyCycle(double DutyCycle)
    {
        EPwm4Regs.CMPA.bit.CMPA = ((int) SWITCH_PWM_TBPRD * DutyCycle);
    }
}
