#include <src/Switch/switch.h>

namespace Switch
{

    short current_switch_state = DISBALE_SWITCHES;

    void ConfigureGPIO()
    {

        EALLOW;

        // PULL DOWN
        GpioCtrlRegs.GPAPUD.bit.S1 = 1;
        GpioCtrlRegs.GPAPUD.bit.S2 = 1;
        GpioCtrlRegs.GPAPUD.bit.S3 = 1;
        GpioCtrlRegs.GPAPUD.bit.S4 = 1;
        GpioCtrlRegs.GPAPUD.bit.TST = 1;

        // UTILIZAR PORTA COMO GPIO
        GpioCtrlRegs.GPAMUX1.bit.S1 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S2 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S3 = 0;
        GpioCtrlRegs.GPAMUX1.bit.S4 = 0;
        GpioCtrlRegs.GPAMUX1.bit.TST = 0;

        // CONFIGURA COMO SAÍDA
        GpioCtrlRegs.GPADIR.bit.S1 = 1;
        GpioCtrlRegs.GPADIR.bit.S2 = 1;
        GpioCtrlRegs.GPADIR.bit.S3 = 1;
        GpioCtrlRegs.GPADIR.bit.S4 = 1;
        GpioCtrlRegs.GPADIR.bit.TST = 1;

        EDIS;

    }

    //
    // Configure - Configure EPWM SOC and compare values
    //
    void ConfigurePWM(void)
    {
        InitEPwm5Gpio();
        InitEPwm6Gpio();

        EPwm5Regs.TBPRD=SWITCH_PWM_TBPRD;   // 10 kHz
        EPwm6Regs.TBCTR = 0x0000;           // Clear counter

        EPwm6Regs.TBPRD=SWITCH_PWM_TBPRD;   // SWITCH_PWM_TBPRD
        EPwm6Regs.TBCTR = 0x0000;           // Clear counter

        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            EPwm6Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
            EPwm6Regs.AQCTLB.all=0x00600;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLB.all=0x00900;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear

            EPwm6Regs.CMPA.bit.CMPA = 800;                  // S1
            EPwm6Regs.CMPB.bit.CMPB = 800;                  // S2
            EPwm5Regs.CMPB.bit.CMPB = 0;                    // S3
            EPwm5Regs.CMPA.bit.CMPA = SWITCH_PWM_TBPRD;     // S4
            break;

        case BaseConverter::ID_Boost:
            EPwm6Regs.AQCTLA.all=0x0090;        // ZRO=set, PRD-clear
            EPwm6Regs.AQCTLB.all=0x00900;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLB.all=0x00900;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLA.all=0x0060;        // ZRO=set, PRD-clear

            EPwm6Regs.CMPA.bit.CMPA = SWITCH_PWM_TBPRD;     // S1
            EPwm6Regs.CMPB.bit.CMPB = 0;                    // S2
            EPwm5Regs.CMPB.bit.CMPB = 800;                  // S3
            EPwm5Regs.CMPA.bit.CMPA = 800;                  // S4
            break;

        case BaseConverter::ID_BuckBoost:
            EPwm6Regs.AQCTLA.all=0x00900;        // ZRO=set, PRD-clear
            EPwm6Regs.AQCTLB.all=0x00600;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLB.all=0x00900;       // ZRO=set, PRD-clear
            EPwm5Regs.AQCTLA.all=0x00600;        // ZRO=set, PRD-clear

            EPwm6Regs.CMPA.bit.CMPA = SWITCH_PWM_TBPRD;     // S1
            EPwm6Regs.CMPB.bit.CMPB = 800;                  // S2
            EPwm5Regs.CMPB.bit.CMPB = 800;                  // S3
            EPwm5Regs.CMPA.bit.CMPA = SWITCH_PWM_TBPRD;     // S4
            break;

        default:
            break;
        }

        EALLOW;
        EPwm5Regs.TZCLR.bit.OST = 1;
        EPwm5Regs.TZEINT.bit.OST = 1;
        EPwm6Regs.TZCLR.bit.OST = 1;
        EPwm6Regs.TZEINT.bit.OST = 1;
        EDIS;
    }


    bool SetState(int state)
    {
        bool switched = false;

        // If the switch is already ate the desired state, do nothing
        if (state == current_switch_state && current_switch_state!=DISBALE_SWITCHES)
            return switched;

        // Register new state
        current_switch_state = state;

        switched = DeactivateSwitches();
        // Start Deadband Timer
        CpuTimer0Regs.TCR.bit.TSS = 0;

        return switched;
    }


    bool DeactivateSwitches()
    {
        bool switched = true;
        switch(current_switch_state)
        {
        case 0:
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            break;
        case 1:
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            break;
        case 2:
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S4 = 1;
            break;
        case DISBALE_SWITCHES:
        default:
            switched = false;
            GpioDataRegs.GPACLEAR.bit.S1 = 1;
            GpioDataRegs.GPACLEAR.bit.S2 = 1;
            GpioDataRegs.GPACLEAR.bit.S3 = 1;
            GpioDataRegs.GPACLEAR.bit.S4 = 1;
            break;
        }

        return switched;
    }

    void ActivateSwitches(bool synchronous)
    {
        CpuTimer0Regs.TCR.bit.TSS = 1;

        switch(current_switch_state)
        {
        case 0:
            GpioDataRegs.GPASET.bit.S1 = 1;
            GpioDataRegs.GPASET.bit.S4 = 1 && synchronous;
            break;
        case 1:
            GpioDataRegs.GPASET.bit.S2 = 1 && synchronous;
            GpioDataRegs.GPASET.bit.S4 = 1 && synchronous;
            break;
        case 2:
            GpioDataRegs.GPASET.bit.S1 = 1;
            GpioDataRegs.GPASET.bit.S3 = 1;
            break;
        default:
            break;
        }
    }


    short GetState(void)
    {
        return current_switch_state;
    }


    void EnablePWM(void)
    {
        EALLOW;
        EPwm5Regs.TZCLR.bit.OST = 1;
        EPwm5Regs.TZEINT.bit.OST = 1;
        EPwm6Regs.TZCLR.bit.OST = 1;
        EPwm6Regs.TZEINT.bit.OST = 1;

        EPwm5Regs.TZCTL.bit.TZA = 0x01; // fOR FORCE low
        EPwm5Regs.TZCTL.bit.TZB = 0x01; //fOR fORCE loW
        EPwm6Regs.TZCTL.bit.TZA = 0x01; // fOR FORCE low
        EPwm6Regs.TZCTL.bit.TZB = 0x01; //fOR fORCE loW
        EDIS;
    }


    void DisablePWM(void)
    {
        EALLOW;
        EPwm5Regs.TZFRC.bit.OST = 1;
        EPwm6Regs.TZFRC.bit.OST = 1;

        EPwm5Regs.TZCTL.bit.TZA = 0x02; // fOR FORCE low
        EPwm5Regs.TZCTL.bit.TZB = 0x02; //fOR fORCE loW
        EPwm6Regs.TZCTL.bit.TZA = 0x02; // fOR FORCE low
        EPwm6Regs.TZCTL.bit.TZB = 0x02; //fOR fORCE loW
        EDIS;
    }


    void UpdateDutyCycle(double DutyCycle)
    {
        int valueCMP = ((int) SWITCH_PWM_TBPRD * DutyCycle);

        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            EPwm6Regs.CMPA.bit.CMPA = valueCMP;                  // S1
            EPwm6Regs.CMPB.bit.CMPB = valueCMP;                  // S2
            break;

        case BaseConverter::ID_Boost:
            EPwm5Regs.CMPB.bit.CMPB = valueCMP;                  // S3
            EPwm5Regs.CMPA.bit.CMPA = valueCMP;                  // S4
            break;

        case BaseConverter::ID_BuckBoost:
            EPwm6Regs.CMPB.bit.CMPB = valueCMP;                  // S2
            EPwm5Regs.CMPB.bit.CMPB = valueCMP;                  // S3
            break;

        default:
            break;
        }
    }
}
