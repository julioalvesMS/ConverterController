#include <src/Config/PWM_DAC.h>

void enviar_dac_pwm_4Canais(Uint16 sdata0,Uint16 sdata1,Uint16 sdata2,Uint16 sdata3)
{
    EPwm8Regs.CMPA.bit.CMPA = sdata0;
    EPwm8Regs.CMPB.bit.CMPB = sdata1;
    EPwm7Regs.CMPA.bit.CMPA = sdata2;
    EPwm7Regs.CMPB.bit.CMPB = sdata3;
}

