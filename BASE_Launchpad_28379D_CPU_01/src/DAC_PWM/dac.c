#include <src/DAC_PWM/dac.h>

void DAC_PWM_Configure(void)
{
    EALLOW;
    EPwm7Regs.TZCLR.bit.OST = 1;
    EPwm7Regs.TZEINT.bit.OST = 1;

    EPwm8Regs.TZCLR.bit.OST = 1;
    EPwm8Regs.TZEINT.bit.OST = 1;
    EDIS;
}

void DAC_PWM_SendData(Uint16 opt)
{
    switch(opt)
    {
    case 0:
        enviar_dac_pwm_4Canais(Ix,Iy,Iz,Icc2);
        break;
    case 1:
        enviar_dac_pwm_4Canais(Vx,Vy,Vz,Vcc2);
        break;
    case 2:
        enviar_dac_pwm_4Canais(Iu,Iv,Iw,Icc1);
        break;
    case 3:
        enviar_dac_pwm_4Canais(Vu,Vv,Vw,Vcc1);
        break;
    default:
        break;
    }
}
