#include <src/DAC/dac.h>

extern double *Vin, *Vout, *IL, *Iout, Vref;
extern int ADC_Vout, ADC_Vin, ADC_IL, ADC_Iout;

namespace DAC_PWM
{
    void Configure(void)
    {
        EALLOW;
        EPwm7Regs.TZCLR.bit.OST = 1;
        EPwm7Regs.TZEINT.bit.OST = 1;

        EPwm8Regs.TZCLR.bit.OST = 1;
        EPwm8Regs.TZEINT.bit.OST = 1;
        EDIS;
    }

    void SendData(Channel opt)
    {
        switch(opt)
        {
        case CH_ADC:
            enviar_dac_pwm_4Canais(ADC_RESULT_VOUT, ADC_RESULT_VIN, ADC_RESULT_IL, ADC_RESULT_IOUT);
            break;
        case CH_CONTROLE:
            enviar_dac_pwm_4Canais((*Vout)*0.67, Vref*0.67, (*Vin)*0.67, ((*IL)+5)*3.3);
            break;
        default:
            break;
        }
    }
}
