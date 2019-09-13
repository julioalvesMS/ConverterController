#include <src/DAC/dac.h>

namespace DAC_SPI
{
    void SendData(Channel opt)
    {
        switch(opt)
        {
        case CH_ADC:
            enviar_dac_spi_4Canais(ADC_RESULT_VOUT, ADC_RESULT_VIN, ADC_RESULT_IL, ADC_RESULT_IOUT);
            break;
        case CH_CONTROLE:
            enviar_dac_spi_4Canais((int) 4095*(*Vout)/60, (int) 4095*(Vref)/60, (int) 4095*(*Vin)/60, ADC_RESULT_IL);
            break;
        default:
            break;
        }
    }
}
