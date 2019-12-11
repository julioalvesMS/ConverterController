#include <src/DAC/dac.h>

extern double Vin, Vout, IL, Iout, Vref;
extern int ADC_Vout, ADC_Vin, ADC_IL, ADC_Iout;

namespace DAC_SPI
{
    void Configure(void)
    {
        spi_fifo_init();    // Configura SPI-FIFO DA DAC
        spi_init();         // Inicializa SPI-FIFO DA DAC
    }

    void SendData(Channel opt)
    {
        switch(opt)
        {
        case CH_ADC:
            enviar_dac_spi_4Canais(ADC_Vout, ADC_Vin, ADC_IL, ADC_Iout);
            break;
        case CH_CONTROLE:
            enviar_dac_spi_4Canais(((Uint32) 4095*Vout)/60, ((Uint32) 4095*Vref)/60, ((Uint32) 4095*Vin/60), ADC_IL);
            break;
        default:
            break;
        }
    }
}
