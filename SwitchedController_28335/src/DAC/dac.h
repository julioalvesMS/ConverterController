#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include <src/settings.h>
#include <src/Config/SPI_DAC.h>

extern double *Vin, *Vout, *IL, *Iout, Vref;

#define DAC_CHANNEL_COUNT 2

namespace DAC_SPI
{
    enum Channel
    {
        CH_ADC = 0,
        CH_CONTROLE = 1,
    };

    void SendData(Channel opt);
}

#endif /* SRC_DAC_H_ */
