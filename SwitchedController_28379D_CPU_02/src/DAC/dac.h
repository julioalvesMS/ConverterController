#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include <src/settings.h>
#include <src/Config/SPI_DAC.h>

#define DAC_CHANNEL_COUNT 4

namespace DAC_SPI
{
    enum Channel
    {
        CH_ADC = 0,
        CH_CONTROLE = 1,
        CH_TRIGGER = 2,
        CH_TRIGGER_HOLD = 3,
    };

    void Configure(void);

    void SendData(Channel opt);
}

#endif /* SRC_DAC_H_ */
