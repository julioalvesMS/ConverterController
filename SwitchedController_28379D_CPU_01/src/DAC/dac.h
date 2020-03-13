#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include <src/settings.h>
#include <src/Config/PWM_DAC.h>

#define DAC_CHANNEL_COUNT 2

namespace DAC_PWM
{
    enum Channel
    {
        CH_ADC = 0,
        CH_CONTROLE = 1,
    };

    void Configure(void);

    void SendData(Channel opt);
}

#endif /* SRC_DAC_H_ */
