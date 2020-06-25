#ifndef SRC_SWITCH_SWITCH_H_
#define SRC_SWITCH_SWITCH_H_

#include "F28x_Project.h"
#include <src/settings.h>
#include <src/Converter/base_converter.h>

#define DISBALE_SWITCHES -1

extern BaseConverter::ConverterID activeConverter;

namespace Switch
{
    void ConfigureGPIO(void);

    void ConfigurePWM(void);

    bool SetState(int state, bool synchronous);

    short GetState(void);

    void EnablePWM(void);

    void DisablePWM(void);

    void UpdateDutyCycle(double DutyCycle);
}

#endif /* SRC_SWITCH_SWITCH_H_ */
