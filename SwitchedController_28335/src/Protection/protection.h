#ifndef SRC_PROTECTION_H_
#define SRC_PROTECTION_H_

#include <src/settings.h>
#include <src/Switch/switch.h>

namespace Protection
{
    enum Problem
    {
        NONE = 0,
        FAULT_OUTPUT_CURRENT_MAX = 1,
        FAULT_OUTPUT_VOLTAGE_MAX = 2,
        FAULT_INDUCTOR_CURRENT_MAX = 3,
        FAULT_INPUT_VOLTAGE_MAX = 4,
        FAULT_EMERGENCY_STOP = 5
    };


    Problem CheckProtections(double Vin, double Vout, double IL, double Iout);

    void ProtectSystem(void);
}

#endif /* SRC_PROTECTION_H_ */
