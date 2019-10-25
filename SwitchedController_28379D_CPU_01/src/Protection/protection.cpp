#include <src/Protection/protection.h>

namespace Protection
{
    Problem CheckProtections(double Vin, double Vout, double IL, double Iout)
    {
        Problem fault = NONE;

        if (Vin > PROTECTION_VIN_MAX)           fault = FAULT_INPUT_VOLTAGE_MAX;
        else if (Vout > PROTECTION_VOUT_MAX)    fault = FAULT_OUTPUT_VOLTAGE_MAX;
        else if (IL > PROTECTION_IL_MAX)        fault = FAULT_INDUCTOR_CURRENT_MAX;
        else if (Iout > PROTECTION_IOUT_MAX)    fault = FAULT_OUTPUT_CURRENT_MAX;

        return fault;
    }

    void ProtectSystem(void)
    {
        Switch::SetState(DISBALE_SWITCHES);
        Switch::DisablePWM();

        Manager::DisableOperation();
    }
}
