#include <src/Controller/controller.h>

namespace Controller
{
    bool isSwitchedControl(ControlStrategy controlStrategy)
    {
        if(controlStrategy == CS_CONTINUOUS_THEOREM_1 ||
                controlStrategy == CS_CONTINUOUS_THEOREM_2 ||
                controlStrategy == CS_DISCRETE_THEOREM_1 ||
                controlStrategy == CS_LIMIT_CYCLE_COST ||
                controlStrategy == CS_LIMIT_CYCLE_H2 ||
                controlStrategy == CS_LIMIT_CYCLE_Hinf)
            return true;
        return false;
    }

    bool isClassicControl(ControlStrategy controlStrategy)
    {
        if(controlStrategy == CS_CLASSIC_PWM ||
                controlStrategy == CS_CLASSIC_VC_PWM ||
                controlStrategy == CS_STATE_H2_PWM)
            return true;
        return false;
    }
}
