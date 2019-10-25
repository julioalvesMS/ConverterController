#include <src/Controller/controller.h>

namespace Controller
{
    bool isSwitchedControl(ControlStrategy controlStrategy)
    {
        if(controlStrategy == CS_CONTINUOUS_THEOREM_1 ||
                controlStrategy == CS_CONTINUOUS_THEOREM_2 ||
                controlStrategy == CS_DISCRETE_THEOREM_1)
            return true;
        return false;
    }

    bool isClassicControl(ControlStrategy controlStrategy)
    {
        if(controlStrategy == CS_CLASSIC_PWM)
            return true;
        return false;
    }
}
