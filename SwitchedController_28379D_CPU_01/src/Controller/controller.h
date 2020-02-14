#ifndef SRC_CONTROLLER_CONTROLLER_H_
#define SRC_CONTROLLER_CONTROLLER_H_


namespace Controller
{
    enum ControlStrategy
    {
        CS_CLASSIC_PWM = 0,
        CS_CONTINUOUS_THEOREM_1 = 1,
        CS_CONTINUOUS_THEOREM_2 = 2,
        CS_DISCRETE_THEOREM_1 = 3,
        CS_CLASSIC_VC_PWM = 4,
        CS_LIMIT_CYCLE_COST = 5,
        CS_LIMIT_CYCLE_H2 = 6,
        CS_LIMIT_CYCLE_Hinf = 7,
        CS_STATE_H2_PWM = 8,
    };

    bool isSwitchedControl(ControlStrategy controlStrategy);

    bool isClassicControl(ControlStrategy controlStrategy);
}


#endif /* SRC_CONTROLLER_CONTROLLER_H_ */
