#ifndef SRC_CONTROLLER_SWITCHING_RULE_1_H_
#define SRC_CONTROLLER_SWITCHING_RULE_1_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace SwitchingRule1
{
    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u, double Rom);

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u, double Rom);

    void EvaluateModelR(SubSystem *subSys, double Rom);
}

#endif /* SRC_CONTROLLER_SWITCHING_RULE_1_H_ */
