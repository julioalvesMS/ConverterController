#ifndef SRC_CORE_SWITCHING_RULE_2_H_
#define SRC_CORE_SWITCHING_RULE_2_H_

#include <src/Util/Common/constants.h>
#include <src/Core/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace SwitchingRule2
{
    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u);

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u);
}

#endif /* SRC_CORE_SWITCHING_RULE_2_H_ */
