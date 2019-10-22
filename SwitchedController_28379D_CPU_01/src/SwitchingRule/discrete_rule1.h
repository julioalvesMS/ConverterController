#ifndef SRC_CORE_SWITCHING_RULE_1_H_
#define SRC_CORE_SWITCHING_RULE_1_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace DiscreteSwitchingRule1
{
    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER], double d, double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u);

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER], double d, double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u);
}

#endif /* SRC_CORE_SWITCHING_RULE_1_H_ */
