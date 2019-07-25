#ifndef SRC_CORE_SWITCHING_RULE_2_H_
#define SRC_CORE_SWITCHING_RULE_2_H_

#include <src/Util/Common/constants.h>
#include <src/Util/Math/matrix.h>
#include <src/Core/SwitchedSystem/switched_system.h>

using namespace Math;
using namespace SwitchedSystem;

namespace SwitchingRule2
{
    int SwitchingRule(System *sys, Matrix *P, Vector *X, Vector *Xe, double u);

    double EvaluateSubSystem(SubSystem *subSys, Matrix *P, Vector *X, Vector *Xe, double u);
}

#endif /* SRC_CORE_SWITCHING_RULE_2_H_ */
