#ifndef SRC_SWITCHING_RULE_2_H_
#define SRC_SWITCHING_RULE_2_H_

#include <src/Math/matrix.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/Common/constants.h>

using namespace Math;
using namespace SwitchedSystem;

namespace SwitchingRule2
{
    int switchingRule(System *sys, Matrix *P, Vector *X, Vector *Xe, double u);

    double evaluateSubSystem(SubSystem *subSys, Matrix *P, Vector *X, Vector *Xe, double u);
}

#endif /* SRC_SWITCHING_RULE_2_H_ */