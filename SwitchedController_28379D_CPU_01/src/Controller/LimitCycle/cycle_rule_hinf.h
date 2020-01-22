#ifndef SRC_CONTROLLER_LIMIT_CYCLE_RULE_HINF_H_
#define SRC_CONTROLLER_LIMIT_CYCLE_RULE_HINF_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/SwitchedSystem/cycle_sequence.h>

using namespace SwitchedSystem;
using namespace CycleSequence;

namespace LimitCycleRuleHinf
{
    int SwitchingRule(System *sys, Cycle *cycle, double X[SYSTEM_ORDER]);

    double EvaluateSubSystem(SubSystem *subSys, int i, Cycle *cycle);
}

#endif /* SRC_CONTROLLER_LIMIT_CYCLE_RULE_HINF_H_ */
