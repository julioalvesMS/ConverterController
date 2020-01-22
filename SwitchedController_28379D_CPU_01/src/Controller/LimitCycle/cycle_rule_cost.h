#ifndef SRC_CONTROLLER_LIMIT_CYCLE_RULE_COST_H_
#define SRC_CONTROLLER_LIMIT_CYCLE_RULE_COST_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>
#include <src/SwitchedSystem/cycle_sequence.h>

using namespace SwitchedSystem;
using namespace CycleSequence;

namespace LimitCycleRuleCost
{
    int SwitchingRule(System *sys, Cycle *cycle, double X[SYSTEM_ORDER]);

    double EvaluateSubSystem(SubSystem *subSys, int i, CycleStep *cycleStep, CycleStep *nextStep);
}

#endif /* SRC_CONTROLLER_LIMIT_CYCLE_RULE_COST_H_ */
