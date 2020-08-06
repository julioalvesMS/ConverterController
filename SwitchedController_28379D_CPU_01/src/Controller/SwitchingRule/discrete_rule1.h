#ifndef SRC_CONTROLLER_DISCRETE_SWITCHING_RULE_1_H_
#define SRC_CONTROLLER_DISCRETE_SWITCHING_RULE_1_H_

#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace BaseConverter;
using namespace SwitchedSystem;

extern ConverterID activeConverter;

namespace DiscreteSwitchingRule1
{
    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u);

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u);

    void CalculateDiscreteSystem(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u);

    void CalculateLambdas(double Xe[SYSTEM_ORDER], double u);
}

#endif /* SRC_CONTROLLER_DISCRETE_SWITCHING_RULE_1_H_ */
