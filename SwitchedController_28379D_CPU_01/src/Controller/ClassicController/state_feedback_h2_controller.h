#ifndef SRC_CLASSIC_CONTROLLER_STATE_FEEDBACK_H2_H_
#define SRC_CLASSIC_CONTROLLER_STATE_FEEDBACK_H2_H_

#include <math.h>
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/Converter/buck.h>
#include <src/Converter/boost.h>
#include <src/Converter/buck_boost.h>

extern BaseConverter::ConverterID activeConverter;

using namespace ConverterBuck;
using namespace ConverterBoost;
using namespace ConverterBuckBoost;

namespace StateFeedbackH2Controller
{
    void LoadController(void);

    double Update(double Xe[SYSTEM_ORDER], double X[SYSTEM_ORDER]);
}

#endif /* SRC_CLASSIC_CONTROLLER_STATE_FEEDBACK_H2_H_ */
