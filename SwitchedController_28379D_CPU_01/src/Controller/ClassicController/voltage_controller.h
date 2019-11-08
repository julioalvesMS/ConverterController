#ifndef SRC_CLASSIC_CONTROLLER_VOLTAGE_H_
#define SRC_CLASSIC_CONTROLLER_VOLTAGE_H_

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

namespace VoltageController
{
    void LoadController(void);

    double Update(double Vref, double Vout, double Vin);

    void ResetController(void);
}

#endif /* SRC_CLASSIC_CONTROLLER_VOLTAGE_H_ */
