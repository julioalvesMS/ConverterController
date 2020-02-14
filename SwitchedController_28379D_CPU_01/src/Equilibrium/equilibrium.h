#ifndef SRC_EQUILIBRIUM_H_
#define SRC_EQUILIBRIUM_H_

#include <math.h>
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/Converter/buck.h>
#include <src/Converter/boost.h>
#include <src/Converter/buck_boost.h>
#include <src/Converter/buck_boost_3.h>

using namespace BaseConverter;
using namespace ConverterBuck;
using namespace ConverterBoost;
using namespace ConverterBuckBoost;
using namespace ConverterBuckBoost3;

extern ConverterID activeConverter;
extern double Vref;

namespace Equilibrium
{
    enum EquilibriumMethod
    {
        NONE = 0,
        REFERENCE_UPDATE = 1,
        PARTIAL_INFORMATION = 2,
        CURRENT_CORRECTION = 3,
    };

    void Configure(void);

    double* GetEquilibrium(void);

    void UpdateEquilibrium(double u);

    double EstimateEquilibriumCurrent(double Ve, double u);
}


#endif /* SRC_EQUILIBRIUM_H_ */
