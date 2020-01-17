#ifndef SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <math.h>
#include "F28x_Project.h"
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/Converter/buck.h>
#include <src/Converter/boost.h>
#include <src/Converter/buck_boost.h>
#include <src/Converter/buck_boost_3.h>
#include <src/Equilibrium/equilibrium.h>
#include <src/settings.h>

using namespace BaseConverter;
using namespace ConverterBuck;
using namespace ConverterBoost;
using namespace ConverterBuckBoost;
using namespace ConverterBuckBoost3;

extern ConverterID activeConverter;
extern double Vref;
extern double *Xe;

namespace ReferenceUpdate
{
    void Configure(void);

    void LoadController(void);

    void UpdateReference(double Vout, double u);

    void ResetController(void);
}

#endif /* SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
