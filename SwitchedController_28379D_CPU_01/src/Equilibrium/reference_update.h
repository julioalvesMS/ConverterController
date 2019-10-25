#ifndef SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <math.h>
#include "F28x_Project.h"
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>
#include <src/settings.h>

using namespace BaseConverter;
extern ConverterID activeConverter;
extern bool ReferenceControlerEnabled;

namespace Equilibrium
{
    const double pid_kp = REFERENCE_CONTROLLER_PID_KP;
    const double pid_ki = REFERENCE_CONTROLLER_PID_KI;

    void Configure(void);

    void UpdateReference(double Vref, double Vout, double u);

    double* GetReference(void);

    void ResetController(void);
}

#endif /* SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
