#ifndef SRC_CLASSIC_CONTROLLER_PID_H_
#define SRC_CLASSIC_CONTROLLER_PID_H_

#include <math.h>
#include <src/Common/constants.h>
#include <src/Converter/base_converter.h>

extern BaseConverter::ConverterID activeConverter;

namespace PID
{
    const double pid_kp = 0.5;
    const double pid_ki = 0.1;

    double Update(double Vref, double Vout, double Vin);

    void ResetController(void);
}

#endif /* SRC_CLASSIC_CONTROLLER_PID_H_ */
